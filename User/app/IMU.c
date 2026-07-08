/**
 * IMU.c — MPU6050 + QMC5883P 姿态解算与卡尔曼滤波
 *
 * Ported from the GitHub STM32 reference IMU module to this MSPM0 project.
 * The attitude/filter/data-path logic is kept aligned with the GitHub version;
 * only the I2C address probing is adapted to this project's 7-bit MyI2C/QMC APIs.
 */

#include "IMU.h"
#include "MPU6050.h"
#include "MPU6050_Reg.h"
#include "QMC5883L.h"
#include "MyI2C.h"
#include "delay.h"
#include <math.h>
#include <stdio.h>

/* ══════════════════════════════════════════
   传感器量程与标度因子
   ══════════════════════════════════════════ */
#define GYRO_SCALE       (1.0f / 16.4f)        /* (°/s) / LSB */
#define ACCEL_SCALE      (9.80665f / 16384.0f) /* (m/s²) / LSB */
#define MAG_SCALE        (100.0f / 3000.0f)    /* μT / LSB */

/* ══════════════════════════════════════════
   内部静态变量
   ══════════════════════════════════════════ */
static uint8_t imu_mpu_id   = 0U;
static uint8_t imu_mag_id   = 0U;
static uint8_t imu_mag_addr = 0x0DU;   /* MSPM0 驱动使用 7-bit 地址；GitHub 0x1A(8-bit) => 0x0D */

/* 磁力计硬铁校准偏移 (μT) — 由 IMU_MagCalibApply() 或 IMU_SetMagOffsets() 设定 */
static float imu_mag_offset_x = 0.0f;
static float imu_mag_offset_y = 0.0f;
static float imu_mag_offset_z = 0.0f;

static uint8_t IMU_Addr8ToAddr7(uint8_t addr8)
{
    return (uint8_t)(addr8 >> 1);
}

/* ══════════════════════════════════════════
   初始化
   ══════════════════════════════════════════ */
uint8_t IMU_Init(void)
{
    uint8_t found = 0U;

    /* ———— 1. 初始化 MPU6050 ———— */
    imu_mpu_id = MPU6050_Init();
    if (imu_mpu_id == 0U) {
        return 0U;   /* MPU6050 无响应 */
    }

    /* MPU6050_Init 已配置 bypass 模式，确保开启 */
    if (!MPU6050_IsBypassEnabled()) {
        (void)MPU6050_WriteReg(MPU6050_USER_CTRL, 0x00U);
        (void)MPU6050_WriteReg(MPU6050_INT_PIN_CFG, 0x02U);
        Delay_ms(50U);
    }

    /* ———— 2. I2C 总线扫描，定位 QMC5883P ———— */
    /* GitHub 候选 8-bit 地址: 0x1A(7b=0x0D), 0x34(7b=0x1A), 0x58(7b=0x2C) */
    {
        const uint8_t candidates8[] = {0x1AU, 0x34U, 0x58U};
        uint8_t i;
        for (i = 0U; i < (uint8_t)(sizeof(candidates8) / sizeof(candidates8[0])); i++) {
            uint8_t id = 0xFFU;
            uint8_t addr7 = IMU_Addr8ToAddr7(candidates8[i]);
            if (QMC5883L_ProbeAddr(addr7, &id)) {
                imu_mag_addr = addr7;
                found = 1U;
                break;
            }
            Delay_us(50U);
        }
    }

    /* 如果候选地址都没找到，尝试全总线扫描 */
    if (!found) {
        uint8_t addr7;
        for (addr7 = 0x01U; addr7 < 0x7FU; addr7++) {
            uint8_t id = 0xFFU;

            /* 跳过 MPU6050 */
            if (addr7 == MPU6050_ADDR_7BIT) {
                continue;
            }

            /* 尝试读 CHIPID 确认 */
            if (QMC5883L_ProbeAddr(addr7, &id) && (id == 0x80U)) {
                imu_mag_addr = addr7;
                found = 1U;
                break;
            }
            Delay_us(50U);
        }
    }

    if (!found) {
        return 0U;   /* 未找到磁力计 */
    }

    /* ———— 3. 配置 QMC5883P ———— */
    QMC5883L_SetAddr(imu_mag_addr);

    if (!QMC5883L_Init()) {
        return 0U;   /* 磁力计初始化失败 */
    }

    imu_mag_id = QMC5883L_GetID();

    return 1U;   /* 成功 */
}

/* ══════════════════════════════════════════
   状态查询
   ══════════════════════════════════════════ */
uint8_t IMU_IsMagReady(void)
{
    return QMC5883L_IsDataReady();
}

void IMU_GetInfo(char *buf, uint16_t len)
{
    if ((buf == 0) || (len == 0U)) {
        return;
    }

    (void)snprintf(buf, len,
                   "MPU6050: WHO_AM_I=0x%02X %s\r\n"
                   "QMC5883P: CHIPID=0x%02X Addr7=0x%02X %s\r\n"
                   "Config: Gyro=+-2000dps Accel=+-2g Mag=+-8G\r\n"
                   "Filter: Kalman (dt=0.01s default)\r\n",
                   imu_mpu_id,
                   ((imu_mpu_id == 0x68U) || (imu_mpu_id == 0x72U)) ? "OK" : "FAIL",
                   imu_mag_id,
                   imu_mag_addr,
                   (imu_mag_id == 0x80U) ? "OK" : "WARN (expected 0x80)");
}

/* ══════════════════════════════════════════
   数据采集
   ══════════════════════════════════════════ */
void IMU_ReadRaw(IMU_RawData *raw)
{
    if (raw == 0) {
        return;
    }

    /* 读取 MPU6050 6 轴数据 */
    MPU6050_GetData(&raw->AccelX, &raw->AccelY, &raw->AccelZ,
                    &raw->GyroX,  &raw->GyroY,  &raw->GyroZ);

    /* 读取 QMC5883P 3 轴数据 */
    (void)QMC5883L_GetData(&raw->MagX, &raw->MagY, &raw->MagZ);
}

void IMU_ReadScaled(IMU_ScaledData *sc)
{
    int16_t ax, ay, az, gx, gy, gz, mx, my, mz;
    int16_t temp_raw;
    uint8_t temp_h;
    uint8_t temp_l;

    if (sc == 0) {
        return;
    }

    /* ———— 读取原始数据 ———— */
    MPU6050_GetData(&ax, &ay, &az, &gx, &gy, &gz);
    (void)QMC5883L_GetData(&mx, &my, &mz);

    /* 温度（MPU6050 内部） */
    temp_h = MPU6050_ReadRegValue(MPU6050_TEMP_OUT_H);
    temp_l = MPU6050_ReadRegValue(MPU6050_TEMP_OUT_L);
    temp_raw = (int16_t)(((uint16_t)temp_h << 8) | temp_l);

    /* ———— 标度转换 ———— */
    sc->AccelX = (float)ax * ACCEL_SCALE;
    sc->AccelY = (float)ay * ACCEL_SCALE;
    sc->AccelZ = (float)az * ACCEL_SCALE;

    sc->GyroX  = (float)gx * GYRO_SCALE;
    sc->GyroY  = (float)gy * GYRO_SCALE;
    sc->GyroZ  = (float)gz * GYRO_SCALE;

    sc->MagX   = (float)mx * MAG_SCALE - imu_mag_offset_x;
    sc->MagY   = (float)my * MAG_SCALE - imu_mag_offset_y;
    sc->MagZ   = (float)mz * MAG_SCALE - imu_mag_offset_z;

    /* MPU6050 温度公式: T = Temp/340 + 36.53 (°C) */
    sc->Temp   = (float)temp_raw / 340.0f + 36.53f;
}

/* ══════════════════════════════════════════
   角度解算（无滤波）
   ══════════════════════════════════════════ */
void IMU_AccelToAngles(const IMU_ScaledData *sc, float *roll, float *pitch)
{
    float ax;
    float ay;
    float az;

    if ((sc == 0) || (roll == 0) || (pitch == 0)) {
        return;
    }

    ax = sc->AccelX;
    ay = sc->AccelY;
    az = sc->AccelZ;

    /* atan2f 返回弧度，乘以 180/π 转为度 */
    *roll  = atan2f(ay, az) * 57.2957795f;
    *pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * 57.2957795f;
}

float IMU_ComputeHeading(const IMU_ScaledData *sc, float roll_deg, float pitch_deg)
{
    float roll;
    float pitch;
    float mx;
    float my;
    float mz;
    float cos_roll;
    float sin_roll;
    float cos_pitch;
    float sin_pitch;
    float Xh;
    float Yh;
    float yaw;

    if (sc == 0) {
        return 0.0f;
    }

    roll  = roll_deg  * 0.0174532925f;   /* 度 → 弧度 */
    pitch = pitch_deg * 0.0174532925f;

    mx = sc->MagX;
    my = sc->MagY;
    mz = sc->MagZ;

    cos_roll  = cosf(roll);
    sin_roll  = sinf(roll);
    cos_pitch = cosf(pitch);
    sin_pitch = sinf(pitch);

    /* 倾斜补偿：投影到水平面 */
    Xh = mx * cos_pitch
       + my * sin_roll * sin_pitch
       - mz * cos_roll * sin_pitch;
    Yh = my * cos_roll
       + mz * sin_roll;

    yaw = atan2f(Yh, Xh) * 57.2957795f;   /* 弧度 → 度 */
    if (yaw < 0.0f) yaw += 360.0f;

    return yaw;
}

void IMU_GetAttitudeRaw(IMU_ScaledData *sc, IMU_Attitude *att)
{
    if ((sc == 0) || (att == 0)) {
        return;
    }

    IMU_AccelToAngles(sc, &att->Roll, &att->Pitch);
    att->Yaw = IMU_ComputeHeading(sc, att->Roll, att->Pitch);
}

/* ══════════════════════════════════════════
   便捷接口
   ══════════════════════════════════════════ */
float IMU_GetYaw(void)
{
    IMU_ScaledData sc;
    float roll;
    float pitch;

    IMU_ReadScaled(&sc);
    IMU_AccelToAngles(&sc, &roll, &pitch);
    return IMU_ComputeHeading(&sc, roll, pitch);
}

/* ══════════════════════════════════════════
   卡尔曼滤波 (1D)
   ══════════════════════════════════════════ */
void IMU_KalmanInit(IMU_Kalman1D *kf, float Q_angle, float Q_bias, float R_measure)
{
    if (kf == 0) {
        return;
    }

    kf->Q_angle   = Q_angle;
    kf->Q_bias    = Q_bias;
    kf->R_measure = R_measure;

    kf->angle = 0.0f;
    kf->bias  = 0.0f;
    kf->rate  = 0.0f;

    /* 初始协方差矩阵 — 对角阵（初始不确定性较大） */
    kf->P[0][0] = 1.0f;
    kf->P[0][1] = 0.0f;
    kf->P[1][0] = 0.0f;
    kf->P[1][1] = 1.0f;
}

float IMU_KalmanUpdate(IMU_Kalman1D *kf, float measured_angle, float gyro_rate, float dt)
{
    float y;
    float S;
    float K0;
    float K1;
    float P00;
    float P01;
    float P10;
    float P11;

    if (kf == 0) {
        return 0.0f;
    }

    /* 第 1 步: 预测 (Prior) */
    kf->rate  = gyro_rate - kf->bias;
    kf->angle += kf->rate * dt;

    P00 = kf->P[0][0];
    P01 = kf->P[0][1];
    P10 = kf->P[1][0];
    P11 = kf->P[1][1];

    kf->P[0][0] = P00 + dt * (dt * P11 - P01 - P10 + kf->Q_angle);
    kf->P[0][1] = P01 - dt * P11;
    kf->P[1][0] = P10 - dt * P11;
    kf->P[1][1] = P11 + kf->Q_bias * dt;

    /* 第 2 步: 更新 (Posterior / Correction) */
    y = measured_angle - kf->angle;
    S = kf->P[0][0] + kf->R_measure;
    K0 = kf->P[0][0] / S;
    K1 = kf->P[1][0] / S;

    kf->angle += K0 * y;
    kf->bias  += K1 * y;

    P00 = kf->P[0][0];
    P01 = kf->P[0][1];
    P10 = kf->P[1][0];
    P11 = kf->P[1][1];

    kf->P[0][0] = P00 - K0 * P00;
    kf->P[0][1] = P01 - K0 * P01;
    kf->P[1][0] = P10 - K1 * P00;
    kf->P[1][1] = P11 - K1 * P01;

    return kf->angle;
}

/* ══════════════════════════════════════════
   互补滤波
   ══════════════════════════════════════════ */
void IMU_CompInit(IMU_Complementary *cf, float alpha)
{
    if (cf == 0) {
        return;
    }

    cf->alpha     = alpha;
    cf->angle     = 0.0f;
    cf->last_time = 0.0f;
}

float IMU_CompUpdate(IMU_Complementary *cf, float accel_angle, float gyro_rate, float dt)
{
    float gyro_angle;

    if (cf == 0) {
        return 0.0f;
    }

    /* 陀螺仪积分 */
    gyro_angle = cf->angle + gyro_rate * dt;

    /* 互补融合 */
    cf->angle = cf->alpha * gyro_angle + (1.0f - cf->alpha) * accel_angle;

    (void)dt;
    return cf->angle;
}

/* ══════════════════════════════════════════
   高层融合接口
   ══════════════════════════════════════════ */
void IMU_GetAttitudeKF(IMU_Kalman1D *kf_roll, IMU_Kalman1D *kf_pitch,
                       IMU_ScaledData *sc, IMU_Attitude *att, float dt)
{
    float accel_roll;
    float accel_pitch;

    if ((kf_roll == 0) || (kf_pitch == 0) || (sc == 0) || (att == 0)) {
        return;
    }

    /* 1. 加速度计 → roll/pitch */
    IMU_AccelToAngles(sc, &accel_roll, &accel_pitch);

    /* 2. 卡尔曼滤波融合（陀螺仪积分预测 + 加速度计校正） */
    att->Roll  = IMU_KalmanUpdate(kf_roll,  accel_roll,  sc->GyroX, dt);
    att->Pitch = IMU_KalmanUpdate(kf_pitch, accel_pitch, sc->GyroY, dt);

    /* 3. 磁力计 yaw（带倾斜补偿，使用滤波后的 roll/pitch） */
    att->Yaw = IMU_ComputeHeading(sc, att->Roll, att->Pitch);
}

/* ══════════════════════════════════════════
   磁力计硬铁校准
   ══════════════════════════════════════════ */
void IMU_MagCalibReset(IMU_MagCalib *calib)
{
    if (calib == 0) {
        return;
    }

    calib->MinX =  32767;
    calib->MaxX = -32768;
    calib->MinY =  32767;
    calib->MaxY = -32768;
    calib->MinZ =  32767;
    calib->MaxZ = -32768;
    calib->OffsetX = 0.0f;
    calib->OffsetY = 0.0f;
    calib->OffsetZ = 0.0f;
    calib->valid = 0U;
}

void IMU_MagCalibUpdate(IMU_MagCalib *calib, const IMU_RawData *raw)
{
    if ((calib == 0) || (raw == 0)) {
        return;
    }

    if (raw->MagX < calib->MinX) calib->MinX = raw->MagX;
    if (raw->MagX > calib->MaxX) calib->MaxX = raw->MagX;

    if (raw->MagY < calib->MinY) calib->MinY = raw->MagY;
    if (raw->MagY > calib->MaxY) calib->MaxY = raw->MagY;

    if (raw->MagZ < calib->MinZ) calib->MinZ = raw->MagZ;
    if (raw->MagZ > calib->MaxZ) calib->MaxZ = raw->MagZ;
}

void IMU_MagCalibApply(IMU_MagCalib *calib)
{
    float mid_x;
    float mid_y;
    float mid_z;

    if (calib == 0) {
        return;
    }

    /* 圆心 = (max + min) / 2，再乘以标度因子转为 μT */
    mid_x = (float)(calib->MaxX + calib->MinX) * 0.5f * MAG_SCALE;
    mid_y = (float)(calib->MaxY + calib->MinY) * 0.5f * MAG_SCALE;
    mid_z = (float)(calib->MaxZ + calib->MinZ) * 0.5f * MAG_SCALE;

    calib->OffsetX = mid_x;
    calib->OffsetY = mid_y;
    calib->OffsetZ = mid_z;
    calib->valid = 1U;

    /* 同步到全局偏移（IMU_ReadScaled 会使用） */
    imu_mag_offset_x = mid_x;
    imu_mag_offset_y = mid_y;
    imu_mag_offset_z = mid_z;
}

void IMU_SetMagOffsets(float ox, float oy, float oz)
{
    imu_mag_offset_x = ox;
    imu_mag_offset_y = oy;
    imu_mag_offset_z = oz;
}

void IMU_GetMagOffsets(float *ox, float *oy, float *oz)
{
    if (ox) *ox = imu_mag_offset_x;
    if (oy) *oy = imu_mag_offset_y;
    if (oz) *oz = imu_mag_offset_z;
}
