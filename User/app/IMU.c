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
#define GYRO_SCALE       (1.0f / MPU6050_GYRO_LSB_PER_DPS)
#define ACCEL_SCALE      (9.80665f / MPU6050_ACCEL_LSB_PER_G)
#define MAG_SCALE        (100.0f / 3750.0f)    /* QMC5883P ±8G: μT / LSB */

/* ══════════════════════════════════════════
   内部静态变量
   ══════════════════════════════════════════ */
static uint8_t imu_mpu_id   = 0U;
static uint8_t imu_mag_id   = 0U;
static uint8_t imu_mag_addr = 0x2CU;

/*
 * 本车水平整圈标定结果（QMC原生坐标系）。
 * X/Y偏移单位为μT；2x2矩阵用于补偿水平面内的比例差和非正交误差。
 * Z轴没有在水平旋转中获得充分激励，因此保持0偏移且不参与软铁补偿。
 */
static float imu_mag_offset_x = -5.7823f;
static float imu_mag_offset_y = 33.0860f;
static float imu_mag_offset_z = 0.0f;
static float imu_mag_matrix_00 = 0.980118f;
static float imu_mag_matrix_01 = 0.011096f;
static float imu_mag_matrix_10 = 0.011096f;
static float imu_mag_matrix_11 = 1.020961f;
static IMU_Sample imu_last_sample;
static uint8_t imu_last_sample_valid = 0U;

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

    /* ———— 2. 严格识别 QMC5883P ———— */
    /* 官方默认7-bit地址为0x2C；保留两个已知兼容模块地址，但必须校验CHIPID。 */
    {
        const uint8_t candidates7[] = {0x2CU, 0x0DU, 0x1AU};
        uint8_t i;
        for (i = 0U; i < (uint8_t)(sizeof(candidates7) / sizeof(candidates7[0])); i++) {
            uint8_t id = 0xFFU;
            uint8_t addr7 = candidates7[i];
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

    if (imu_mag_id != 0x80U) {
        return 0U;
    }

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
                   "Config: Gyro=+-500dps Accel=+-2g Mag=+-8G\r\n"
                   "Filter: Roll/Pitch Kalman + GyroZ/Mag heading fusion\r\n",
                   imu_mpu_id,
                   ((imu_mpu_id == 0x68U) || (imu_mpu_id == 0x72U)) ? "OK" : "FAIL",
                   imu_mag_id,
                   imu_mag_addr,
                   (imu_mag_id == 0x80U) ? "OK" : "WARN (expected 0x80)");
}

/* ══════════════════════════════════════════
   数据采集
   ══════════════════════════════════════════ */
static void IMU_ConvertSample(IMU_Sample *sample)
{
    float qmc_x;
    float qmc_y;
    float qmc_z;
    float qmc_cal_x;
    float qmc_cal_y;

    sample->Scaled.AccelX = (float)sample->Raw.AccelX * ACCEL_SCALE;
    sample->Scaled.AccelY = (float)sample->Raw.AccelY * ACCEL_SCALE;
    sample->Scaled.AccelZ = (float)sample->Raw.AccelZ * ACCEL_SCALE;

    sample->Scaled.GyroX = (float)sample->Raw.GyroX * GYRO_SCALE;
    sample->Scaled.GyroY = (float)sample->Raw.GyroY * GYRO_SCALE;
    sample->Scaled.GyroZ = (float)sample->Raw.GyroZ * GYRO_SCALE;

    /* Hard/soft-iron calibration remains expressed in the native QMC sensor frame. */
    qmc_x = (float)sample->Raw.MagX * MAG_SCALE - imu_mag_offset_x;
    qmc_y = (float)sample->Raw.MagY * MAG_SCALE - imu_mag_offset_y;
    qmc_z = (float)sample->Raw.MagZ * MAG_SCALE - imu_mag_offset_z;
    qmc_cal_x = imu_mag_matrix_00 * qmc_x + imu_mag_matrix_01 * qmc_y;
    qmc_cal_y = imu_mag_matrix_10 * qmc_x + imu_mag_matrix_11 * qmc_y;

    /* Confirmed board alignment: MPU +X=-QMC Y, +Y=QMC X, +Z=QMC Z. */
    sample->Scaled.MagX = -qmc_cal_y;
    sample->Scaled.MagY =  qmc_cal_x;
    sample->Scaled.MagZ =  qmc_z;
    sample->Scaled.Temp = 0.0f;
}

uint8_t IMU_ReadSample(IMU_Sample *sample)
{
    uint8_t mag_status = 0U;

    if (sample == 0) return 0U;

    sample->Raw.AccelX = 0;
    sample->Raw.AccelY = 0;
    sample->Raw.AccelZ = 0;
    sample->Raw.GyroX = 0;
    sample->Raw.GyroY = 0;
    sample->Raw.GyroZ = 0;
    sample->Raw.MagX = 0;
    sample->Raw.MagY = 0;
    sample->Raw.MagZ = 0;

    sample->MpuValid = MPU6050_GetData(&sample->Raw.AccelX,
                                       &sample->Raw.AccelY,
                                       &sample->Raw.AccelZ,
                                       &sample->Raw.GyroX,
                                       &sample->Raw.GyroY,
                                       &sample->Raw.GyroZ) ? 1U : 0U;

    sample->MagReadValid = QMC5883L_GetDataChecked(&sample->Raw.MagX,
                                                    &sample->Raw.MagY,
                                                    &sample->Raw.MagZ,
                                                    &mag_status) ? 1U : 0U;
    sample->MagReady = ((mag_status & QMC5883L_SR_DRDY) != 0U) ? 1U : 0U;
    sample->MagOverflow = ((mag_status & QMC5883L_SR_OVFL) != 0U) ? 1U : 0U;

    IMU_ConvertSample(sample);
    imu_last_sample = *sample;
    imu_last_sample_valid = sample->MpuValid;
    return sample->MpuValid;
}

uint8_t IMU_GetLastSample(IMU_Sample *sample)
{
    if ((sample == 0) || (!imu_last_sample_valid)) return 0U;
    *sample = imu_last_sample;
    return 1U;
}

uint8_t IMU_ReadRaw(IMU_RawData *raw)
{
    IMU_Sample sample;

    if ((raw == 0) || (!IMU_ReadSample(&sample))) return 0U;
    *raw = sample.Raw;
    return sample.MagReadValid;
}

uint8_t IMU_ReadScaled(IMU_ScaledData *sc)
{
    IMU_Sample sample;

    if ((sc == 0) || (!IMU_ReadSample(&sample))) return 0U;
    *sc = sample.Scaled;
    return sample.MagReadValid;
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

    if (!IMU_ReadScaled(&sc)) {
        return 0.0f;
    }
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

void IMU_SetMagCalibration2D(float ox, float oy,
                             float m00, float m01, float m10, float m11)
{
    imu_mag_offset_x = ox;
    imu_mag_offset_y = oy;
    imu_mag_matrix_00 = m00;
    imu_mag_matrix_01 = m01;
    imu_mag_matrix_10 = m10;
    imu_mag_matrix_11 = m11;
}

void IMU_GetMagCalibration2D(float *ox, float *oy,
                             float *m00, float *m01, float *m10, float *m11)
{
    if (ox)  *ox = imu_mag_offset_x;
    if (oy)  *oy = imu_mag_offset_y;
    if (m00) *m00 = imu_mag_matrix_00;
    if (m01) *m01 = imu_mag_matrix_01;
    if (m10) *m10 = imu_mag_matrix_10;
    if (m11) *m11 = imu_mag_matrix_11;
}
