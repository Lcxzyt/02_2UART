#include "IMU.h"
#include "MPU6050.h"
#include "MPU6050_Reg.h"
#include "QMC5883L.h"
#include "MyI2C.h"
#include "delay.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

/* Same scale factors as the tested STM32 project. */
#define GYRO_SCALE       (1.0f / 16.4f)          /* (deg/s) / LSB, MPU6050 +/-2000 dps */
#define ACCEL_SCALE      (9.80665f / 16384.0f)  /* (m/s^2) / LSB, MPU6050 +/-2 g */
#define MAG_SCALE        (100.0f / 3000.0f)     /* uT / LSB, QMC5883P +/-8 G */
#define IMU_RAD_TO_DEG   (57.2957795131f)
#define IMU_DEG_TO_RAD   (0.01745329252f)

static uint8_t imu_mpu_id = 0U;
static uint8_t imu_mag_id = 0U;
static uint8_t imu_mag_addr = 0x0DU; /* MSPM0 driver uses 7-bit addresses. STM32 0x1A => 0x0D. */
static uint8_t imu_initialized = 0U;

static float imu_mag_offset_x = 0.0f;
static float imu_mag_offset_y = 0.0f;
static float imu_mag_offset_z = 0.0f;

static float IMU_NormalizeYaw(float yaw)
{
    while (yaw < 0.0f) {
        yaw += 360.0f;
    }
    while (yaw >= 360.0f) {
        yaw -= 360.0f;
    }
    return yaw;
}

uint8_t IMU_Init(void)
{
    uint8_t found = 0U;
    uint8_t id = 0U;

    if (imu_initialized) {
        return 1U;
    }

    imu_mpu_id = MPU6050_Init();
    if (imu_mpu_id == 0U) {
        imu_initialized = 0U;
        return 0U;
    }

    if (!MPU6050_IsBypassEnabled()) {
        (void)MPU6050_WriteReg(MPU6050_USER_CTRL, 0x00U);
        (void)MPU6050_WriteReg(MPU6050_INT_PIN_CFG, 0x02U);
        Delay_ms(50U);
    }

    /* STM32 candidates were 8-bit {0x1A,0x34,0x58}; current I2C API wants 7-bit. */
    {
        const uint8_t candidates[] = {0x0DU, 0x1AU, 0x2CU};
        uint32_t i;
        for (i = 0U; i < (sizeof(candidates) / sizeof(candidates[0])); i++) {
            id = 0xFFU;
            if (QMC5883L_ProbeAddr(candidates[i], &id)) {
                imu_mag_addr = candidates[i];
                imu_mag_id = id;
                found = 1U;
                break;
            }
            Delay_us(50U);
        }
    }

    if (!found) {
        uint8_t addr;
        for (addr = 0x02U; addr < 0x78U; addr++) {
            if (addr == MPU6050_ADDR_7BIT) {
                continue;
            }
            id = 0xFFU;
            if (QMC5883L_ProbeAddr(addr, &id) && (id == 0x80U)) {
                imu_mag_addr = addr;
                imu_mag_id = id;
                found = 1U;
                break;
            }
            Delay_us(50U);
        }
    }

    if (!found) {
        imu_initialized = 0U;
        return 0U;
    }

    QMC5883L_SetAddr(imu_mag_addr);
    if (!QMC5883L_Init()) {
        imu_initialized = 0U;
        return 0U;
    }

    imu_mag_id = QMC5883L_GetID();
    imu_initialized = 1U;
    return 1U;
}

uint8_t IMU_IsMagReady(void)
{
    return QMC5883L_IsDataReady();
}

uint8_t IMU_IsInitialized(void)
{
    return imu_initialized;
}

void IMU_GetInfo(char *buf, uint16_t len)
{
    if ((buf == 0) || (len == 0U)) {
        return;
    }
    (void)snprintf(buf, len,
                   "MPU6050 id=0x%02X, QMC5883P addr7=0x%02X id=0x%02X",
                   imu_mpu_id, imu_mag_addr, imu_mag_id);
}

static void IMU_RawToScaled(const IMU_RawData *raw, IMU_ScaledData *sc)
{
    sc->AccelX = (float)raw->AccelX * ACCEL_SCALE;
    sc->AccelY = (float)raw->AccelY * ACCEL_SCALE;
    sc->AccelZ = (float)raw->AccelZ * ACCEL_SCALE;
    sc->GyroX  = (float)raw->GyroX * GYRO_SCALE;
    sc->GyroY  = (float)raw->GyroY * GYRO_SCALE;
    sc->GyroZ  = (float)raw->GyroZ * GYRO_SCALE;
    sc->MagX   = ((float)raw->MagX * MAG_SCALE) - imu_mag_offset_x;
    sc->MagY   = ((float)raw->MagY * MAG_SCALE) - imu_mag_offset_y;
    sc->MagZ   = ((float)raw->MagZ * MAG_SCALE) - imu_mag_offset_z;
    sc->Temperature = ((float)raw->TempRaw / 340.0f) + 36.53f;
}

uint8_t IMU_ReadRawStatus(IMU_RawData *raw, uint8_t *mpu_ok, uint8_t *mag_ok)
{
    uint8_t buf[14];
    uint8_t mpu_read_ok = 0U;
    uint8_t mag_read_ok = 0U;
    int16_t mx = 0;
    int16_t my = 0;
    int16_t mz = 0;

    if (raw == 0) {
        if (mpu_ok != 0) *mpu_ok = 0U;
        if (mag_ok != 0) *mag_ok = 0U;
        return 0U;
    }

    (void)memset(raw, 0, sizeof(*raw));

    if (MPU6050_ReadRegs(MPU6050_ACCEL_XOUT_H, buf, sizeof(buf))) {
        raw->AccelX  = (int16_t)(((uint16_t)buf[0] << 8) | buf[1]);
        raw->AccelY  = (int16_t)(((uint16_t)buf[2] << 8) | buf[3]);
        raw->AccelZ  = (int16_t)(((uint16_t)buf[4] << 8) | buf[5]);
        raw->TempRaw = (int16_t)(((uint16_t)buf[6] << 8) | buf[7]);
        raw->GyroX   = (int16_t)(((uint16_t)buf[8] << 8) | buf[9]);
        raw->GyroY   = (int16_t)(((uint16_t)buf[10] << 8) | buf[11]);
        raw->GyroZ   = (int16_t)(((uint16_t)buf[12] << 8) | buf[13]);
        mpu_read_ok = 1U;
    }

    if (QMC5883L_GetData(&mx, &my, &mz)) {
        raw->MagX = mx;
        raw->MagY = my;
        raw->MagZ = mz;
        mag_read_ok = 1U;
    }

    if (mpu_ok != 0) *mpu_ok = mpu_read_ok;
    if (mag_ok != 0) *mag_ok = mag_read_ok;
    return (mpu_read_ok && mag_read_ok) ? 1U : 0U;
}

void IMU_ReadRaw(IMU_RawData *raw)
{
    (void)IMU_ReadRawStatus(raw, 0, 0);
}

uint8_t IMU_ReadScaledStatus(IMU_ScaledData *sc, uint8_t *mpu_ok, uint8_t *mag_ok)
{
    IMU_RawData raw;
    uint8_t ok;

    if (sc == 0) {
        if (mpu_ok != 0) *mpu_ok = 0U;
        if (mag_ok != 0) *mag_ok = 0U;
        return 0U;
    }

    ok = IMU_ReadRawStatus(&raw, mpu_ok, mag_ok);
    IMU_RawToScaled(&raw, sc);
    return ok;
}

void IMU_ReadScaled(IMU_ScaledData *sc)
{
    (void)IMU_ReadScaledStatus(sc, 0, 0);
}

void IMU_AccelToAngles(const IMU_ScaledData *sc, float *roll, float *pitch)
{
    float r = 0.0f;
    float p = 0.0f;

    if (sc != 0) {
        r = atan2f(sc->AccelY, sc->AccelZ) * IMU_RAD_TO_DEG;
        p = atan2f(-sc->AccelX,
                   sqrtf((sc->AccelY * sc->AccelY) + (sc->AccelZ * sc->AccelZ))) * IMU_RAD_TO_DEG;
    }

    if (roll != 0) {
        *roll = r;
    }
    if (pitch != 0) {
        *pitch = p;
    }
}

float IMU_ComputeHeading(const IMU_ScaledData *sc, float roll, float pitch)
{
    float roll_rad;
    float pitch_rad;
    float sin_roll;
    float cos_roll;
    float sin_pitch;
    float cos_pitch;
    float xh;
    float yh;
    float yaw;

    if (sc == 0) {
        return 0.0f;
    }

    roll_rad = roll * IMU_DEG_TO_RAD;
    pitch_rad = pitch * IMU_DEG_TO_RAD;

    sin_roll = sinf(roll_rad);
    cos_roll = cosf(roll_rad);
    sin_pitch = sinf(pitch_rad);
    cos_pitch = cosf(pitch_rad);

    /* Tilt-compensated magnetometer projection, same STM32 algorithmic path. */
    xh = (sc->MagX * cos_pitch) + (sc->MagZ * sin_pitch);
    yh = (sc->MagX * sin_roll * sin_pitch) +
         (sc->MagY * cos_roll) -
         (sc->MagZ * sin_roll * cos_pitch);

    yaw = atan2f(yh, xh) * IMU_RAD_TO_DEG;
    return IMU_NormalizeYaw(yaw);
}

void IMU_GetAttitudeRaw(IMU_ScaledData *sc, IMU_Attitude *att)
{
    float roll = 0.0f;
    float pitch = 0.0f;

    if ((sc == 0) || (att == 0)) {
        return;
    }

    IMU_AccelToAngles(sc, &roll, &pitch);
    att->Roll = roll;
    att->Pitch = pitch;
    att->Yaw = IMU_ComputeHeading(sc, roll, pitch);
}

void IMU_KalmanInit(IMU_Kalman1D *kf, float Q_angle, float Q_bias, float R_measure)
{
    if (kf == 0) {
        return;
    }

    kf->Angle = 0.0f;
    kf->Bias = 0.0f;
    kf->Rate = 0.0f;
    kf->P[0][0] = 0.0f;
    kf->P[0][1] = 0.0f;
    kf->P[1][0] = 0.0f;
    kf->P[1][1] = 0.0f;
    kf->Q_angle = Q_angle;
    kf->Q_bias = Q_bias;
    kf->R_measure = R_measure;
}

float IMU_KalmanUpdate(IMU_Kalman1D *kf, float measured_angle, float gyro_rate, float dt)
{
    float y;
    float s;
    float k0;
    float k1;
    float p00_temp;
    float p01_temp;

    if (kf == 0) {
        return measured_angle;
    }

    if (dt <= 0.0f) {
        dt = 0.01f;
    }

    kf->Rate = gyro_rate - kf->Bias;
    kf->Angle += dt * kf->Rate;

    kf->P[0][0] += dt * ((dt * kf->P[1][1]) - kf->P[0][1] - kf->P[1][0] + kf->Q_angle);
    kf->P[0][1] -= dt * kf->P[1][1];
    kf->P[1][0] -= dt * kf->P[1][1];
    kf->P[1][1] += kf->Q_bias * dt;

    y = measured_angle - kf->Angle;
    s = kf->P[0][0] + kf->R_measure;
    if (s == 0.0f) {
        return kf->Angle;
    }

    k0 = kf->P[0][0] / s;
    k1 = kf->P[1][0] / s;

    kf->Angle += k0 * y;
    kf->Bias += k1 * y;

    p00_temp = kf->P[0][0];
    p01_temp = kf->P[0][1];

    kf->P[0][0] -= k0 * p00_temp;
    kf->P[0][1] -= k0 * p01_temp;
    kf->P[1][0] -= k1 * p00_temp;
    kf->P[1][1] -= k1 * p01_temp;

    return kf->Angle;
}

void IMU_CompInit(IMU_Complementary *cf, float alpha)
{
    if (cf == 0) {
        return;
    }
    cf->Angle = 0.0f;
    cf->Alpha = alpha;
}

float IMU_CompUpdate(IMU_Complementary *cf, float accel_angle, float gyro_rate, float dt)
{
    if (cf == 0) {
        return accel_angle;
    }
    if (dt <= 0.0f) {
        dt = 0.01f;
    }
    cf->Angle = (cf->Alpha * (cf->Angle + (gyro_rate * dt))) +
                ((1.0f - cf->Alpha) * accel_angle);
    return cf->Angle;
}

void IMU_GetAttitudeKF(IMU_Kalman1D *kf_roll, IMU_Kalman1D *kf_pitch,
                       IMU_ScaledData *sc, IMU_Attitude *att, float dt)
{
    float accel_roll = 0.0f;
    float accel_pitch = 0.0f;

    if ((kf_roll == 0) || (kf_pitch == 0) || (sc == 0) || (att == 0)) {
        return;
    }

    IMU_AccelToAngles(sc, &accel_roll, &accel_pitch);
    att->Roll = IMU_KalmanUpdate(kf_roll, accel_roll, sc->GyroX, dt);
    att->Pitch = IMU_KalmanUpdate(kf_pitch, accel_pitch, sc->GyroY, dt);
    att->Yaw = IMU_ComputeHeading(sc, att->Roll, att->Pitch);
}

void IMU_MagCalibReset(IMU_MagCalib *calib)
{
    if (calib == 0) {
        return;
    }

    calib->MinX = 32767;
    calib->MaxX = -32768;
    calib->MinY = 32767;
    calib->MaxY = -32768;
    calib->MinZ = 32767;
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
    if (calib == 0) {
        return;
    }

    calib->OffsetX = ((float)calib->MaxX + (float)calib->MinX) * 0.5f * MAG_SCALE;
    calib->OffsetY = ((float)calib->MaxY + (float)calib->MinY) * 0.5f * MAG_SCALE;
    calib->OffsetZ = ((float)calib->MaxZ + (float)calib->MinZ) * 0.5f * MAG_SCALE;
    calib->valid = 1U;

    IMU_SetMagOffsets(calib->OffsetX, calib->OffsetY, calib->OffsetZ);
}

void IMU_SetMagOffsets(float ox, float oy, float oz)
{
    imu_mag_offset_x = ox;
    imu_mag_offset_y = oy;
    imu_mag_offset_z = oz;
}

void IMU_GetMagOffsets(float *ox, float *oy, float *oz)
{
    if (ox != 0) {
        *ox = imu_mag_offset_x;
    }
    if (oy != 0) {
        *oy = imu_mag_offset_y;
    }
    if (oz != 0) {
        *oz = imu_mag_offset_z;
    }
}

float IMU_GetYaw(void)
{
    IMU_ScaledData sc;
    float roll = 0.0f;
    float pitch = 0.0f;

    IMU_ReadScaled(&sc);
    IMU_AccelToAngles(&sc, &roll, &pitch);
    return IMU_ComputeHeading(&sc, roll, pitch);
}
