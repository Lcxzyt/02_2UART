#ifndef __IMU_H
#define __IMU_H

#include <stdint.h>

/* STM32 reference IMU data model: MPU6050 + QMC5883P. */
typedef struct {
    int16_t AccelX;
    int16_t AccelY;
    int16_t AccelZ;
    int16_t TempRaw;
    int16_t GyroX;
    int16_t GyroY;
    int16_t GyroZ;
    int16_t MagX;
    int16_t MagY;
    int16_t MagZ;
} IMU_RawData;

typedef struct {
    float AccelX;      /* m/s^2 */
    float AccelY;
    float AccelZ;
    float GyroX;       /* deg/s */
    float GyroY;
    float GyroZ;
    float MagX;        /* uT, hard-iron offset removed */
    float MagY;
    float MagZ;
    float Temperature; /* deg C */
} IMU_ScaledData;

typedef struct {
    float Roll;
    float Pitch;
    float Yaw;
} IMU_Attitude;

typedef struct {
    float Angle;
    float Bias;
    float Rate;
    float P[2][2];
    float Q_angle;
    float Q_bias;
    float R_measure;
} IMU_Kalman1D;

typedef struct {
    float Angle;
    float Alpha;
} IMU_Complementary;

typedef struct {
    int16_t MinX;
    int16_t MaxX;
    int16_t MinY;
    int16_t MaxY;
    int16_t MinZ;
    int16_t MaxZ;
    float OffsetX;
    float OffsetY;
    float OffsetZ;
    uint8_t valid;
} IMU_MagCalib;

uint8_t IMU_Init(void);
uint8_t IMU_IsMagReady(void);
void IMU_GetInfo(char *buf, uint16_t len);

void IMU_ReadRaw(IMU_RawData *raw);
uint8_t IMU_ReadRawStatus(IMU_RawData *raw, uint8_t *mpu_ok, uint8_t *mag_ok);
void IMU_ReadScaled(IMU_ScaledData *sc);
uint8_t IMU_ReadScaledStatus(IMU_ScaledData *sc, uint8_t *mpu_ok, uint8_t *mag_ok);
uint8_t IMU_IsInitialized(void);

void IMU_AccelToAngles(const IMU_ScaledData *sc, float *roll, float *pitch);
float IMU_ComputeHeading(const IMU_ScaledData *sc, float roll, float pitch);
void IMU_GetAttitudeRaw(IMU_ScaledData *sc, IMU_Attitude *att);

void IMU_KalmanInit(IMU_Kalman1D *kf, float Q_angle, float Q_bias, float R_measure);
float IMU_KalmanUpdate(IMU_Kalman1D *kf, float measured_angle, float gyro_rate, float dt);

void IMU_CompInit(IMU_Complementary *cf, float alpha);
float IMU_CompUpdate(IMU_Complementary *cf, float accel_angle, float gyro_rate, float dt);

void IMU_GetAttitudeKF(IMU_Kalman1D *kf_roll, IMU_Kalman1D *kf_pitch,
                       IMU_ScaledData *sc, IMU_Attitude *att, float dt);

void IMU_MagCalibReset(IMU_MagCalib *calib);
void IMU_MagCalibUpdate(IMU_MagCalib *calib, const IMU_RawData *raw);
void IMU_MagCalibApply(IMU_MagCalib *calib);
void IMU_SetMagOffsets(float ox, float oy, float oz);
void IMU_GetMagOffsets(float *ox, float *oy, float *oz);

float IMU_GetYaw(void);

#endif /* __IMU_H */
