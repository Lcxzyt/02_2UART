#include "Heading.h"
#include "IMU.h"
#include "MPU6050.h"
#include "QMC5883L.h"

#define HEADING_SAMPLE_PERIOD_SEC (0.02f)
#define HEADING_DT_MIN_SEC        (0.001f)
#define HEADING_DT_MAX_SEC        (0.200f)
#define GYRO_SCALE                (1.0f / 16.4f)
#define ACCEL_SCALE               (9.80665f / 16384.0f)
#define MAG_SCALE                 (100.0f / 3000.0f)

static Heading_Data heading_data = {0};
static uint8_t heading_initialized = 0U;
static IMU_Kalman1D heading_kf_roll;
static IMU_Kalman1D heading_kf_pitch;
static IMU_Attitude heading_attitude;
static IMU_RawData heading_raw;

static float Heading_NormalizeFloat(float angle)
{
    while (angle < 0.0f) {
        angle += 360.0f;
    }
    while (angle >= 360.0f) {
        angle -= 360.0f;
    }
    return angle;
}

static int16_t Heading_NormalizeInt(int16_t angle)
{
    while (angle < 0) {
        angle = (int16_t)(angle + 360);
    }
    while (angle >= 360) {
        angle = (int16_t)(angle - 360);
    }
    return angle;
}

static int16_t Heading_RoundNormalize(float angle)
{
    float norm = Heading_NormalizeFloat(angle);
    return Heading_NormalizeInt((int16_t)((norm >= 0.0f) ? (norm + 0.5f) : (norm - 0.5f)));
}

static float Heading_AngleDiffFloat(float target_deg, float current_deg)
{
    float diff = target_deg - current_deg;
    while (diff > 180.0f) {
        diff -= 360.0f;
    }
    while (diff < -180.0f) {
        diff += 360.0f;
    }
    return diff;
}

static void Heading_SetState(uint8_t state, uint8_t ready)
{
    heading_data.state = state;
    heading_data.ready = ready;
}

static void Heading_RawToScaled(const IMU_RawData *raw, IMU_ScaledData *sc)
{
    float ox = 0.0f;
    float oy = 0.0f;
    float oz = 0.0f;

    IMU_GetMagOffsets(&ox, &oy, &oz);

    sc->AccelX = (float)raw->AccelX * ACCEL_SCALE;
    sc->AccelY = (float)raw->AccelY * ACCEL_SCALE;
    sc->AccelZ = (float)raw->AccelZ * ACCEL_SCALE;
    sc->GyroX = (float)raw->GyroX * GYRO_SCALE;
    sc->GyroY = (float)raw->GyroY * GYRO_SCALE;
    sc->GyroZ = (float)raw->GyroZ * GYRO_SCALE;
    sc->MagX = ((float)raw->MagX * MAG_SCALE) - ox;
    sc->MagY = ((float)raw->MagY * MAG_SCALE) - oy;
    sc->MagZ = ((float)raw->MagZ * MAG_SCALE) - oz;
    sc->Temperature = ((float)raw->TempRaw / 340.0f) + 36.53f;
}

static void Heading_ResetFilters(void)
{
    IMU_KalmanInit(&heading_kf_roll, 0.001f, 0.003f, 0.03f);
    IMU_KalmanInit(&heading_kf_pitch, 0.001f, 0.003f, 0.03f);
}

bool Heading_Init(void)
{
    int8_t old_sign = heading_data.gyro_z_sign;
    uint8_t ok;

    if (old_sign == 0) {
        old_sign = 1;
    }

    heading_data.yaw_deg = 0;
    heading_data.mag_yaw_deg = 0;
    heading_data.gyro_z_raw = 0;
    heading_data.gyro_z_bias = 0;
    heading_data.gyro_z_sign = old_sign;
    heading_data.cal_progress = 0U;
    heading_data.mpu_ok = 0U;
    heading_data.mag_ok = 0U;
    Heading_SetState(HEADING_STATE_IDLE, 0U);

    Heading_ResetFilters();

    ok = IMU_Init();
    heading_initialized = ok ? 1U : 0U;
    heading_data.mpu_ok = ok ? 1U : 0U;
    heading_data.mag_ok = ok ? 1U : 0U;

    if (!ok) {
        Heading_SetState(HEADING_STATE_SENSOR_FAIL, 0U);
        return false;
    }

    return true;
}

void Heading_StartCalibration(void)
{
    if (!heading_initialized) {
        (void)Heading_Init();
    }

    /* STM32 reference path has no gyro-Z bias calibration for yaw.
       Keep the public command compatible, but finish immediately. */
    Heading_ResetFilters();
    heading_data.gyro_z_bias = 0;
    heading_data.cal_progress = 100U;

    if (heading_initialized) {
        Heading_SetState(HEADING_STATE_READY, 1U);
    } else {
        Heading_SetState(HEADING_STATE_SENSOR_FAIL, 0U);
    }
}

bool Heading_UpdateWithDt(float dt_sec)
{
    IMU_ScaledData sc;
    uint8_t mpu_ok = 0U;
    uint8_t mag_ok = 0U;

    if (dt_sec < HEADING_DT_MIN_SEC) {
        dt_sec = HEADING_SAMPLE_PERIOD_SEC;
    } else if (dt_sec > HEADING_DT_MAX_SEC) {
        dt_sec = HEADING_DT_MAX_SEC;
    }

    if (!heading_initialized) {
        if (!Heading_Init()) {
            return false;
        }
    }

    (void)IMU_ReadRawStatus(&heading_raw, &mpu_ok, &mag_ok);
    heading_data.mpu_ok = mpu_ok;
    heading_data.mag_ok = mag_ok;
    heading_data.gyro_z_raw = heading_raw.GyroZ;
    heading_data.gyro_z_bias = 0;

    if ((!mpu_ok) || (!mag_ok)) {
        Heading_SetState(HEADING_STATE_SENSOR_FAIL, 0U);
        return false;
    }

    Heading_RawToScaled(&heading_raw, &sc);
    IMU_GetAttitudeKF(&heading_kf_roll, &heading_kf_pitch, &sc, &heading_attitude, dt_sec);

    heading_data.mag_yaw_deg = Heading_RoundNormalize(heading_attitude.Yaw);
    heading_data.yaw_deg = heading_data.mag_yaw_deg;
    if (heading_data.cal_progress == 0U) {
        heading_data.cal_progress = 100U;
    }

    Heading_SetState(HEADING_STATE_READY, 1U);
    return true;
}

bool Heading_Update(void)
{
    return Heading_UpdateWithDt(HEADING_SAMPLE_PERIOD_SEC);
}

uint8_t Heading_IsReady(void) { return heading_data.ready; }
uint8_t Heading_GetState(void) { return heading_data.state; }
int16_t Heading_GetYawDeg(void) { return heading_data.yaw_deg; }
int16_t Heading_GetMagYawDeg(void) { return heading_data.mag_yaw_deg; }
int16_t Heading_GetGyroZRaw(void) { return heading_data.gyro_z_raw; }
int16_t Heading_GetGyroZBias(void) { return heading_data.gyro_z_bias; }
uint8_t Heading_GetCalProgress(void) { return heading_data.cal_progress; }

void Heading_SetGyroZSign(int8_t sign)
{
    heading_data.gyro_z_sign = (sign < 0) ? -1 : 1;
}

int8_t Heading_GetGyroZSign(void) { return heading_data.gyro_z_sign; }

int16_t Heading_AngleDiffDeg(int16_t target_deg, int16_t current_deg)
{
    float diff = Heading_AngleDiffFloat((float)Heading_NormalizeInt(target_deg),
                                        (float)Heading_NormalizeInt(current_deg));
    return (int16_t)((diff >= 0.0f) ? (diff + 0.5f) : (diff - 0.5f));
}

const Heading_Data *Heading_GetData(void)
{
    return &heading_data;
}
