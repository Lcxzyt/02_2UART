#include "Heading.h"
#include "IMU.h"
#include <math.h>

#define HEADING_SAMPLE_PERIOD_SEC (0.02f)
#define HEADING_DT_MIN_SEC        (0.001f)
#define HEADING_DT_MAX_SEC        (0.200f)
/* MPU6050 gyro: ±2000 deg/s → 16.4 LSB/(deg/s). */
#define HEADING_GYRO_LSB_PER_DPS  (16.4f)
/* Gyro-z sign: +1 or -1, set via Heading_SetGyroZSign(). */
/* Mag correction: when mag is within this gate of the gyro-predicted
   heading, slowly nudge the fused heading toward the mag reading.
   Outside the gate (e.g. during fast turns / motor EMI) trust the
   gyro alone. */
#define HEADING_MAG_CORR_ALPHA     (0.005f)
#define HEADING_MAG_GATE_DEG       (30.0f)

static Heading_Data heading_data = {0};
static uint8_t heading_initialized = 0U;
static uint8_t heading_yaw_seeded = 0U;
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
        old_sign = -1;
    }

    heading_data.yaw_deg = 0;
    heading_data.mag_yaw_deg = 0;
    heading_data.gyro_z_raw = 0;
    heading_data.gyro_z_bias = 0;
    heading_data.gyro_z_sign = old_sign;
    heading_data.cal_progress = 0U;
    heading_data.mpu_ok = 0U;
    heading_data.mag_ok = 0U;
    heading_yaw_seeded = 0U;
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
       Keep the public command compatible, but finish immediately.
       Do NOT reset the roll/pitch Kalman filters here: this function runs
       every time HeadingDrive_Start() fires (i.e. every single 'h' press),
       right as the motors kick on. Wiping an already-converged filter at
       that exact moment forced roll/pitch to reconverge from zero during
       the startup vibration window, which showed up as a large spurious
       yaw excursion (measured even with wheels lifted off the ground, so
       it was never a real chassis rotation). Reuse whatever roll/pitch
       state already exists; it only starts from zero once, in
       Heading_Init(), which runs at boot / after a real sensor failure. */
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

    IMU_ReadRaw(&heading_raw);
    IMU_ReadScaled(&sc);
    heading_data.mpu_ok = heading_initialized;
    heading_data.mag_ok = heading_initialized;
    heading_data.gyro_z_raw = heading_raw.GyroZ;
    heading_data.gyro_z_bias = 0;

    if (!heading_initialized) {
        Heading_SetState(HEADING_STATE_SENSOR_FAIL, 0U);
        return false;
    }

    IMU_GetAttitudeKF(&heading_kf_roll, &heading_kf_pitch, &sc, &heading_attitude, dt_sec);

    heading_data.mag_yaw_deg = Heading_RoundNormalize(heading_attitude.Yaw);

    /* Fused heading: gyro integration for fast response, mag for slow
       drift correction.  Gyro responds instantly to rotation; mag is
       only used when its reading is close to the gyro prediction
       (within HEADING_MAG_GATE_DEG).  During curves or motor EMI the
       mag will be outside the gate and the gyro runs alone, avoiding
       the lag that a pure-mag filter would introduce. */
    if (!heading_yaw_seeded) {
        heading_data.yaw_deg = heading_data.mag_yaw_deg;
        heading_yaw_seeded = 1U;
    } else {
        float gyro_dps;
        float predicted;
        float mag_err;

        /* gyro_z_raw is raw LSB; convert to deg/s. */
        gyro_dps = (float)heading_data.gyro_z_raw
                   * (1.0f / HEADING_GYRO_LSB_PER_DPS)
                   * (float)heading_data.gyro_z_sign;

        predicted = (float)heading_data.yaw_deg + gyro_dps * dt_sec;

        mag_err = Heading_AngleDiffFloat((float)heading_data.mag_yaw_deg,
                                         predicted);

        if (fabsf(mag_err) <= HEADING_MAG_GATE_DEG) {
            /* Mag is trustworthy – slowly nudge toward it. */
            heading_data.yaw_deg =
                Heading_RoundNormalize(predicted
                                       + HEADING_MAG_CORR_ALPHA * mag_err);
        } else {
            /* Mag is likely disturbed (turn / EMI) – trust gyro alone. */
            heading_data.yaw_deg = Heading_RoundNormalize(predicted);
        }
    }

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

bool Heading_UpdateAndSnapYaw(float dt_sec)
{
    if (!Heading_UpdateWithDt(dt_sec)) {
        return false;
    }

    /* hi means: use the car's heading at THIS moment as the new reference.
       Bypass the normal yaw rate guard / smoothing here; otherwise a stale
       filtered yaw only moves a few degrees per command and hi cannot really
       re-zero the straight-driving target. */
    heading_data.yaw_deg = heading_data.mag_yaw_deg;
    heading_yaw_seeded = 1U;
    Heading_SetState(HEADING_STATE_READY, 1U);
    return true;
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
