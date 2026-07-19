#include "Heading.h"
#include "IMU.h"
#include "IMUTest.h"
#include "MPU6050.h"
#include "delay.h"
#include <math.h>

#define HEADING_SAMPLE_PERIOD_SEC    (0.020f)
#define HEADING_DT_MIN_SEC           (0.001f)
#define HEADING_DT_MAX_SEC           (0.200f)
#define HEADING_MAG_TAU_SEC          (3.0f)
#define HEADING_MAG_GATE_DEG         (25.0f)
#define HEADING_MAG_NORM_MIN_RATIO   (0.75f)
#define HEADING_MAG_NORM_MAX_RATIO   (1.25f)
#define HEADING_MAG_REF_ALPHA        (0.002f)
#define HEADING_MAG_MAX_TILT_DEG     (12.0f)
#define HEADING_GYRO_CAL_SAMPLES     200U
#define HEADING_GYRO_CAL_MAX_TRIES   500U
#define HEADING_GYRO_STILL_RAW       500
#define HEADING_ACCEL_NORM_MIN_G     (0.85f)
#define HEADING_ACCEL_NORM_MAX_G     (1.15f)

static Heading_Data heading_data;
static uint8_t heading_initialized;
static uint8_t heading_yaw_seeded;
static uint8_t heading_level_attitude_valid;
static float heading_level_roll_deg;
static float heading_level_pitch_deg;
static IMU_Kalman1D heading_kf_roll;
static IMU_Kalman1D heading_kf_pitch;

static float Heading_Wrap360f(float angle)
{
    while (angle < 0.0f) angle += 360.0f;
    while (angle >= 360.0f) angle -= 360.0f;
    return angle;
}

float Heading_AngleDiffDegF(float target_deg, float current_deg)
{
    float diff = target_deg - current_deg;
    while (diff > 180.0f) diff -= 360.0f;
    while (diff < -180.0f) diff += 360.0f;
    return diff;
}

static int16_t Heading_RoundDeg(float value)
{
    return (int16_t)((value >= 0.0f) ? (value + 0.5f) : (value - 0.5f));
}

static void Heading_SetState(uint8_t state, uint8_t ready)
{
    heading_data.state = state;
    heading_data.ready = ready;
}

static uint8_t Heading_IsStillSample(const IMU_Sample *sample)
{
    float ax_g;
    float ay_g;
    float az_g;
    float accel_norm_g;

    if ((sample == 0) || (!sample->MpuValid)) return 0U;
    if ((sample->Raw.GyroX > HEADING_GYRO_STILL_RAW) ||
        (sample->Raw.GyroX < -HEADING_GYRO_STILL_RAW) ||
        (sample->Raw.GyroY > HEADING_GYRO_STILL_RAW) ||
        (sample->Raw.GyroY < -HEADING_GYRO_STILL_RAW) ||
        (sample->Raw.GyroZ > HEADING_GYRO_STILL_RAW) ||
        (sample->Raw.GyroZ < -HEADING_GYRO_STILL_RAW)) {
        return 0U;
    }

    ax_g = (float)sample->Raw.AccelX / MPU6050_ACCEL_LSB_PER_G;
    ay_g = (float)sample->Raw.AccelY / MPU6050_ACCEL_LSB_PER_G;
    az_g = (float)sample->Raw.AccelZ / MPU6050_ACCEL_LSB_PER_G;
    accel_norm_g = sqrtf(ax_g * ax_g + ay_g * ay_g + az_g * az_g);
    return ((accel_norm_g >= HEADING_ACCEL_NORM_MIN_G) &&
            (accel_norm_g <= HEADING_ACCEL_NORM_MAX_G)) ? 1U : 0U;
}

static uint8_t Heading_CalibrateGyroZ(void)
{
    IMU_Sample sample;
    uint16_t valid_count = 0U;
    uint16_t tries = 0U;
    int32_t sum = 0L;

    Heading_SetState(HEADING_STATE_CALIBRATING, 0U);
    while ((valid_count < HEADING_GYRO_CAL_SAMPLES) &&
           (tries < HEADING_GYRO_CAL_MAX_TRIES)) {
        tries++;
        if (IMU_ReadSample(&sample) && Heading_IsStillSample(&sample)) {
            sum += sample.Raw.GyroZ;
            valid_count++;
        } else {
            valid_count = 0U;
            sum = 0L;
        }
        Delay_ms(10U);
    }

    if (valid_count < HEADING_GYRO_CAL_SAMPLES) {
        heading_data.gyro_calibrated = 0U;
        return 0U;
    }

    heading_data.gyro_bias_z_dps =
        ((float)sum / (float)valid_count) / MPU6050_GYRO_LSB_PER_DPS;
    heading_data.gyro_calibrated = 1U;
    return 1U;
}

bool Heading_Init(void)
{
    heading_initialized = 0U;
    heading_yaw_seeded = 0U;
    heading_level_attitude_valid = 0U;
    heading_level_roll_deg = 0.0f;
    heading_level_pitch_deg = 0.0f;
    heading_data.yaw_fused_deg = 0.0f;
    heading_data.yaw_continuous_deg = 0.0f;
    heading_data.yaw_mag_deg = 0.0f;
    heading_data.roll_deg = 0.0f;
    heading_data.pitch_deg = 0.0f;
    heading_data.gyro_z_dps = 0.0f;
    heading_data.gyro_bias_z_dps = 0.0f;
    heading_data.mag_norm_uT = 0.0f;
    heading_data.mag_norm_ref_uT = 0.0f;
    heading_data.gyro_calibrated = 0U;
    heading_data.mag_valid = 0U;
    heading_data.mag_disturbed = 0U;
    Heading_SetState(HEADING_STATE_IDLE, 0U);

    IMU_KalmanInit(&heading_kf_roll, 0.001f, 0.003f, 0.03f);
    IMU_KalmanInit(&heading_kf_pitch, 0.001f, 0.003f, 0.03f);

    if (!IMUTest_Init()) {
        Heading_SetState(HEADING_STATE_SENSOR_FAIL, 0U);
        return false;
    }

    heading_initialized = 1U;
    if (!Heading_CalibrateGyroZ()) {
        heading_initialized = 0U;
        Heading_SetState(HEADING_STATE_SENSOR_FAIL, 0U);
        return false;
    }

    Heading_SetState(HEADING_STATE_CALIBRATING, 0U);
    return true;
}

void Heading_StartCalibration(void)
{
    if (!heading_initialized) {
        (void)Heading_Init();
    }
    if (heading_initialized && heading_data.gyro_calibrated) {
        Heading_SetState(HEADING_STATE_CALIBRATING, 0U);
    }
}

bool Heading_UpdateWithDt(float dt_sec)
{
    IMU_Sample sample;
    IMU_Attitude attitude;
    float accel_roll;
    float accel_pitch;
    float yaw_predict;
    float yaw_delta;
    float correction_gain;
    float mag_ratio = 1.0f;
    uint8_t mag_candidate;

    if (!heading_initialized) {
        if (!Heading_Init()) return false;
    }
    if (!heading_data.gyro_calibrated) {
        Heading_SetState(HEADING_STATE_CALIBRATING, 0U);
        return false;
    }

    if (dt_sec < HEADING_DT_MIN_SEC) dt_sec = HEADING_SAMPLE_PERIOD_SEC;
    if (dt_sec > HEADING_DT_MAX_SEC) dt_sec = HEADING_DT_MAX_SEC;

    if (!IMU_ReadSample(&sample)) {
        Heading_SetState(HEADING_STATE_SENSOR_FAIL, 0U);
        return false;
    }

    IMU_AccelToAngles(&sample.Scaled, &accel_roll, &accel_pitch);
    attitude.Roll = IMU_KalmanUpdate(&heading_kf_roll,
                                     accel_roll,
                                     sample.Scaled.GyroX,
                                     dt_sec);
    attitude.Pitch = IMU_KalmanUpdate(&heading_kf_pitch,
                                      accel_pitch,
                                      sample.Scaled.GyroY,
                                      dt_sec);
    heading_data.roll_deg = attitude.Roll;
    heading_data.pitch_deg = attitude.Pitch;

    heading_data.gyro_z_dps = sample.Scaled.GyroZ - heading_data.gyro_bias_z_dps;
    yaw_predict = Heading_Wrap360f(heading_data.yaw_fused_deg +
                                   heading_data.gyro_z_dps * dt_sec);

    heading_data.mag_valid = sample.MagReadValid;
    heading_data.mag_disturbed = 0U;
    mag_candidate = sample.MagReadValid;

    if (mag_candidate) {
        float tilt_roll;
        float tilt_pitch;

        heading_data.yaw_mag_deg = IMU_ComputeHeading(&sample.Scaled,
                                                      attitude.Roll,
                                                      attitude.Pitch);
        heading_data.mag_norm_uT = sqrtf(sample.Scaled.MagX * sample.Scaled.MagX +
                                         sample.Scaled.MagY * sample.Scaled.MagY +
                                         sample.Scaled.MagZ * sample.Scaled.MagZ);

        if (heading_data.mag_norm_ref_uT <= 0.0f) {
            heading_data.mag_norm_ref_uT = heading_data.mag_norm_uT;
        } else if (heading_data.mag_norm_ref_uT > 0.0f) {
            mag_ratio = heading_data.mag_norm_uT / heading_data.mag_norm_ref_uT;
            if ((mag_ratio < HEADING_MAG_NORM_MIN_RATIO) ||
                (mag_ratio > HEADING_MAG_NORM_MAX_RATIO)) {
                heading_data.mag_disturbed = 1U;
            }
        }

        /*
         * The current calibration is a horizontal 2D ellipse. Outside its
         * validated plane, especially with an uncalibrated QMC Z axis, tilt
         * compensation can create a plausible but wrong magnetic yaw. Use
         * short-term GyroZ propagation until the chassis is near level again.
         */
        if (heading_level_attitude_valid) {
            tilt_roll = attitude.Roll - heading_level_roll_deg;
            tilt_pitch = attitude.Pitch - heading_level_pitch_deg;
            if ((tilt_roll * tilt_roll + tilt_pitch * tilt_pitch) >
                (HEADING_MAG_MAX_TILT_DEG * HEADING_MAG_MAX_TILT_DEG)) {
                heading_data.mag_disturbed = 1U;
            }
        }

        if (heading_yaw_seeded) {
            float mag_error = Heading_AngleDiffDegF(heading_data.yaw_mag_deg,
                                                     yaw_predict);
            if (fabsf(mag_error) > HEADING_MAG_GATE_DEG) {
                heading_data.mag_disturbed = 1U;
            }
        }
    }

    if (!heading_yaw_seeded) {
        if ((!mag_candidate) || heading_data.mag_disturbed) {
            Heading_SetState(HEADING_STATE_CALIBRATING, 0U);
            return false;
        }
        heading_data.yaw_fused_deg = heading_data.yaw_mag_deg;
        heading_data.yaw_continuous_deg = heading_data.yaw_mag_deg;
        heading_level_roll_deg = attitude.Roll;
        heading_level_pitch_deg = attitude.Pitch;
        heading_level_attitude_valid = 1U;
        heading_yaw_seeded = 1U;
    } else {
        float yaw_new = yaw_predict;
        if (mag_candidate && (!heading_data.mag_disturbed)) {
            float mag_error = Heading_AngleDiffDegF(heading_data.yaw_mag_deg,
                                                     yaw_predict);
            correction_gain = 1.0f - expf(-dt_sec / HEADING_MAG_TAU_SEC);
            yaw_new = Heading_Wrap360f(yaw_predict + correction_gain * mag_error);
            heading_data.mag_norm_ref_uT +=
                HEADING_MAG_REF_ALPHA *
                (heading_data.mag_norm_uT - heading_data.mag_norm_ref_uT);
        }
        yaw_delta = Heading_AngleDiffDegF(yaw_new, heading_data.yaw_fused_deg);
        heading_data.yaw_continuous_deg += yaw_delta;
        heading_data.yaw_fused_deg = yaw_new;
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
    uint8_t attempt;

    /* Capturing a target must not overwrite the estimator with raw magnetic yaw. */
    for (attempt = 0U; attempt < 10U; attempt++) {
        if (Heading_UpdateWithDt(dt_sec)) return true;
        if (Heading_GetState() == HEADING_STATE_SENSOR_FAIL) return false;
        Delay_ms(5U);
    }
    return false;
}

uint8_t Heading_IsReady(void) { return heading_data.ready; }
uint8_t Heading_GetState(void) { return heading_data.state; }
float Heading_GetYawDegF(void) { return heading_data.yaw_fused_deg; }
float Heading_GetMagYawDegF(void) { return heading_data.yaw_mag_deg; }

int16_t Heading_GetYawDeg(void)
{
    return Heading_RoundDeg(Heading_Wrap360f(heading_data.yaw_fused_deg));
}

int16_t Heading_AngleDiffDeg(int16_t target_deg, int16_t current_deg)
{
    return Heading_RoundDeg(Heading_AngleDiffDegF((float)target_deg,
                                                  (float)current_deg));
}

const Heading_Data *Heading_GetData(void)
{
    return &heading_data;
}
