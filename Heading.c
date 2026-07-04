#include "Heading.h"
#include "IMUTest.h"

#define HEADING_BIAS_SAMPLE_COUNT 60U
#define HEADING_SAMPLE_PERIOD_SEC 0.020f
#define HEADING_GYRO_LSB_PER_DPS  16.4f
#define HEADING_MAG_CORR_ALPHA    0.050f

static Heading_Data heading_data;
static uint8_t heading_initialized;
static uint16_t heading_cal_samples;
static int32_t heading_bias_accum;
static float heading_fused_yaw;

static float Heading_NormalizeFloat(float deg)
{
    while (deg < 0.0f) {
        deg += 360.0f;
    }
    while (deg >= 360.0f) {
        deg -= 360.0f;
    }
    return deg;
}

static int16_t Heading_NormalizeInt(int16_t deg)
{
    while (deg < 0) {
        deg = (int16_t)(deg + 360);
    }
    while (deg >= 360) {
        deg = (int16_t)(deg - 360);
    }
    return deg;
}

static int16_t Heading_RoundNormalize(float deg)
{
    int16_t rounded = (int16_t)((deg >= 0.0f) ? (deg + 0.5f) : (deg - 0.5f));
    return Heading_NormalizeInt(rounded);
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

bool Heading_Init(void)
{
    bool ok;

    heading_initialized = 1U;
    heading_data.gyro_z_sign = 1;
    heading_data.yaw_deg = 0;
    heading_data.mag_yaw_deg = 0;
    heading_data.gyro_z_raw = 0;
    heading_data.gyro_z_bias = 0;
    heading_data.mpu_ok = 0U;
    heading_data.mag_ok = 0U;
    heading_data.cal_progress = 0U;
    heading_fused_yaw = 0.0f;
    heading_cal_samples = 0U;
    heading_bias_accum = 0;
    Heading_SetState(HEADING_STATE_IDLE, 0U);

    ok = IMUTest_Init();
    return ok;
}

void Heading_StartCalibration(void)
{
    if (!heading_initialized) {
        (void)Heading_Init();
    } else {
        (void)IMUTest_Init();
    }

    heading_cal_samples = 0U;
    heading_bias_accum = 0;
    heading_data.cal_progress = 0U;
    Heading_SetState(HEADING_STATE_CALIBRATING, 0U);
}

bool Heading_Update(void)
{
    IMUTest_Data imu;
    bool mag_valid;
    float mag_yaw;
    float gyro_delta;
    float predicted;

    if (!heading_initialized) {
        (void)Heading_Init();
    }

    (void)IMUTest_Read(&imu);
    heading_data.mpu_ok = imu.MpuOk ? 1U : 0U;
    heading_data.mag_ok = imu.MagOk ? 1U : 0U;
    heading_data.gyro_z_raw = imu.GyroZ;
    heading_data.mag_yaw_deg = Heading_NormalizeInt(imu.YawDeg);
    mag_valid = (imu.MagOk && imu.MpuOk) ? true : false;

    if (!imu.MpuOk) {
        Heading_SetState(HEADING_STATE_SENSOR_FAIL, 0U);
        return false;
    }

    if (heading_data.state == HEADING_STATE_CALIBRATING) {
        heading_bias_accum += imu.GyroZ;
        heading_cal_samples++;
        heading_data.cal_progress =
            (uint8_t)((uint32_t)heading_cal_samples * 100U / HEADING_BIAS_SAMPLE_COUNT);

        if (heading_cal_samples < HEADING_BIAS_SAMPLE_COUNT) {
            return false;
        }

        heading_data.gyro_z_bias =
            (int16_t)(heading_bias_accum / (int32_t)HEADING_BIAS_SAMPLE_COUNT);
        heading_data.cal_progress = 100U;

        if (!mag_valid) {
            Heading_SetState(HEADING_STATE_SENSOR_FAIL, 0U);
            return false;
        }

        heading_fused_yaw = Heading_NormalizeFloat((float)heading_data.mag_yaw_deg);
        heading_data.yaw_deg = Heading_RoundNormalize(heading_fused_yaw);
        Heading_SetState(HEADING_STATE_READY, 1U);
        return true;
    }

    if (!heading_data.ready) {
        if (!mag_valid) {
            Heading_SetState(HEADING_STATE_SENSOR_FAIL, 0U);
            return false;
        }
        heading_fused_yaw = Heading_NormalizeFloat((float)heading_data.mag_yaw_deg);
        heading_data.yaw_deg = Heading_RoundNormalize(heading_fused_yaw);
        Heading_SetState(HEADING_STATE_READY, 1U);
        return true;
    }

    gyro_delta = ((float)(imu.GyroZ - heading_data.gyro_z_bias) *
                  (float)heading_data.gyro_z_sign) *
                 (HEADING_SAMPLE_PERIOD_SEC / HEADING_GYRO_LSB_PER_DPS);
    predicted = Heading_NormalizeFloat(heading_fused_yaw + gyro_delta);

    if (mag_valid) {
        mag_yaw = Heading_NormalizeFloat((float)heading_data.mag_yaw_deg);
        predicted = Heading_NormalizeFloat(
            predicted + (HEADING_MAG_CORR_ALPHA * Heading_AngleDiffFloat(mag_yaw, predicted)));
    }

    heading_fused_yaw = predicted;
    heading_data.yaw_deg = Heading_RoundNormalize(heading_fused_yaw);
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
