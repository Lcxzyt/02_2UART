#include "IMUTest.h"
#include "MPU6050.h"
#include "QMC5883L.h"
#include "Serial.h"
#include "Bluetooth.h"
#include "delay.h"
#include <math.h>
#include <stdarg.h>
#include <stdio.h>

#define IMU_PRINTF_BUF_SIZE 160U
#define IMU_RAD_TO_DEG 57.2957795f
#define IMU_DEG_TO_RAD 0.0174532925f

/* Magnetometer calibration parameters (from 360° horizontal rotation) */
#define MAG_OFFSET_X  (-303.5f)
#define MAG_OFFSET_Y  (866.0f)
#define MAG_OFFSET_Z  (-2131.5f)

#define MAG_SCALE_X   (0.6977f)
#define MAG_SCALE_Y   (0.8515f)
#define MAG_SCALE_Z   (2.5488f)

/* Low-pass filter coefficient (EMA): 0.0 = strong filter, 1.0 = no filter */
#define FILTER_ALPHA_ACCEL  (0.3f)  /* Accelerometer filter */
#define FILTER_ALPHA_GYRO   (0.5f)  /* Gyroscope filter (less filtering, faster response) */
#define FILTER_ALPHA_MAG    (0.2f)  /* Magnetometer filter (stronger, reduces compass jitter) */
#define FILTER_ALPHA_ANGLE  (0.3f)  /* Final angle filter */

static IMUTest_Data imu_last;
static bool imu_inited = false;
static bool imu_init_tried = false;

/* Filtered sensor values */
static float filtered_ax = 0.0f;
static float filtered_ay = 0.0f;
static float filtered_az = 0.0f;
static float filtered_gx = 0.0f;
static float filtered_gy = 0.0f;
static float filtered_gz = 0.0f;
static float filtered_mx = 0.0f;
static float filtered_my = 0.0f;
static float filtered_mz = 0.0f;
static float filtered_roll = 0.0f;
static float filtered_pitch = 0.0f;
static float filtered_yaw = 0.0f;
static bool filter_initialized = false;

static void IMUTest_Printf(char *format, ...)
{
    char buf[IMU_PRINTF_BUF_SIZE];
    va_list arg;

    va_start(arg, format);
    (void)vsnprintf(buf, sizeof(buf), format, arg);
    va_end(arg);

    Serial_SendString(buf);
    Bluetooth_SendString(buf);
}

static int16_t IMUTest_RoundDeg(float value)
{
    return (int16_t)((value >= 0.0f) ? (value + 0.5f) : (value - 0.5f));
}

static float ApplyEMA(float current, float previous, float alpha)
{
    return alpha * current + (1.0f - alpha) * previous;
}

static float ApplyAngleEMA(float current, float previous, float alpha)
{
    /* Handle 360° wrap-around for angles */
    float diff = current - previous;

    /* Normalize diff to [-180, 180] */
    while (diff > 180.0f) diff -= 360.0f;
    while (diff < -180.0f) diff += 360.0f;

    /* Apply EMA on the difference */
    float result = previous + alpha * diff;

    /* Normalize result to [0, 360] */
    while (result < 0.0f) result += 360.0f;
    while (result >= 360.0f) result -= 360.0f;

    return result;
}

static void IMUTest_UpdateAngles(void)
{
    float ax = (float)imu_last.AccelX;
    float ay = (float)imu_last.AccelY;
    float az = (float)imu_last.AccelZ;
    float roll = 0.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;

    if ((imu_last.AccelX != 0) || (imu_last.AccelY != 0) || (imu_last.AccelZ != 0)) {
        roll = atan2f(ay, az) * IMU_RAD_TO_DEG;
        pitch = atan2f(-ax, sqrtf((ay * ay) + (az * az))) * IMU_RAD_TO_DEG;
    }

    if (imu_last.MagOk &&
        ((imu_last.MagX != 0) || (imu_last.MagY != 0) || (imu_last.MagZ != 0))) {
        float mx_raw = (float)imu_last.MagX;
        float my_raw = (float)imu_last.MagY;
        float mz_raw = (float)imu_last.MagZ;

        /* Apply magnetometer calibration */
        float mx = (mx_raw - MAG_OFFSET_X) * MAG_SCALE_X;
        float my = (my_raw - MAG_OFFSET_Y) * MAG_SCALE_Y;
        float mz = (mz_raw - MAG_OFFSET_Z) * MAG_SCALE_Z;
        float roll_rad = roll * IMU_DEG_TO_RAD;
        float pitch_rad = pitch * IMU_DEG_TO_RAD;
        float cos_roll = cosf(roll_rad);
        float sin_roll = sinf(roll_rad);
        float cos_pitch = cosf(pitch_rad);
        float sin_pitch = sinf(pitch_rad);
        float xh;
        float yh;

        /* 先做倾斜补偿，再用磁力计投影计算偏航角。 */
        xh = (mx * cos_pitch) + (my * sin_roll * sin_pitch) -
             (mz * cos_roll * sin_pitch);
        yh = (my * cos_roll) + (mz * sin_roll);
        yaw = atan2f(yh, xh) * IMU_RAD_TO_DEG;
        if (yaw < 0.0f) {
            yaw += 360.0f;
        }
    }

    /* Apply EMA filter to final angles */
    if (!filter_initialized) {
        filtered_roll = roll;
        filtered_pitch = pitch;
        filtered_yaw = yaw;
    } else {
        /* Roll and Pitch: normal EMA (no wrap-around needed, range -180 to +180) */
        filtered_roll = ApplyEMA(roll, filtered_roll, FILTER_ALPHA_ANGLE);
        filtered_pitch = ApplyEMA(pitch, filtered_pitch, FILTER_ALPHA_ANGLE);

        /* Yaw: special handling for 360° wrap-around */
        if (yaw > 0.0f) {  /* Only filter if yaw is valid */
            filtered_yaw = ApplyAngleEMA(yaw, filtered_yaw, FILTER_ALPHA_ANGLE);
        }
    }

    imu_last.RollDeg = IMUTest_RoundDeg(filtered_roll);
    imu_last.PitchDeg = IMUTest_RoundDeg(filtered_pitch);
    imu_last.YawDeg = IMUTest_RoundDeg(filtered_yaw);
}

static bool IMUTest_InitMag(void)
{
    static const uint8_t candidates[] = {0x0DU, 0x1AU, 0x2CU};
    uint8_t i;
    uint8_t id = 0xFFU;

    for (i = 0U; i < (uint8_t)(sizeof(candidates) / sizeof(candidates[0])); i++) {
        if (QMC5883L_ProbeAddr(candidates[i], &id)) {
            QMC5883L_SetAddr(candidates[i]);
            imu_last.MagAddr = candidates[i];
            imu_last.MagId = id;
            if (QMC5883L_Init()) {
                imu_last.MagId = QMC5883L_GetID();
                return true;
            }
        }
        Delay_ms(2U);
    }

    imu_last.MagAddr = 0U;
    imu_last.MagId = 0U;
    return false;
}

bool IMUTest_Init(void)
{
    imu_init_tried = true;
    imu_last.MpuId = MPU6050_Init();
    imu_last.MpuOk = ((imu_last.MpuId == 0x68U) || (imu_last.MpuId == 0x72U));
    imu_last.MagOk = false;

    if (imu_last.MpuOk) {
        imu_last.MagOk = IMUTest_InitMag();
    }

    imu_inited = (imu_last.MpuOk && imu_last.MagOk);
    return imu_inited;
}

bool IMUTest_Read(IMUTest_Data *data)
{
    bool mag_ok = false;
    int16_t raw_ax, raw_ay, raw_az, raw_gx, raw_gy, raw_gz;
    int16_t raw_mx, raw_my, raw_mz;

    if (!imu_init_tried) {
        (void)IMUTest_Init();
    }

    if (imu_last.MpuOk) {
        MPU6050_GetData(&raw_ax, &raw_ay, &raw_az, &raw_gx, &raw_gy, &raw_gz);

        /* Apply EMA filter to accelerometer and gyroscope */
        if (!filter_initialized) {
            /* First reading: initialize filter state */
            filtered_ax = (float)raw_ax;
            filtered_ay = (float)raw_ay;
            filtered_az = (float)raw_az;
            filtered_gx = (float)raw_gx;
            filtered_gy = (float)raw_gy;
            filtered_gz = (float)raw_gz;
        } else {
            /* Apply exponential moving average */
            filtered_ax = ApplyEMA((float)raw_ax, filtered_ax, FILTER_ALPHA_ACCEL);
            filtered_ay = ApplyEMA((float)raw_ay, filtered_ay, FILTER_ALPHA_ACCEL);
            filtered_az = ApplyEMA((float)raw_az, filtered_az, FILTER_ALPHA_ACCEL);
            filtered_gx = ApplyEMA((float)raw_gx, filtered_gx, FILTER_ALPHA_GYRO);
            filtered_gy = ApplyEMA((float)raw_gy, filtered_gy, FILTER_ALPHA_GYRO);
            filtered_gz = ApplyEMA((float)raw_gz, filtered_gz, FILTER_ALPHA_GYRO);
        }

        imu_last.AccelX = (int16_t)(filtered_ax + 0.5f);
        imu_last.AccelY = (int16_t)(filtered_ay + 0.5f);
        imu_last.AccelZ = (int16_t)(filtered_az + 0.5f);
        imu_last.GyroX = (int16_t)(filtered_gx + 0.5f);
        imu_last.GyroY = (int16_t)(filtered_gy + 0.5f);
        imu_last.GyroZ = (int16_t)(filtered_gz + 0.5f);
    }

    if (imu_last.MagOk) {
        mag_ok = QMC5883L_GetData(&raw_mx, &raw_my, &raw_mz);
        imu_last.MagOk = mag_ok;

        if (mag_ok) {
            /* Apply EMA filter to magnetometer */
            if (!filter_initialized) {
                filtered_mx = (float)raw_mx;
                filtered_my = (float)raw_my;
                filtered_mz = (float)raw_mz;
                filter_initialized = true;
            } else {
                filtered_mx = ApplyEMA((float)raw_mx, filtered_mx, FILTER_ALPHA_MAG);
                filtered_my = ApplyEMA((float)raw_my, filtered_my, FILTER_ALPHA_MAG);
                filtered_mz = ApplyEMA((float)raw_mz, filtered_mz, FILTER_ALPHA_MAG);
            }

            imu_last.MagX = (int16_t)(filtered_mx + 0.5f);
            imu_last.MagY = (int16_t)(filtered_my + 0.5f);
            imu_last.MagZ = (int16_t)(filtered_mz + 0.5f);
        }
    } else {
        imu_last.MagX = 0;
        imu_last.MagY = 0;
        imu_last.MagZ = 0;
    }

    IMUTest_UpdateAngles();

    if (data != 0) {
        *data = imu_last;
    }

    return (imu_last.MpuOk && imu_last.MagOk);
}

bool IMUTest_Print(void)
{
    IMUTest_Data data;
    bool ok;

    ok = IMUTest_Read(&data);
    IMUTest_Printf("[IMU] I2C0 SCL=PA1 SDA=PA0 %s\r\n", ok ? "OK" : "WARN");
    IMUTest_Printf("[IMU] MPU6050 addr=0x68 id=0x%02X %s bypass=%u\r\n",
                   data.MpuId,
                   data.MpuOk ? "OK" : "FAIL",
                   data.MpuOk ? (unsigned int)MPU6050_IsBypassEnabled() : 0U);
    IMUTest_Printf("[IMU] QMC5883P addr=0x%02X id=0x%02X %s\r\n",
                   data.MagAddr,
                   data.MagId,
                   data.MagOk ? "OK" : "FAIL");
    IMUTest_Printf("[IMU] A=%d,%d,%d G=%d,%d,%d M=%d,%d,%d\r\n",
                   (int)data.AccelX, (int)data.AccelY, (int)data.AccelZ,
                   (int)data.GyroX, (int)data.GyroY, (int)data.GyroZ,
                   (int)data.MagX, (int)data.MagY, (int)data.MagZ);
    IMUTest_Printf("[IMU] Angle R=%d P=%d Y=%d deg\r\n",
                   (int)data.RollDeg, (int)data.PitchDeg, (int)data.YawDeg);
    return ok;
}

void IMUTest_GetLast(IMUTest_Data *data)
{
    if (data != 0) {
        *data = imu_last;
    }
}

bool IMUTest_IsReady(void)
{
    return imu_inited;
}

