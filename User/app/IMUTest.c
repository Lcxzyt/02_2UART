#include "IMUTest.h"
#include "IMU.h"
#include "MPU6050.h"
#include "QMC5883L.h"
#include "Serial.h"
#include "Bluetooth.h"
#include <stdarg.h>
#include <stdio.h>

#define IMU_PRINTF_BUF_SIZE 160U
#define MAG_SCALE           (100.0f / 3750.0f)

static IMUTest_Data imu_last;
static bool imu_inited = false;
static bool imu_init_tried = false;
static char imu_printf_buffer[IMU_PRINTF_BUF_SIZE];

static void IMUTest_Printf(char *format, ...)
{
    va_list arg;

    va_start(arg, format);
    (void)vsnprintf(imu_printf_buffer, sizeof(imu_printf_buffer), format, arg);
    va_end(arg);

    Serial_SendString(imu_printf_buffer);
    Bluetooth_SendString(imu_printf_buffer);
}

static int16_t IMUTest_RoundDeg(float value)
{
    return (int16_t)((value >= 0.0f) ? (value + 0.5f) : (value - 0.5f));
}

static int16_t IMUTest_RoundNormalizeYaw(float yaw)
{
    while (yaw < 0.0f) {
        yaw += 360.0f;
    }
    while (yaw >= 360.0f) {
        yaw -= 360.0f;
    }
    return IMUTest_RoundDeg(yaw);
}

bool IMUTest_Init(void)
{
    if (imu_inited) return true;

    /* A previous transient I2C failure must not latch the IMU off until reset. */
    imu_init_tried = true;
    imu_inited = (IMU_Init() != 0U);

    imu_last.MpuId = MPU6050_GetID();
    imu_last.MagAddr = QMC5883L_GetAddr();
    imu_last.MagId = QMC5883L_GetID();
    imu_last.MpuOk = (imu_last.MpuId == 0x68U) || (imu_last.MpuId == 0x72U);
    imu_last.MagOk = imu_inited;


    return imu_inited;
}

bool IMUTest_ReadMagRaw(int16_t *magX, int16_t *magY, int16_t *magZ)
{
    IMU_RawData raw;

    if (!imu_init_tried) {
        (void)IMUTest_Init();
    }

    if ((!imu_inited) || (!IMU_ReadRaw(&raw))) {
        imu_last.MpuOk = false;
        imu_last.MagOk = false;
        return false;
    }

    if (magX != 0) *magX = raw.MagX;
    if (magY != 0) *magY = raw.MagY;
    if (magZ != 0) *magZ = raw.MagZ;

    imu_last.MagX = raw.MagX;
    imu_last.MagY = raw.MagY;
    imu_last.MagZ = raw.MagZ;
    imu_last.MpuOk = true;
    imu_last.MagOk = true;
    return true;
}

void IMUTest_SetMagCalibration(float offsetX, float offsetY, float offsetZ,
                               float scaleX, float scaleY, float scaleZ)
{
    (void)scaleX;
    (void)scaleY;
    (void)scaleZ;

    /* STM32 version only applies hard-iron offsets in uT. Existing command gives raw LSB. */
    IMU_SetMagOffsets(offsetX * MAG_SCALE, offsetY * MAG_SCALE, offsetZ * MAG_SCALE);
}

void IMUTest_GetMagCalibration(float *offsetX, float *offsetY, float *offsetZ,
                               float *scaleX, float *scaleY, float *scaleZ)
{
    float ox = 0.0f;
    float oy = 0.0f;
    float oz = 0.0f;

    IMU_GetMagOffsets(&ox, &oy, &oz);

    if (offsetX != 0) *offsetX = ox / MAG_SCALE;
    if (offsetY != 0) *offsetY = oy / MAG_SCALE;
    if (offsetZ != 0) *offsetZ = oz / MAG_SCALE;
    if (scaleX != 0) *scaleX = 1.0f;
    if (scaleY != 0) *scaleY = 1.0f;
    if (scaleZ != 0) *scaleZ = 1.0f;
}

bool IMUTest_Read(IMUTest_Data *data)
{
    IMU_Sample sample;
    IMU_Attitude att;

    if (!imu_init_tried) {
        (void)IMUTest_Init();
    }

    if (imu_inited && IMU_ReadSample(&sample) && sample.MagReadValid) {
        IMU_GetAttitudeRaw(&sample.Scaled, &att);
    } else {
        att.Roll = 0.0f;
        att.Pitch = 0.0f;
        att.Yaw = 0.0f;
        imu_last.MpuOk = false;
        imu_last.MagOk = false;
        if (data != 0) {
            *data = imu_last;
        }
        return false;
    }

    imu_last.AccelX = sample.Raw.AccelX;
    imu_last.AccelY = sample.Raw.AccelY;
    imu_last.AccelZ = sample.Raw.AccelZ;
    imu_last.GyroX = sample.Raw.GyroX;
    imu_last.GyroY = sample.Raw.GyroY;
    imu_last.GyroZ = sample.Raw.GyroZ;
    imu_last.MagX = sample.Raw.MagX;
    imu_last.MagY = sample.Raw.MagY;
    imu_last.MagZ = sample.Raw.MagZ;
    imu_last.RollDeg = IMUTest_RoundDeg(att.Roll);
    imu_last.PitchDeg = IMUTest_RoundDeg(att.Pitch);
    imu_last.YawDeg = IMUTest_RoundNormalizeYaw(att.Yaw);
    imu_last.MpuId = MPU6050_GetID();
    imu_last.MagAddr = QMC5883L_GetAddr();
    imu_last.MagId = QMC5883L_GetID();
    imu_last.MpuOk = true;
    imu_last.MagOk = true;

    if (data != 0) {
        *data = imu_last;
    }

    return true;
}

bool IMUTest_Print(void)
{
    IMUTest_Data data;
    bool ok;
    float ox = 0.0f;
    float oy = 0.0f;
    float oz = 0.0f;
    int ox100;
    int oy100;
    int oz100;

    ok = IMUTest_Read(&data);
    IMU_GetMagOffsets(&ox, &oy, &oz);
    ox100 = (int)((ox >= 0.0f) ? ((ox * 100.0f) + 0.5f) : ((ox * 100.0f) - 0.5f));
    oy100 = (int)((oy >= 0.0f) ? ((oy * 100.0f) + 0.5f) : ((oy * 100.0f) - 0.5f));
    oz100 = (int)((oz >= 0.0f) ? ((oz * 100.0f) + 0.5f) : ((oz * 100.0f) - 0.5f));

    IMUTest_Printf("[IMU] GitHub direct Raw->Accel Roll/Pitch + tilt-comp Mag Yaw %s\r\n",
                   ok ? "OK" : "WARN");
    IMUTest_Printf("[IMU] MPU6050 addr=0x68 id=0x%02X %s bypass=%u\r\n",
                   data.MpuId,
                   data.MpuOk ? "OK" : "FAIL",
                   data.MpuOk ? (unsigned int)MPU6050_IsBypassEnabled() : 0U);
    IMUTest_Printf("[IMU] QMC5883P addr7=0x%02X id=0x%02X %s\r\n",
                   data.MagAddr,
                   data.MagId,
                   data.MagOk ? "OK" : "FAIL");
    IMUTest_Printf("[IMU] A=%d,%d,%d G=%d,%d,%d M=%d,%d,%d\r\n",
                   (int)data.AccelX, (int)data.AccelY, (int)data.AccelZ,
                   (int)data.GyroX, (int)data.GyroY, (int)data.GyroZ,
                   (int)data.MagX, (int)data.MagY, (int)data.MagZ);
    IMUTest_Printf("[IMU] Angle R=%d P=%d Y=%d deg MagOff=%d.%02d,%d.%02d,%d.%02d uT\r\n",
                   (int)data.RollDeg, (int)data.PitchDeg, (int)data.YawDeg,
                   ox100 / 100, (ox100 < 0 ? -ox100 : ox100) % 100,
                   oy100 / 100, (oy100 < 0 ? -oy100 : oy100) % 100,
                   oz100 / 100, (oz100 < 0 ? -oz100 : oz100) % 100);
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
