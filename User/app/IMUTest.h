#ifndef __IMU_TEST_H
#define __IMU_TEST_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    int16_t AccelX;
    int16_t AccelY;
    int16_t AccelZ;
    int16_t GyroX;
    int16_t GyroY;
    int16_t GyroZ;
    int16_t MagX;
    int16_t MagY;
    int16_t MagZ;
    int16_t RollDeg;
    int16_t PitchDeg;
    int16_t YawDeg;
    uint8_t MpuId;
    uint8_t MagId;
    uint8_t MagAddr;
    bool MpuOk;
    bool MagOk;
} IMUTest_Data;

bool IMUTest_Init(void);
bool IMUTest_Read(IMUTest_Data *data);
bool IMUTest_ReadMagRaw(int16_t *magX, int16_t *magY, int16_t *magZ);
void IMUTest_SetMagCalibration(float offsetX, float offsetY, float offsetZ,
                               float scaleX, float scaleY, float scaleZ);
void IMUTest_GetMagCalibration(float *offsetX, float *offsetY, float *offsetZ,
                               float *scaleX, float *scaleY, float *scaleZ);
bool IMUTest_Print(void);
void IMUTest_GetLast(IMUTest_Data *data);
bool IMUTest_IsReady(void);

#endif
