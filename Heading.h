#ifndef __HEADING_H
#define __HEADING_H

#include <stdbool.h>
#include <stdint.h>

#define HEADING_STATE_IDLE        0U
#define HEADING_STATE_CALIBRATING 1U
#define HEADING_STATE_READY       2U
#define HEADING_STATE_SENSOR_FAIL 3U

typedef struct {
    int16_t yaw_deg;
    int16_t mag_yaw_deg;
    int16_t gyro_z_raw;
    int16_t gyro_z_bias;
    int8_t gyro_z_sign;
    uint8_t state;
    uint8_t ready;
    uint8_t mpu_ok;
    uint8_t mag_ok;
    uint8_t cal_progress;
} Heading_Data;

bool Heading_Init(void);
void Heading_StartCalibration(void);
bool Heading_Update(void);
bool Heading_UpdateWithDt(float dt_sec);
bool Heading_UpdateAndSnapYaw(float dt_sec);

uint8_t Heading_IsReady(void);
uint8_t Heading_GetState(void);
int16_t Heading_GetYawDeg(void);
int16_t Heading_GetMagYawDeg(void);
int16_t Heading_GetGyroZRaw(void);
int16_t Heading_GetGyroZBias(void);
uint8_t Heading_GetCalProgress(void);

void Heading_SetGyroZSign(int8_t sign);
int8_t Heading_GetGyroZSign(void);
int16_t Heading_AngleDiffDeg(int16_t target_deg, int16_t current_deg);
const Heading_Data *Heading_GetData(void);

#endif
