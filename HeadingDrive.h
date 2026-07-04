#ifndef __HEADING_DRIVE_H
#define __HEADING_DRIVE_H

#include <stdint.h>

#define HD_STATE_IDLE        0U
#define HD_STATE_CALIBRATING 1U
#define HD_STATE_RUN         2U
#define HD_STATE_SENSOR_FAIL 3U

typedef struct {
    uint8_t enabled;
    uint8_t state;
    int16_t base_speed;
    int16_t target_yaw;
    int16_t current_yaw;
    int16_t error_deg;
    int16_t last_diff;
    int32_t integral;
    int16_t diff_limit;
    int8_t output_sign;
} HeadingDrive_Data;

void HeadingDrive_Init(void);
void HeadingDrive_Start(void);
void HeadingDrive_Stop(void);
void HeadingDrive_Update(void);

void HeadingDrive_SetBaseSpeed(int16_t speed);
int16_t HeadingDrive_GetBaseSpeed(void);

void HeadingDrive_SetTunings(float kp, float ki, float kd);
void HeadingDrive_GetTunings(float *kp, float *ki, float *kd);
void HeadingDrive_SetDiffLimit(int16_t limit);
int16_t HeadingDrive_GetDiffLimit(void);
void HeadingDrive_SetOutputSign(int8_t sign);
int8_t HeadingDrive_GetOutputSign(void);

uint8_t HeadingDrive_IsEnabled(void);
uint8_t HeadingDrive_GetState(void);
int16_t HeadingDrive_GetTargetYaw(void);
int16_t HeadingDrive_GetCurrentYaw(void);
int16_t HeadingDrive_GetErrorDeg(void);
int16_t HeadingDrive_GetLastDiff(void);
int32_t HeadingDrive_GetIntegral(void);
const HeadingDrive_Data *HeadingDrive_GetData(void);

#endif
