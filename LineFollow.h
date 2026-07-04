#ifndef __LINE_FOLLOW_H
#define __LINE_FOLLOW_H

#include <stdint.h>

#define LF_STATE_LOST 0U
#define LF_STATE_TRACK 1U
#define LF_STATE_RECOVER 2U
#define LF_STATE_BLACK 3U

void LineFollow_Init(void);
void LineFollow_Start(void);
void LineFollow_Stop(void);
void LineFollow_Update(void);

void LineFollow_SetBaseSpeed(int16_t speed);
void LineFollow_SetTurnLimit(int16_t limit);
void LineFollow_SetTunings(float kp, float ki, float kd);
void LineFollow_GetTunings(float *kp, float *ki, float *kd);

uint8_t LineFollow_IsEnabled(void);
uint8_t LineFollow_GetState(void);
int16_t LineFollow_GetBaseSpeed(void);
int16_t LineFollow_GetTurnLimit(void);
int16_t LineFollow_GetLastError(void);
int32_t LineFollow_GetIntegral(void);
int16_t LineFollow_GetLastDiff(void);

#endif
