#ifndef __AUTO_TRACK_TASK_H
#define __AUTO_TRACK_TASK_H

#include <stdint.h>

#define AUTO_TRACK_STATE_IDLE        0U
#define AUTO_TRACK_STATE_PRECHECK    1U
#define AUTO_TRACK_STATE_STRAIGHT_AB 2U
#define AUTO_TRACK_STATE_FOLLOW_BC   3U
#define AUTO_TRACK_STATE_STRAIGHT_CD 4U
#define AUTO_TRACK_STATE_FOLLOW_DA   5U
#define AUTO_TRACK_STATE_FINISHED    6U
#define AUTO_TRACK_STATE_ERROR       7U

#define AUTO_TRACK_ERROR_NONE        0U
#define AUTO_TRACK_ERROR_HEADING     1U
#define AUTO_TRACK_ERROR_SENSOR      2U
#define AUTO_TRACK_ERROR_NOT_WHITE   3U
#define AUTO_TRACK_ERROR_TIMEOUT     4U

void AutoTrackTask_Init(void);
void AutoTrackTask_Start(void);
void AutoTrackTask_Stop(void);
void AutoTrackTask_Update(float dt_sec);

uint8_t AutoTrackTask_IsActive(void);
uint8_t AutoTrackTask_IsRunning(void);
uint8_t AutoTrackTask_GetState(void);
uint8_t AutoTrackTask_GetError(void);
uint8_t AutoTrackTask_GetLineBits(void);
uint8_t AutoTrackTask_GetBlackCount(void);
uint8_t AutoTrackTask_GetWhiteCount(void);
uint16_t AutoTrackTask_GetLineStrength(void);
int16_t AutoTrackTask_GetLineError(void);
int16_t AutoTrackTask_GetBaseYaw(void);
int16_t AutoTrackTask_GetReverseYaw(void);
int16_t AutoTrackTask_GetTargetYaw(void);
uint16_t AutoTrackTask_GetSegmentTicks(void);
uint16_t AutoTrackTask_GetTotalTicks(void);

#endif
