#ifndef __HEADING_H
#define __HEADING_H

#include <stdbool.h>
#include <stdint.h>

#define HEADING_STATE_IDLE        0U
#define HEADING_STATE_CALIBRATING 1U
#define HEADING_STATE_READY       2U
#define HEADING_STATE_SENSOR_FAIL 3U

/* ── 生命周期 ── */
bool Heading_Init(void);
void Heading_StartCalibration(void);
bool Heading_Update(void);
bool Heading_UpdateWithDt(float dt_sec);
bool Heading_UpdateAndSnapYaw(float dt_sec);

/* ── 状态 ── */
uint8_t Heading_IsReady(void);
uint8_t Heading_GetState(void);
int16_t Heading_GetYawDeg(void);

/* ── 工具 ── */
int16_t Heading_AngleDiffDeg(int16_t target_deg, int16_t current_deg);

#endif
