#ifndef __HEADING_H
#define __HEADING_H

#include <stdbool.h>
#include <stdint.h>

#define HEADING_STATE_IDLE        0U
#define HEADING_STATE_CALIBRATING 1U
#define HEADING_STATE_READY       2U
#define HEADING_STATE_SENSOR_FAIL 3U

typedef struct {
    float yaw_fused_deg;
    float yaw_continuous_deg;
    float yaw_mag_deg;
    float roll_deg;
    float pitch_deg;
    float gyro_z_dps;
    float gyro_bias_z_dps;
    float mag_norm_uT;
    float mag_norm_ref_uT;
    uint8_t gyro_calibrated;
    uint8_t mag_valid;
    uint8_t mag_disturbed;
    uint8_t ready;
    uint8_t state;
} Heading_Data;

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
float Heading_GetYawDegF(void);
float Heading_GetMagYawDegF(void);
const Heading_Data *Heading_GetData(void);

/* ── 工具 ── */
int16_t Heading_AngleDiffDeg(int16_t target_deg, int16_t current_deg);
float Heading_AngleDiffDegF(float target_deg, float current_deg);

#endif
