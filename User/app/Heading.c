#include "Heading.h"
#include "IMUTest.h"

#define HEADING_SAMPLE_PERIOD_SEC (0.02f)

/* ── 内部状态 ── */
static int16_t heading_yaw_deg;
static uint8_t heading_state;
static uint8_t heading_ready;
static uint8_t heading_initialized;

static void Heading_SetState(uint8_t state, uint8_t ready)
{
    heading_state = state;
    heading_ready  = ready;
}

/* ══════════════════════════════════════════
   生命周期 — 全部委托给 IMUTest
   ══════════════════════════════════════════ */
bool Heading_Init(void)
{
    bool ok = IMUTest_Init();

    heading_yaw_deg     = 0;
    heading_initialized = ok ? 1U : 0U;

    if (!ok) {
        Heading_SetState(HEADING_STATE_SENSOR_FAIL, 0U);
        return false;
    }

    Heading_SetState(HEADING_STATE_IDLE, 0U);
    return true;
}

void Heading_StartCalibration(void)
{
    if (!heading_initialized) {
        (void)Heading_Init();
    }

    if (heading_initialized) {
        Heading_SetState(HEADING_STATE_READY, 1U);
    } else {
        Heading_SetState(HEADING_STATE_SENSOR_FAIL, 0U);
    }
}

bool Heading_UpdateWithDt(float dt_sec)
{
    IMUTest_Data imu;
    (void)dt_sec;

    if (!heading_initialized) {
        if (!Heading_Init()) return false;
    }

    if (!IMUTest_Read(&imu)) {
        Heading_SetState(HEADING_STATE_SENSOR_FAIL, 0U);
        return false;
    }

    heading_yaw_deg = (int16_t)imu.YawDeg;
    Heading_SetState(HEADING_STATE_READY, 1U);
    return true;
}

bool Heading_Update(void)
{
    return Heading_UpdateWithDt(HEADING_SAMPLE_PERIOD_SEC);
}

bool Heading_UpdateAndSnapYaw(float dt_sec)
{
    /* 简化：直接委托给 UpdateWithDt。
       不再需要特殊的 "snap" 逻辑，因为 IMUTest 每帧都会刷新磁力计 yaw。 */
    return Heading_UpdateWithDt(dt_sec);
}

/* ══════════════════════════════════════════
   状态查询
   ══════════════════════════════════════════ */
uint8_t Heading_IsReady(void)  { return heading_ready; }
uint8_t Heading_GetState(void) { return heading_state; }
int16_t Heading_GetYawDeg(void) { return heading_yaw_deg; }

/* ══════════════════════════════════════════
   工具
   ══════════════════════════════════════════ */
int16_t Heading_AngleDiffDeg(int16_t target_deg, int16_t current_deg)
{
    int16_t diff = (int16_t)(target_deg - current_deg);
    while (diff > 180) {
        diff = (int16_t)(diff - 360);
    }
    while (diff < -180) {
        diff = (int16_t)(diff + 360);
    }
    return diff;
}
