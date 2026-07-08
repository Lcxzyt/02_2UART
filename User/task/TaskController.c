#include "TaskController.h"
#include "AutoTrackTask.h"
#include "BoardIO.h"

/* ── 内部状态 ── */
static TaskMode   tc_mode;
static uint8_t    tc_running;

/* ── 模式名称表 ── */
static const char *mode_text[TASK_MODE_COUNT] = {
    "MODE:IDLE       ",
    "MODE:AUTO TRACK "
};

/* ── 实现 ── */
void TaskController_Init(void)
{
    tc_mode    = TASK_MODE_IDLE;
    tc_running = 0U;
    AutoTrackTask_Init();
}

void TaskController_Start(TaskMode mode)
{
    if (mode >= TASK_MODE_COUNT) return;

    /* 先停掉当前正在跑的任何东西 */
    TaskController_Stop();

    tc_mode = mode;

    switch (tc_mode) {
        case TASK_MODE_IDLE:
            tc_running = 0U;
            return;

        case TASK_MODE_AUTO_TRACK:
            AutoTrackTask_Start();
            tc_running = 1U;
            return;

        default:
            tc_running = 0U;
            return;
    }
}

void TaskController_Stop(void)
{
    if (!tc_running) return;

    switch (tc_mode) {
        case TASK_MODE_AUTO_TRACK:
            AutoTrackTask_Stop();
            break;
        default:
            break;
    }

    BoardIO_BuzzerSet(0U);
    BoardIO_LedSet(0U);
    tc_mode    = TASK_MODE_IDLE;
    tc_running = 0U;
}

void TaskController_Update(float dt_sec)
{
    if (!tc_running) return;

    switch (tc_mode) {
        case TASK_MODE_AUTO_TRACK:
            AutoTrackTask_Update(dt_sec);
            /* 巡迹完成或出错 → 自动回到 IDLE */
            if (AutoTrackTask_GetState() == AUTO_TRACK_STATE_FINISHED ||
                AutoTrackTask_GetState() == AUTO_TRACK_STATE_ERROR) {
                if (!AutoTrackTask_IsActive()) {
                    tc_running = 0U;
                    tc_mode    = TASK_MODE_IDLE;
                }
            }
            break;

        case TASK_MODE_IDLE:
        default:
            break;
    }
}

TaskMode TaskController_GetMode(void)
{
    return tc_mode;
}

uint8_t TaskController_IsRunning(void)
{
    return tc_running;
}

const char* TaskController_GetModeText(void)
{
    return mode_text[tc_mode];
}

const char* TaskController_GetStateText(void)
{
    if (!tc_running) {
        return "ST:IDLE         ";
    }

    switch (tc_mode) {
        case TASK_MODE_AUTO_TRACK:
            return AutoTrackTask_GetStateText();
        default:
            return "ST:UNKNOWN      ";
    }
}
