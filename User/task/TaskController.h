#ifndef __TASK_CONTROLLER_H
#define __TASK_CONTROLLER_H

#include <stdint.h>

/* ── 运行模式 ── */
typedef enum {
    TASK_MODE_IDLE = 0,
    TASK_MODE_AUTO_TRACK,
    TASK_MODE_COUNT
} TaskMode;

/* ── 主循环接口 ── */
void TaskController_Init(void);
void TaskController_Start(TaskMode mode);
void TaskController_Stop(void);
void TaskController_Update(float dt_sec);

/* ── 状态查询 ── */
TaskMode TaskController_GetMode(void);
uint8_t  TaskController_IsRunning(void);
const char* TaskController_GetModeText(void);
const char* TaskController_GetStateText(void);

#endif
