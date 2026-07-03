#ifndef __CMD_DISPATCH_H
#define __CMD_DISPATCH_H

#include <stdint.h>

void CmdDispatch_Process(void);
void CmdDispatch_ApplyTargets(void);

extern volatile int16_t SpeedL;
extern volatile int16_t SpeedR;
extern volatile uint8_t g_Cmd;
extern volatile uint8_t g_Run;
extern volatile uint8_t g_SampleReady;
extern volatile uint8_t g_Stream;
extern volatile uint8_t g_DisplayDirty;
extern volatile uint8_t g_ImuDisplayDirty;
extern volatile uint8_t g_DisplayMode;

#endif

