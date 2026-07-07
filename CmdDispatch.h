#ifndef __CMD_DISPATCH_H
#define __CMD_DISPATCH_H

#include <stdint.h>

#define STREAM_TARGET_NONE 0U
#define STREAM_TARGET_SERIAL 1U
#define STREAM_TARGET_BLUETOOTH 2U

void CmdDispatch_Process(void);
void CmdDispatch_ApplyTargets(void);
void CmdDispatch_PrintTracking(uint8_t target);
void CmdDispatch_PrintImu(uint8_t target);
void CmdDispatch_PrintMagCal(uint8_t target);
void CmdDispatch_UpdateMagAutoCal(void);

extern volatile int16_t SpeedL;
extern volatile int16_t SpeedR;
extern volatile int16_t SpeedFiltL;
extern volatile int16_t SpeedFiltR;
extern volatile uint8_t g_Cmd;
extern volatile uint8_t g_Run;
extern volatile uint8_t g_SampleReady;
extern volatile uint8_t g_SampleTicks;
extern volatile uint8_t g_Stream;
extern volatile uint8_t g_StreamTarget;
extern volatile uint8_t g_IrStream;
extern volatile uint8_t g_IrStreamTarget;
extern volatile uint8_t g_ImuStream;
extern volatile uint8_t g_ImuStreamTarget;
extern volatile uint8_t g_MagCalStream;
extern volatile uint8_t g_MagCalStreamTarget;
extern volatile uint8_t g_MagAutoCal;
extern volatile uint8_t g_MagAutoCalProgress;
extern volatile uint8_t g_DisplayDirty;
extern volatile uint8_t g_ImuDisplayDirty;
extern volatile uint8_t g_DisplayMode;

#endif

