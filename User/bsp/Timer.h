#ifndef __TIMER_H
#define __TIMER_H

#include <stdint.h>

void Timer_Init(void);
void Timer_ResetSpeedFilter(void);
void Timer_NotifyMainAlive(void);
uint8_t Timer_WasSafetyStop(void);

#endif
