#ifndef __BLUETOOTH_H
#define __BLUETOOTH_H

#include <stdint.h>

void Bluetooth_Init(void);
void Bluetooth_SendByte(uint8_t byte);
void Bluetooth_SendString(char *string);
void Bluetooth_Printf(char *format, ...);
void Bluetooth_PollRx(void);

void Bluetooth_RingBuf_Put(uint8_t ch);
uint8_t Bluetooth_RingBuf_Get(void);
uint8_t Bluetooth_RingBuf_IsEmpty(void);
uint32_t Bluetooth_GetRxCount(void);
uint32_t Bluetooth_GetIrqCount(void);

#endif
