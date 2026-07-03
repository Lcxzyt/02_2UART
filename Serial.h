#ifndef __SERIAL_H
#define __SERIAL_H

#include <stdint.h>

void Serial_Init(void);
void Serial_SendByte(uint8_t Byte);
void Serial_SendArray(uint8_t *Array, uint16_t Length);
void Serial_SendString(char *String);
void Serial_SendNumber(uint32_t Number, uint8_t Length);
void Serial_Printf(char *format, ...);
void Serial_PrintFloat(float val, uint8_t intDig, uint8_t fracDig);

void Serial_RingBuf_Put(uint8_t ch);
uint8_t Serial_RingBuf_Get(void);
uint8_t Serial_RingBuf_IsEmpty(void);

#endif
