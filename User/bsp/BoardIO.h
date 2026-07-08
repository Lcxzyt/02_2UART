#ifndef __BOARD_IO_H
#define __BOARD_IO_H

#include <stdint.h>

void BoardIO_Init(void);
void BoardIO_Update20ms(void);

uint8_t BoardIO_MenuPressed(void);
uint8_t BoardIO_FuncPressed(void);

uint8_t BoardIO_GetMenuRawPressed(void);
uint8_t BoardIO_GetFuncRawPressed(void);

void BoardIO_LedSet(uint8_t on);
void BoardIO_LedToggle(void);
uint8_t BoardIO_LedIsOn(void);

void BoardIO_BuzzerSet(uint8_t on);
void BoardIO_BuzzerToggle(void);
uint8_t BoardIO_BuzzerIsOn(void);

#endif
