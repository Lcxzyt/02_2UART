#ifndef __DISPLAY_H
#define __DISPLAY_H

#include <stdint.h>

void Display_Init(uint8_t oled_ok);
void Display_NextPage(void);
void Display_Function(void);
void Display_Update(void);

#endif
