#ifndef __ENCODER_H
#define __ENCODER_H

#include <stdint.h>

void Encoder_Init(void);
int16_t Encoder_Get_L(void);
int16_t Encoder_Get_R(void);
int32_t Encoder_GetTotal_L(void);
int32_t Encoder_GetTotal_R(void);
uint8_t Encoder_GetPinState(void);

#endif