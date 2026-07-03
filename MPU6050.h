#ifndef __MPU6050_H
#define __MPU6050_H

#include <stdbool.h>
#include <stdint.h>
#include "MPU6050_Reg.h"

#define MPU6050_ADDR_7BIT 0x68U

bool    MPU6050_WriteReg(uint8_t regAddress, uint8_t data);
bool    MPU6050_ReadReg(uint8_t regAddress, uint8_t *data);
bool    MPU6050_ReadRegs(uint8_t regAddress, uint8_t *buf, uint8_t len);
uint8_t MPU6050_ReadRegValue(uint8_t regAddress);
uint8_t MPU6050_Init(void);
uint8_t MPU6050_GetID(void);
uint8_t MPU6050_IsBypassEnabled(void);
void    MPU6050_GetData(int16_t *accX, int16_t *accY, int16_t *accZ,
                        int16_t *gyroX, int16_t *gyroY, int16_t *gyroZ);

#endif
