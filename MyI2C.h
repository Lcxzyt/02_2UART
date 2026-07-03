#ifndef __MYI2C_H
#define __MYI2C_H

#include <stdbool.h>
#include <stdint.h>

bool MyI2C_WriteBytes(uint8_t slaveAddr7, const uint8_t *data, uint32_t len);
bool MyI2C_WriteReg(uint8_t slaveAddr7, uint8_t regAddr, uint8_t data);
bool MyI2C_ReadReg(uint8_t slaveAddr7, uint8_t regAddr, uint8_t *data);
bool MyI2C_ReadRegs(uint8_t slaveAddr7, uint8_t regAddr, uint8_t *buf, uint32_t len);
bool MyI2C_IsDeviceReady(uint8_t slaveAddr7);
void MyI2C_Recover(void);

#endif
