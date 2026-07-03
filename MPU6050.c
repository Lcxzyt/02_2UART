#include "MPU6050.h"
#include "MyI2C.h"
#include "delay.h"

bool MPU6050_WriteReg(uint8_t regAddress, uint8_t data)
{
    return MyI2C_WriteReg(MPU6050_ADDR_7BIT, regAddress, data);
}

bool MPU6050_ReadReg(uint8_t regAddress, uint8_t *data)
{
    return MyI2C_ReadReg(MPU6050_ADDR_7BIT, regAddress, data);
}

bool MPU6050_ReadRegs(uint8_t regAddress, uint8_t *buf, uint8_t len)
{
    return MyI2C_ReadRegs(MPU6050_ADDR_7BIT, regAddress, buf, len);
}

uint8_t MPU6050_ReadRegValue(uint8_t regAddress)
{
    uint8_t data = 0xFFU;
    (void)MPU6050_ReadReg(regAddress, &data);
    return data;
}

uint8_t MPU6050_Init(void)
{
    uint8_t id = 0U;

    (void)MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x80U);
    Delay_ms(100U);
    (void)MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x00U);
    Delay_ms(50U);

    if (!MPU6050_ReadReg(MPU6050_WHO_AM_I, &id)) {
        return 0U;
    }
    if ((id != 0x68U) && (id != 0x72U)) {
        return 0U;
    }

    (void)MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x00U);
    Delay_ms(10U);
    (void)MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x07U);
    (void)MPU6050_WriteReg(MPU6050_CONFIG, 0x06U);
    (void)MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x18U);
    (void)MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x00U);
    (void)MPU6050_WriteReg(MPU6050_PWR_MGMT_2, 0x00U);
    (void)MPU6050_WriteReg(MPU6050_INT_ENABLE, 0x00U);
    (void)MPU6050_WriteReg(MPU6050_USER_CTRL, 0x00U);
    (void)MPU6050_WriteReg(MPU6050_INT_PIN_CFG, 0x02U);
    Delay_ms(50U);

    return id;
}

uint8_t MPU6050_GetID(void)
{
    return MPU6050_ReadRegValue(MPU6050_WHO_AM_I);
}

uint8_t MPU6050_IsBypassEnabled(void)
{
    return (MPU6050_ReadRegValue(MPU6050_INT_PIN_CFG) & 0x02U) ? 1U : 0U;
}

void MPU6050_GetData(int16_t *accX, int16_t *accY, int16_t *accZ,
                     int16_t *gyroX, int16_t *gyroY, int16_t *gyroZ)
{
    uint8_t buf[14];

    if (!MPU6050_ReadRegs(MPU6050_ACCEL_XOUT_H, buf, sizeof(buf))) {
        *accX = 0;
        *accY = 0;
        *accZ = 0;
        *gyroX = 0;
        *gyroY = 0;
        *gyroZ = 0;
        return;
    }

    *accX  = (int16_t)(((uint16_t)buf[0] << 8) | buf[1]);
    *accY  = (int16_t)(((uint16_t)buf[2] << 8) | buf[3]);
    *accZ  = (int16_t)(((uint16_t)buf[4] << 8) | buf[5]);
    *gyroX = (int16_t)(((uint16_t)buf[8] << 8) | buf[9]);
    *gyroY = (int16_t)(((uint16_t)buf[10] << 8) | buf[11]);
    *gyroZ = (int16_t)(((uint16_t)buf[12] << 8) | buf[13]);
}
