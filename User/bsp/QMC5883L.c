#include "QMC5883L.h"
#include "MyI2C.h"
#include "delay.h"

static uint8_t qmc5883_addr = 0x0DU;

void QMC5883L_SetAddr(uint8_t addr7)
{
    qmc5883_addr = addr7;
}

uint8_t QMC5883L_GetAddr(void)
{
    return qmc5883_addr;
}

bool QMC5883L_ProbeAddr(uint8_t addr7, uint8_t *chipId)
{
    uint8_t id = 0xFFU;
    if (!MyI2C_ReadReg(addr7, QMC5883L_CHIPID, &id)) {
        return false;
    }
    if (chipId != 0) {
        *chipId = id;
    }
    return true;
}

static bool QMC5883L_WriteReg(uint8_t regAddr, uint8_t data)
{
    return MyI2C_WriteReg(qmc5883_addr, regAddr, data);
}

static uint8_t QMC5883L_ReadReg(uint8_t regAddr)
{
    uint8_t data = 0xFFU;
    (void)MyI2C_ReadReg(qmc5883_addr, regAddr, &data);
    return data;
}

bool QMC5883L_Init(void)
{
    if (!QMC5883L_WriteReg(QMC5883L_CR2, QMC5883L_CR2_SOFT_RST)) {
        return false;
    }
    Delay_ms(5U);

    if (!QMC5883L_WriteReg(QMC5883L_CR2,
        (uint8_t)(QMC5883L_CR2_RNG_8G | QMC5883L_CR2_SETRST_ON))) {
        return false;
    }

    if (!QMC5883L_WriteReg(QMC5883L_CR1,
        (uint8_t)(QMC5883L_CR1_OSR2_1 | QMC5883L_CR1_OSR1_8 |
                  QMC5883L_CR1_ODR_200HZ | QMC5883L_CR1_MODE_SUSPEND))) {
        return false;
    }
    Delay_ms(1U);

    if (!QMC5883L_WriteReg(QMC5883L_CR1,
        (uint8_t)(QMC5883L_CR1_OSR2_1 | QMC5883L_CR1_OSR1_8 |
                  QMC5883L_CR1_ODR_200HZ | QMC5883L_CR1_MODE_CONTINUOUS))) {
        return false;
    }
    Delay_ms(10U);

    return true;
}

uint8_t QMC5883L_GetID(void)
{
    return QMC5883L_ReadReg(QMC5883L_CHIPID);
}

uint8_t QMC5883L_GetStatus(void)
{
    return QMC5883L_ReadReg(QMC5883L_SR);
}

uint8_t QMC5883L_IsDataReady(void)
{
    return (QMC5883L_GetStatus() & QMC5883L_SR_DRDY) ? 1U : 0U;
}

bool QMC5883L_GetData(int16_t *magX, int16_t *magY, int16_t *magZ)
{
    uint8_t buf[6];

    if (!MyI2C_ReadRegs(qmc5883_addr, QMC5883L_DXRL, buf, sizeof(buf))) {
        *magX = 0;
        *magY = 0;
        *magZ = 0;
        return false;
    }

    *magX = (int16_t)(((uint16_t)buf[1] << 8) | buf[0]);
    *magY = (int16_t)(((uint16_t)buf[3] << 8) | buf[2]);
    *magZ = (int16_t)(((uint16_t)buf[5] << 8) | buf[4]);
    return true;
}
