#include "MyI2C.h"
#include "ti_msp_dl_config.h"

#define MYI2C_TIMEOUT_COUNT (10000U)

void MyI2C_Recover(void)
{
    DL_I2C_resetControllerTransfer(I2C_0_INST);
    DL_I2C_flushControllerTXFIFO(I2C_0_INST);
    DL_I2C_flushControllerRXFIFO(I2C_0_INST);
}

static bool MyI2C_Fail(void)
{
    MyI2C_Recover();
    return false;
}

static bool MyI2C_WaitIdle(void)
{
    uint32_t timeout = MYI2C_TIMEOUT_COUNT;

    while ((DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_IDLE) == 0U) {
        if (--timeout == 0U) {
            return false;
        }
    }
    return true;
}

static bool MyI2C_WaitDone(void)
{
    uint32_t timeout = MYI2C_TIMEOUT_COUNT;

    while ((DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_BUSY) != 0U) {
        if (--timeout == 0U) {
            return false;
        }
    }

    return (DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_ERROR) == 0U;
}

bool MyI2C_WriteBytes(uint8_t slaveAddr7, const uint8_t *data, uint32_t len)
{
    uint32_t offset = 0U;

    if ((data == 0) || (len == 0U)) {
        return false;
    }

    while (offset < len) {
        uint32_t chunk = len - offset;
        if (chunk > 8U) {
            chunk = 8U;
        }

        if (!MyI2C_WaitIdle()) {
            return MyI2C_Fail();
        }

        (void)DL_I2C_fillControllerTXFIFO(I2C_0_INST, (uint8_t *)&data[offset], chunk);
        DL_I2C_startControllerTransfer(I2C_0_INST, slaveAddr7, DL_I2C_CONTROLLER_DIRECTION_TX, chunk);
        delay_cycles(32U);

        if (!MyI2C_WaitDone()) {
            return MyI2C_Fail();
        }
        offset += chunk;
    }

    return true;
}

bool MyI2C_WriteReg(uint8_t slaveAddr7, uint8_t regAddr, uint8_t data)
{
    uint8_t tx[2];
    tx[0] = regAddr;
    tx[1] = data;
    return MyI2C_WriteBytes(slaveAddr7, tx, 2U);
}

bool MyI2C_ReadReg(uint8_t slaveAddr7, uint8_t regAddr, uint8_t *data)
{
    return MyI2C_ReadRegs(slaveAddr7, regAddr, data, 1U);
}

bool MyI2C_ReadRegs(uint8_t slaveAddr7, uint8_t regAddr, uint8_t *buf, uint32_t len)
{
    uint32_t i;

    if ((buf == 0) || (len == 0U)) {
        return false;
    }

    if (!MyI2C_WaitIdle()) {
        return MyI2C_Fail();
    }

    (void)DL_I2C_fillControllerTXFIFO(I2C_0_INST, &regAddr, 1U);
    DL_I2C_startControllerTransferAdvanced(I2C_0_INST, slaveAddr7,
        DL_I2C_CONTROLLER_DIRECTION_TX, 1U, DL_I2C_CONTROLLER_START_ENABLE,
        DL_I2C_CONTROLLER_STOP_DISABLE, DL_I2C_CONTROLLER_ACK_DISABLE);
    delay_cycles(32U);

    if (!MyI2C_WaitDone()) {
        return MyI2C_Fail();
    }

    DL_I2C_startControllerTransferAdvanced(I2C_0_INST, slaveAddr7,
        DL_I2C_CONTROLLER_DIRECTION_RX, (uint16_t)len,
        DL_I2C_CONTROLLER_START_ENABLE, DL_I2C_CONTROLLER_STOP_ENABLE,
        DL_I2C_CONTROLLER_ACK_DISABLE);
    delay_cycles(32U);

    for (i = 0U; i < len; i++) {
        uint32_t timeout = MYI2C_TIMEOUT_COUNT;
        while (DL_I2C_isControllerRXFIFOEmpty(I2C_0_INST)) {
            if (--timeout == 0U) {
                return MyI2C_Fail();
            }
        }
        buf[i] = DL_I2C_receiveControllerData(I2C_0_INST);
    }

    if (!MyI2C_WaitDone()) {
        return MyI2C_Fail();
    }

    return true;
}

bool MyI2C_IsDeviceReady(uint8_t slaveAddr7)
{
    uint8_t dummy = 0x00U;
    return MyI2C_WriteBytes(slaveAddr7, &dummy, 1U);
}