#ifndef __QMC5883L_H
#define __QMC5883L_H

#include <stdbool.h>
#include <stdint.h>

#define QMC5883L_CHIPID             0x00U
#define QMC5883L_DXRL               0x01U
#define QMC5883L_DXRH               0x02U
#define QMC5883L_DYRL               0x03U
#define QMC5883L_DYRH               0x04U
#define QMC5883L_DZRL               0x05U
#define QMC5883L_DZRH               0x06U
#define QMC5883L_SR                 0x09U
#define QMC5883L_CR1                0x0AU
#define QMC5883L_CR2                0x0BU
#define QMC5883L_AXIS_SIGN          0x29U
#define QMC5883L_SR_DRDY            (1U << 0)
#define QMC5883L_SR_OVFL            (1U << 1)
#define QMC5883L_CR1_OSR2_1         (0U << 6)
#define QMC5883L_CR1_OSR1_8         (0U << 4)
#define QMC5883L_CR1_ODR_200HZ      (3U << 2)
#define QMC5883L_CR1_MODE_SUSPEND   (0U << 0)
#define QMC5883L_CR1_MODE_CONTINUOUS (3U << 0)
#define QMC5883L_CR2_SOFT_RST       (1U << 7)
#define QMC5883L_CR2_RNG_8G         (2U << 2)
#define QMC5883L_CR2_SETRST_ON      (0U << 0)

void    QMC5883L_SetAddr(uint8_t addr7);
uint8_t QMC5883L_GetAddr(void);
bool    QMC5883L_ProbeAddr(uint8_t addr7, uint8_t *chipId);
bool    QMC5883L_Init(void);
uint8_t QMC5883L_GetID(void);
uint8_t QMC5883L_GetStatus(void);
uint8_t QMC5883L_IsDataReady(void);
bool    QMC5883L_ReadStatus(uint8_t *status);
bool    QMC5883L_GetData(int16_t *magX, int16_t *magY, int16_t *magZ);
bool    QMC5883L_GetDataChecked(int16_t *magX, int16_t *magY, int16_t *magZ,
                                uint8_t *status);

#endif
