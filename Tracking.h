#ifndef __TRACKING_H
#define __TRACKING_H

#include <stdint.h>

#define TRACK_NUM 8U
#define TRACK_CH5_MASK 0x10U
#define TRACK_DISABLED_MASK 0U
#define TRACK_ACTIVE_MASK ((uint8_t)(0xFFU & (uint8_t)(~TRACK_DISABLED_MASK)))

typedef struct {
    uint16_t raw[TRACK_NUM];
    uint16_t filt[TRACK_NUM];
    uint16_t norm[TRACK_NUM];
    uint16_t strength;
    int16_t error;
    uint8_t valid;
} Tracking_Data;

void Tracking_Init(void);
uint8_t Tracking_Update(void);
const Tracking_Data *Tracking_GetData(void);

uint8_t Tracking_ReadRaw(uint16_t raw[TRACK_NUM]);
void Tracking_SetCalib(const uint16_t white[TRACK_NUM], const uint16_t black[TRACK_NUM]);

#endif



