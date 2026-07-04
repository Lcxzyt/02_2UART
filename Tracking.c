#include "Tracking.h"
#include "ti_msp_dl_config.h"
#include <stdbool.h>

#define TRACK_ADC_INST ADC1
#define TRACK_ADC_TIMEOUT 20000U
#define TRACK_LOST_STRENGTH 80U
#define TRACK_MAX_ERR 3000
#define TRACK_FILTER_SHIFT 1U

/*
 * 四路模拟量接线顺序：
 * Track[0] L2 最左：PA16 / ADC1-A1-1
 * Track[1] L1 左内：PA17 / ADC1-A1-2
 * Track[2] R1 右内：PB17 / ADC1-A1-4
 * Track[3] R2 最右：PB18 / ADC1-A1-5
 */
static const uint32_t track_iomux[TRACK_NUM] = {
    IOMUX_PINCM38,
    IOMUX_PINCM39,
    IOMUX_PINCM43,
    IOMUX_PINCM44,
};

static const DL_ADC12_MEM_IDX track_mem[TRACK_NUM] = {
    DL_ADC12_MEM_IDX_0,
    DL_ADC12_MEM_IDX_1,
    DL_ADC12_MEM_IDX_2,
    DL_ADC12_MEM_IDX_3,
};

static const uint32_t track_chan[TRACK_NUM] = {
    DL_ADC12_INPUT_CHAN_1,
    DL_ADC12_INPUT_CHAN_2,
    DL_ADC12_INPUT_CHAN_4,
    DL_ADC12_INPUT_CHAN_5,
};

/* 实车 2026-07-04 x 指令标定：白底约 180,260,175,400；全黑线约 2690,3370,2520,2020。 */
static uint16_t track_white[TRACK_NUM] = {180U, 260U, 175U, 400U};
static uint16_t track_black[TRACK_NUM] = {2690U, 3370U, 2520U, 2020U};
static const int16_t track_weight[TRACK_NUM] = {-3000, -1000, 1000, 3000};
static Tracking_Data track_data;
static int32_t track_filter[TRACK_NUM];
static int16_t track_last_error;
static uint8_t track_filter_ready;
static uint8_t track_inited;

static uint16_t Tracking_NormalizeOne(uint16_t raw, uint16_t white, uint16_t black)
{
    int32_t span = (int32_t)black - (int32_t)white;
    int32_t value;

    if (span == 0) return 0U;

    value = ((int32_t)raw - (int32_t)white) * 1000L / span;
    if (value < 0) value = 0;
    if (value > 1000) value = 1000;
    return (uint16_t)value;
}
static uint16_t Tracking_FilterToUint16(int32_t value)
{
    int32_t rounded = value + (1L << (TRACK_FILTER_SHIFT - 1U));

    rounded >>= TRACK_FILTER_SHIFT;
    if (rounded < 0) rounded = 0;
    if (rounded > 4095) rounded = 4095;
    return (uint16_t)rounded;
}

static uint16_t Tracking_FilterUpdate(uint8_t index, uint16_t raw)
{
    int32_t target = (int32_t)raw << TRACK_FILTER_SHIFT;

    track_filter[index] += (target - track_filter[index]) >> TRACK_FILTER_SHIFT;
    return Tracking_FilterToUint16(track_filter[index]);
}
void Tracking_SetCalib(const uint16_t white[TRACK_NUM], const uint16_t black[TRACK_NUM])
{
    uint8_t i;

    if ((white == 0) || (black == 0)) return;

    for (i = 0U; i < TRACK_NUM; i++) {
        track_white[i] = white[i];
        track_black[i] = black[i];
    }
    track_filter_ready = 0U;
}

void Tracking_Init(void)
{
    static const DL_ADC12_ClockConfig adc_clock = {
        .clockSel = DL_ADC12_CLOCK_ULPCLK,
        .divideRatio = DL_ADC12_CLOCK_DIVIDE_8,
        .freqRange = DL_ADC12_CLOCK_FREQ_RANGE_32_TO_40,
    };
    uint8_t i;

    for (i = 0U; i < TRACK_NUM; i++) {
        DL_GPIO_initPeripheralAnalogFunction(track_iomux[i]);
    }

    DL_ADC12_reset(TRACK_ADC_INST);
    DL_ADC12_enablePower(TRACK_ADC_INST);
    delay_cycles(POWER_STARTUP_DELAY);

    DL_ADC12_setClockConfig(TRACK_ADC_INST, &adc_clock);
    DL_ADC12_setSampleTime0(TRACK_ADC_INST, 32U);
    DL_ADC12_initSeqSample(TRACK_ADC_INST,
        DL_ADC12_REPEAT_MODE_ENABLED,
        DL_ADC12_SAMPLING_SOURCE_AUTO,
        DL_ADC12_TRIG_SRC_SOFTWARE,
        DL_ADC12_SEQ_START_ADDR_00,
        DL_ADC12_SEQ_END_ADDR_03,
        DL_ADC12_SAMP_CONV_RES_12_BIT,
        DL_ADC12_SAMP_CONV_DATA_FORMAT_UNSIGNED);

    for (i = 0U; i < TRACK_NUM; i++) {
        DL_ADC12_configConversionMem(TRACK_ADC_INST,
            track_mem[i],
            track_chan[i],
            DL_ADC12_REFERENCE_VOLTAGE_VDDA,
            DL_ADC12_SAMPLE_TIMER_SOURCE_SCOMP0,
            DL_ADC12_AVERAGING_MODE_DISABLED,
            DL_ADC12_BURN_OUT_SOURCE_DISABLED,
            DL_ADC12_TRIGGER_MODE_AUTO_NEXT,
            DL_ADC12_WINDOWS_COMP_MODE_DISABLED);
    }

    DL_ADC12_enableConversions(TRACK_ADC_INST);
    DL_ADC12_startConversion(TRACK_ADC_INST);
    track_filter_ready = 0U;
    track_last_error = 0;
    track_inited = 1U;
}

uint8_t Tracking_ReadRaw(uint16_t raw[TRACK_NUM])
{
    uint8_t i;

    if ((raw == 0) || !track_inited) return 0U;

    /* 和原 STM32 工程一致：ADC 后台持续扫描，这里只读取当前缓存结果。 */
    for (i = 0U; i < TRACK_NUM; i++) {
        raw[i] = DL_ADC12_getMemResult(TRACK_ADC_INST, track_mem[i]);
    }
    return 1U;
}

uint8_t Tracking_Update(void)
{
    uint8_t i;
    uint32_t strength = 0U;
    int32_t weighted = 0;

    track_data.valid = Tracking_ReadRaw(track_data.raw);
    if (!track_data.valid) {
        return 0U;
    }
    if (!track_filter_ready) {
        for (i = 0U; i < TRACK_NUM; i++) {
            track_filter[i] = (int32_t)track_data.raw[i] << TRACK_FILTER_SHIFT;
            track_data.filt[i] = track_data.raw[i];
        }
        track_filter_ready = 1U;
    } else {
        for (i = 0U; i < TRACK_NUM; i++) {
            track_data.filt[i] = Tracking_FilterUpdate(i, track_data.raw[i]);
        }
    }
    for (i = 0U; i < TRACK_NUM; i++) {
        track_data.norm[i] = Tracking_NormalizeOne(track_data.filt[i], track_white[i], track_black[i]);
        strength += track_data.norm[i];
        weighted += (int32_t)track_data.norm[i] * (int32_t)track_weight[i];
    }

    if (strength < TRACK_LOST_STRENGTH) {
        track_data.strength = (uint16_t)strength;
        track_data.error = (track_last_error >= 0) ? TRACK_MAX_ERR : -TRACK_MAX_ERR;
        return 1U;
    }

    track_data.strength = (uint16_t)strength;
    track_data.error = (int16_t)(weighted / (int32_t)strength);
    track_last_error = track_data.error;
    return 1U;
}

const Tracking_Data *Tracking_GetData(void)
{
    return &track_data;
}
