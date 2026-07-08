#include "Tracking.h"
#include "ti_msp_dl_config.h"

#define TRACK_SETTLE_CYCLES 320U
#define TRACK_MAX_ERR 3500
#define TRACK_ADC_AVG_SAMPLES 8U
#define TRACK_ADC_TIMEOUT_LOOPS 3000U
#define TRACK_ADC_MAX 4095U
#define TRACK_NORM_MAX 1000U
#define TRACK_ADDR_INVERT 1U
#define TRACK_GANV_DIRECTION_REVERSE 1U

#define TRACK_OUT_ADC_IOMUX IOMUX_PINCM38
#define TRACK_ADC_INST ADC1
#define TRACK_ADC_MEM  DL_ADC12_MEM_IDX_0
#define TRACK_ADC_CHAN DL_ADC12_INPUT_CHAN_1

#define TRACK_AD0_IOMUX IOMUX_PINCM39
#define TRACK_AD0_PORT  GPIOA
#define TRACK_AD0_PIN   DL_GPIO_PIN_17

#define TRACK_AD1_IOMUX IOMUX_PINCM43
#define TRACK_AD1_PORT  GPIOB
#define TRACK_AD1_PIN   DL_GPIO_PIN_17

#define TRACK_AD2_IOMUX IOMUX_PINCM44
#define TRACK_AD2_PORT  GPIOB
#define TRACK_AD2_PIN   DL_GPIO_PIN_18

/* Measured sensor order: bits 0F means the line is on the physical left side, E0 on the right. */
static const int16_t track_weight[TRACK_NUM] = {
    -3500, -2500, -1500, -500, 500, 1500, 2500, 3500
};

/* Ganv calibration measured on current track: white floor raw is high, black line raw is low. */
static const uint16_t track_cal_white_default[TRACK_NUM] = {
    3017U, 3198U, 3158U, 3040U, 3194U, 3109U, 2916U, 2660U
};
static const uint16_t track_cal_black_default[TRACK_NUM] = {
    154U, 161U, 159U, 160U, 189U, 162U, 162U, 163U
};

static uint16_t track_cal_white[TRACK_NUM];
static uint16_t track_cal_black[TRACK_NUM];
static Tracking_Data track_data;
static int16_t track_last_error;
static uint8_t track_inited;

static uint8_t Tracking_ChannelEnabled(uint8_t channel)
{
    return (TRACK_DISABLED_MASK & (uint8_t)(1U << channel)) ? 0U : 1U;
}

static void Tracking_SetAddressPin(GPIO_Regs *port, uint32_t pin, uint8_t high)
{
    if (high) {
        DL_GPIO_setPins(port, pin);
    } else {
        DL_GPIO_clearPins(port, pin);
    }
}

static void Tracking_SetMuxChannel(uint8_t channel)
{
    uint8_t ad0 = (channel & 0x01U) ? 1U : 0U;
    uint8_t ad1 = (channel & 0x02U) ? 1U : 0U;
    uint8_t ad2 = (channel & 0x04U) ? 1U : 0U;

#if TRACK_ADDR_INVERT
    ad0 = (uint8_t)(!ad0);
    ad1 = (uint8_t)(!ad1);
    ad2 = (uint8_t)(!ad2);
#endif

    Tracking_SetAddressPin(TRACK_AD0_PORT, TRACK_AD0_PIN, ad0);
    Tracking_SetAddressPin(TRACK_AD1_PORT, TRACK_AD1_PIN, ad1);
    Tracking_SetAddressPin(TRACK_AD2_PORT, TRACK_AD2_PIN, ad2);

    delay_cycles(TRACK_SETTLE_CYCLES);
}

static void Tracking_AdcInit(void)
{
    static const DL_ADC12_ClockConfig adc_clock_config = {
        .clockSel    = DL_ADC12_CLOCK_ULPCLK,
        .divideRatio = DL_ADC12_CLOCK_DIVIDE_8,
        .freqRange   = DL_ADC12_CLOCK_FREQ_RANGE_24_TO_32,
    };

    DL_ADC12_reset(TRACK_ADC_INST);
    DL_ADC12_enablePower(TRACK_ADC_INST);
    delay_cycles(POWER_STARTUP_DELAY);

    DL_GPIO_initPeripheralAnalogFunction(TRACK_OUT_ADC_IOMUX);

    DL_ADC12_setClockConfig(TRACK_ADC_INST, (DL_ADC12_ClockConfig *) &adc_clock_config);
    DL_ADC12_initSingleSample(TRACK_ADC_INST,
                              DL_ADC12_REPEAT_MODE_DISABLED,
                              DL_ADC12_SAMPLING_SOURCE_AUTO,
                              DL_ADC12_TRIG_SRC_SOFTWARE,
                              DL_ADC12_SAMP_CONV_RES_12_BIT,
                              DL_ADC12_SAMP_CONV_DATA_FORMAT_UNSIGNED);
    DL_ADC12_configConversionMem(TRACK_ADC_INST,
                                 TRACK_ADC_MEM,
                                 TRACK_ADC_CHAN,
                                 DL_ADC12_REFERENCE_VOLTAGE_VDDA,
                                 DL_ADC12_SAMPLE_TIMER_SOURCE_SCOMP0,
                                 DL_ADC12_AVERAGING_MODE_DISABLED,
                                 DL_ADC12_BURN_OUT_SOURCE_DISABLED,
                                 DL_ADC12_TRIGGER_MODE_AUTO_NEXT,
                                 DL_ADC12_WINDOWS_COMP_MODE_DISABLED);
    DL_ADC12_setPowerDownMode(TRACK_ADC_INST, DL_ADC12_POWER_DOWN_MODE_MANUAL);
    DL_ADC12_setSampleTime0(TRACK_ADC_INST, 64U);
    DL_ADC12_clearInterruptStatus(TRACK_ADC_INST, DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED);
    DL_ADC12_enableConversions(TRACK_ADC_INST);
}

static uint8_t Tracking_AdcReadOnce(uint16_t *value)
{
    uint32_t timeout = TRACK_ADC_TIMEOUT_LOOPS;

    if (value == 0) return 0U;

    DL_ADC12_enableConversions(TRACK_ADC_INST);
    DL_ADC12_clearInterruptStatus(TRACK_ADC_INST, DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED);
    DL_ADC12_startConversion(TRACK_ADC_INST);

    while ((DL_ADC12_getRawInterruptStatus(TRACK_ADC_INST, DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED) == 0U) &&
           (timeout > 0U)) {
        timeout--;
    }

    if (timeout == 0U) {
        DL_ADC12_stopConversion(TRACK_ADC_INST);
        DL_ADC12_enableConversions(TRACK_ADC_INST);
        return 0U;
    }

    *value = DL_ADC12_getMemResult(TRACK_ADC_INST, TRACK_ADC_MEM);
    DL_ADC12_clearInterruptStatus(TRACK_ADC_INST, DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED);
    DL_ADC12_enableConversions(TRACK_ADC_INST);
    return 1U;
}

static uint8_t Tracking_AdcReadAverage(uint16_t *value)
{
    uint8_t i;
    uint32_t sum = 0U;
    uint16_t sample;

    if (value == 0) return 0U;

    for (i = 0U; i < TRACK_ADC_AVG_SAMPLES; i++) {
        if (!Tracking_AdcReadOnce(&sample)) {
            return 0U;
        }
        sum += sample;
    }

    *value = (uint16_t)((sum + (TRACK_ADC_AVG_SAMPLES / 2U)) / TRACK_ADC_AVG_SAMPLES);
    return 1U;
}

static uint16_t Tracking_NormalizeBlackStrength(uint8_t channel, uint16_t adc)
{
    uint16_t white = track_cal_white[channel];
    uint16_t black = track_cal_black[channel];
    uint16_t temp;
    uint32_t value;

    if (black > white) {
        temp = white;
        white = black;
        black = temp;
    }

    if (white <= black) return 0U;
    if (adc >= white) return 0U;
    if (adc <= black) return TRACK_NORM_MAX;

    value = ((uint32_t)(white - adc) * TRACK_NORM_MAX) / (uint32_t)(white - black);
    if (value > TRACK_NORM_MAX) value = TRACK_NORM_MAX;
    return (uint16_t)value;
}

void Tracking_SetCalib(const uint16_t white[TRACK_NUM], const uint16_t black[TRACK_NUM])
{
    uint8_t i;

    if ((white == 0) || (black == 0)) return;

    for (i = 0U; i < TRACK_NUM; i++) {
        track_cal_white[i] = white[i];
        track_cal_black[i] = black[i];
    }
}

void Tracking_Init(void)
{
    uint8_t i;

    for (i = 0U; i < TRACK_NUM; i++) {
        track_cal_white[i] = track_cal_white_default[i];
        track_cal_black[i] = track_cal_black_default[i];
    }

    Tracking_AdcInit();

    DL_GPIO_initDigitalOutput(TRACK_AD0_IOMUX);
    DL_GPIO_initDigitalOutput(TRACK_AD1_IOMUX);
    DL_GPIO_initDigitalOutput(TRACK_AD2_IOMUX);

    DL_GPIO_clearPins(TRACK_AD0_PORT, TRACK_AD0_PIN);
    DL_GPIO_clearPins(TRACK_AD1_PORT, TRACK_AD1_PIN);
    DL_GPIO_clearPins(TRACK_AD2_PORT, TRACK_AD2_PIN);

    DL_GPIO_enableOutput(TRACK_AD0_PORT, TRACK_AD0_PIN);
    DL_GPIO_enableOutput(TRACK_AD1_PORT, TRACK_AD1_PIN);
    DL_GPIO_enableOutput(TRACK_AD2_PORT, TRACK_AD2_PIN);

    track_last_error = 0;
    track_data.valid = 0U;
    track_inited = 1U;
}

uint8_t Tracking_ReadRaw(uint16_t raw[TRACK_NUM])
{
    uint8_t i;
    uint8_t out_index;

    if ((raw == 0) || !track_inited) return 0U;

    for (i = 0U; i < TRACK_NUM; i++) {
        Tracking_SetMuxChannel(i);
#if TRACK_GANV_DIRECTION_REVERSE
        out_index = (uint8_t)(TRACK_NUM - 1U - i);
#else
        out_index = i;
#endif
        if (!Tracking_AdcReadAverage(&raw[out_index])) {
            return 0U;
        }
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

    for (i = 0U; i < TRACK_NUM; i++) {
        uint16_t value = Tracking_NormalizeBlackStrength(i, track_data.raw[i]);

        if (!Tracking_ChannelEnabled(i)) {
            value = 0U;
        }

        track_data.filt[i] = value;
        track_data.norm[i] = value;
        strength += value;
        weighted += (int32_t)value * (int32_t)track_weight[i];
    }

    track_data.strength = (strength > 65535U) ? 65535U : (uint16_t)strength;
    if (strength == 0U) {
        track_data.error = (track_last_error >= 0) ? TRACK_MAX_ERR : -TRACK_MAX_ERR;
        return 1U;
    }

    track_data.error = (int16_t)(weighted / (int32_t)strength);
    track_last_error = track_data.error;
    return 1U;
}

const Tracking_Data *Tracking_GetData(void)
{
    return &track_data;
}





