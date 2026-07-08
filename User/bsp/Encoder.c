#include "Encoder.h"
#include "ti_msp_dl_config.h"
#include <stdbool.h>

/*
 * MSPM0 移植版编码器：
 * 左轮 B04=PB4=A、B05=PB5=B；右轮 B02=PB2=A、B03=PB3=B。
 * 四个输入脚均开内部上拉，A/B 两相都开双边沿中断，用状态表做正交解码。
 * 如果实测前进方向为负数，只改下面的 ENCODER_SIGN_x，不要改控制层。
 */
#define ENC_L_A_PIN      DL_GPIO_PIN_4
#define ENC_L_B_PIN      DL_GPIO_PIN_5
#define ENC_R_A_PIN      DL_GPIO_PIN_2
#define ENC_R_B_PIN      DL_GPIO_PIN_3
#define ENC_ALL_PINS     (ENC_L_A_PIN | ENC_L_B_PIN | ENC_R_A_PIN | ENC_R_B_PIN)

#define ENCODER_SIGN_L   (+1)
#define ENCODER_SIGN_R   (-1)

static volatile int32_t enc_count_l = 0;
static volatile int32_t enc_count_r = 0;
static uint8_t enc_state_l = 0;
static uint8_t enc_state_r = 0;

static uint8_t Encoder_ReadState(uint32_t pin_a, uint32_t pin_b)
{
    uint32_t pins = DL_GPIO_readPins(GPIOB, pin_a | pin_b);
    uint8_t state = 0U;

    if ((pins & pin_a) != 0U) {
        state |= 0x01U;
    }
    if ((pins & pin_b) != 0U) {
        state |= 0x02U;
    }

    return state;
}

static int8_t Encoder_DecodeStep(uint8_t old_state, uint8_t new_state)
{
    static const int8_t table[16] = {
         0, -1,  1,  0,
         1,  0,  0, -1,
        -1,  0,  0,  1,
         0,  1, -1,  0
    };

    return table[((old_state & 0x03U) << 2) | (new_state & 0x03U)];
}

static void Encoder_ConfigInput(uint32_t pincm)
{
    DL_GPIO_initDigitalInputFeatures(pincm,
        DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE,
        DL_GPIO_WAKEUP_DISABLE);
}

void Encoder_Init(void)
{
    Encoder_ConfigInput(IOMUX_PINCM15);  /* B02/PB2: 右编码器 A */
    Encoder_ConfigInput(IOMUX_PINCM16);  /* B03/PB3: 右编码器 B */
    Encoder_ConfigInput(IOMUX_PINCM17);  /* B04/PB4: 左编码器 A */
    Encoder_ConfigInput(IOMUX_PINCM18);  /* B05/PB5: 左编码器 B */

    enc_count_l = 0;
    enc_count_r = 0;
    enc_state_l = Encoder_ReadState(ENC_L_A_PIN, ENC_L_B_PIN);
    enc_state_r = Encoder_ReadState(ENC_R_A_PIN, ENC_R_B_PIN);

    DL_GPIO_setLowerPinsPolarity(GPIOB,
        DL_GPIO_PIN_2_EDGE_RISE_FALL |
        DL_GPIO_PIN_3_EDGE_RISE_FALL |
        DL_GPIO_PIN_4_EDGE_RISE_FALL |
        DL_GPIO_PIN_5_EDGE_RISE_FALL);
    DL_GPIO_clearInterruptStatus(GPIOB, ENC_ALL_PINS);
    DL_GPIO_enableInterrupt(GPIOB, ENC_ALL_PINS);

    NVIC_ClearPendingIRQ(GPIOB_INT_IRQn);
    NVIC_EnableIRQ(GPIOB_INT_IRQn);
}

int16_t Encoder_Get_L(void)
{
    int32_t temp;

    __disable_irq();
    temp = enc_count_l;
    enc_count_l = 0;
    __enable_irq();

    temp *= ENCODER_SIGN_L;
    if (temp > 32767) return 32767;
    if (temp < -32768) return -32768;
    return (int16_t)temp;
}

int16_t Encoder_Get_R(void)
{
    int32_t temp;

    __disable_irq();
    temp = enc_count_r;
    enc_count_r = 0;
    __enable_irq();

    temp *= ENCODER_SIGN_R;
    if (temp > 32767) return 32767;
    if (temp < -32768) return -32768;
    return (int16_t)temp;
}

int32_t Encoder_GetTotal_L(void)
{
    int32_t temp;

    __disable_irq();
    temp = enc_count_l;
    __enable_irq();
    return temp * ENCODER_SIGN_L;
}

int32_t Encoder_GetTotal_R(void)
{
    int32_t temp;

    __disable_irq();
    temp = enc_count_r;
    __enable_irq();
    return temp * ENCODER_SIGN_R;
}


uint8_t Encoder_GetPinState(void)
{
    uint32_t pins = DL_GPIO_readPins(GPIOB, ENC_ALL_PINS);
    uint8_t state = 0U;

    if ((pins & ENC_L_A_PIN) != 0U) state |= 0x01U;
    if ((pins & ENC_L_B_PIN) != 0U) state |= 0x02U;
    if ((pins & ENC_R_A_PIN) != 0U) state |= 0x04U;
    if ((pins & ENC_R_B_PIN) != 0U) state |= 0x08U;
    return state;
}
void GROUP1_IRQHandler(void)
{
    uint32_t status = DL_GPIO_getEnabledInterruptStatus(GPIOB, ENC_ALL_PINS);

    if ((status & (ENC_L_A_PIN | ENC_L_B_PIN)) != 0U) {
        uint8_t state = Encoder_ReadState(ENC_L_A_PIN, ENC_L_B_PIN);
        enc_count_l += Encoder_DecodeStep(enc_state_l, state);
        enc_state_l = state;
    }

    if ((status & (ENC_R_A_PIN | ENC_R_B_PIN)) != 0U) {
        uint8_t state = Encoder_ReadState(ENC_R_A_PIN, ENC_R_B_PIN);
        enc_count_r += Encoder_DecodeStep(enc_state_r, state);
        enc_state_r = state;
    }

    DL_GPIO_clearInterruptStatus(GPIOB, status);
}
