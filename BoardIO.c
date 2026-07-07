#include "BoardIO.h"
#include "ti_msp_dl_config.h"

#define BOARD_IO_DEBOUNCE_TICKS 3U

#define BOARD_MENU_IOMUX   IOMUX_PINCM49
#define BOARD_MENU_PORT    GPIOB
#define BOARD_MENU_PIN     DL_GPIO_PIN_21

#define BOARD_FUNC_IOMUX   IOMUX_PINCM33
#define BOARD_FUNC_PORT    GPIOB
#define BOARD_FUNC_PIN     DL_GPIO_PIN_16

#define BOARD_BUZZER_IOMUX IOMUX_PINCM51
#define BOARD_BUZZER_PORT  GPIOB
#define BOARD_BUZZER_PIN   DL_GPIO_PIN_23

#define BOARD_LED_IOMUX    IOMUX_PINCM50
#define BOARD_LED_PORT     GPIOB
#define BOARD_LED_PIN      DL_GPIO_PIN_22

typedef struct {
    uint8_t stable_pressed;
    uint8_t last_raw_pressed;
    uint8_t stable_count;
    uint8_t pressed_event;
} BoardIO_Key;

static BoardIO_Key menu_key;
static BoardIO_Key func_key;
static uint8_t led_on;
static uint8_t buzzer_on;

static uint8_t BoardIO_ReadKeyRaw(GPIO_Regs *port, uint32_t pin)
{
    return (DL_GPIO_readPins(port, pin) == 0U) ? 1U : 0U;
}

static void BoardIO_UpdateKey(BoardIO_Key *key, uint8_t raw_pressed)
{
    if (raw_pressed == key->last_raw_pressed) {
        if (key->stable_count < BOARD_IO_DEBOUNCE_TICKS) {
            key->stable_count++;
        }
    } else {
        key->last_raw_pressed = raw_pressed;
        key->stable_count = 0U;
    }

    if (key->stable_count >= BOARD_IO_DEBOUNCE_TICKS) {
        if (raw_pressed != key->stable_pressed) {
            key->stable_pressed = raw_pressed;
            if (raw_pressed) {
                key->pressed_event = 1U;
            }
        }
    }
}

static uint8_t BoardIO_TakePressedEvent(BoardIO_Key *key)
{
    uint8_t event = key->pressed_event;
    key->pressed_event = 0U;
    return event;
}

void BoardIO_Init(void)
{
    DL_GPIO_initDigitalInputFeatures(BOARD_MENU_IOMUX,
        DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(BOARD_FUNC_IOMUX,
        DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE,
        DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalOutput(BOARD_BUZZER_IOMUX);
    DL_GPIO_initDigitalOutput(BOARD_LED_IOMUX);

    /* Active-low buzzer: keep high when idle. LED is active-high: keep low when idle. */
    DL_GPIO_setPins(BOARD_BUZZER_PORT, BOARD_BUZZER_PIN);
    DL_GPIO_clearPins(BOARD_LED_PORT, BOARD_LED_PIN);
    DL_GPIO_enableOutput(BOARD_BUZZER_PORT, BOARD_BUZZER_PIN);
    DL_GPIO_enableOutput(BOARD_LED_PORT, BOARD_LED_PIN);

    menu_key.stable_pressed = 0U;
    menu_key.last_raw_pressed = BoardIO_ReadKeyRaw(BOARD_MENU_PORT, BOARD_MENU_PIN);
    menu_key.stable_count = 0U;
    menu_key.pressed_event = 0U;

    func_key.stable_pressed = 0U;
    func_key.last_raw_pressed = BoardIO_ReadKeyRaw(BOARD_FUNC_PORT, BOARD_FUNC_PIN);
    func_key.stable_count = 0U;
    func_key.pressed_event = 0U;

    led_on = 0U;
    buzzer_on = 0U;
}

void BoardIO_Update20ms(void)
{
    BoardIO_UpdateKey(&menu_key, BoardIO_ReadKeyRaw(BOARD_MENU_PORT, BOARD_MENU_PIN));
    BoardIO_UpdateKey(&func_key, BoardIO_ReadKeyRaw(BOARD_FUNC_PORT, BOARD_FUNC_PIN));
}

uint8_t BoardIO_MenuPressed(void)
{
    return BoardIO_TakePressedEvent(&menu_key);
}

uint8_t BoardIO_FuncPressed(void)
{
    return BoardIO_TakePressedEvent(&func_key);
}

uint8_t BoardIO_GetMenuRawPressed(void)
{
    return BoardIO_ReadKeyRaw(BOARD_MENU_PORT, BOARD_MENU_PIN);
}

uint8_t BoardIO_GetFuncRawPressed(void)
{
    return BoardIO_ReadKeyRaw(BOARD_FUNC_PORT, BOARD_FUNC_PIN);
}

void BoardIO_LedSet(uint8_t on)
{
    led_on = on ? 1U : 0U;
    if (led_on) {
        DL_GPIO_setPins(BOARD_LED_PORT, BOARD_LED_PIN);
    } else {
        DL_GPIO_clearPins(BOARD_LED_PORT, BOARD_LED_PIN);
    }
}

void BoardIO_LedToggle(void)
{
    BoardIO_LedSet((uint8_t)!led_on);
}

uint8_t BoardIO_LedIsOn(void)
{
    return led_on;
}

void BoardIO_BuzzerSet(uint8_t on)
{
    buzzer_on = on ? 1U : 0U;
    if (buzzer_on) {
        DL_GPIO_clearPins(BOARD_BUZZER_PORT, BOARD_BUZZER_PIN);
    } else {
        DL_GPIO_setPins(BOARD_BUZZER_PORT, BOARD_BUZZER_PIN);
    }
}

void BoardIO_BuzzerToggle(void)
{
    BoardIO_BuzzerSet((uint8_t)!buzzer_on);
}

uint8_t BoardIO_BuzzerIsOn(void)
{
    return buzzer_on;
}
