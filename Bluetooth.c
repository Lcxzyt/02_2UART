#include "Bluetooth.h"
#include "ti_msp_dl_config.h"
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>

#define BLUETOOTH_UART_ENABLE 1U
#define BLUETOOTH_RING_SIZE 256U
#define BLUETOOTH_PRINTF_BUF_SIZE 256U

static volatile uint8_t bluetooth_ring[BLUETOOTH_RING_SIZE];
static volatile uint16_t bluetooth_ring_head = 0U;
static volatile uint16_t bluetooth_ring_tail = 0U;
static volatile uint32_t bluetooth_rx_count = 0U;
static volatile uint32_t bluetooth_irq_count = 0U;

void Bluetooth_RingBuf_Put(uint8_t ch)
{
    uint16_t next = (uint16_t)(bluetooth_ring_head + 1U);
    if (next >= BLUETOOTH_RING_SIZE) next = 0U;
    if (next == bluetooth_ring_tail) return;

    bluetooth_ring[bluetooth_ring_head] = ch;
    bluetooth_ring_head = next;
}

uint8_t Bluetooth_RingBuf_Get(void)
{
    uint8_t ch = bluetooth_ring[bluetooth_ring_tail];
    uint16_t next = (uint16_t)(bluetooth_ring_tail + 1U);
    if (next >= BLUETOOTH_RING_SIZE) next = 0U;
    bluetooth_ring_tail = next;
    return ch;
}

uint8_t Bluetooth_RingBuf_IsEmpty(void)
{
    return (bluetooth_ring_head == bluetooth_ring_tail) ? 1U : 0U;
}

static void Bluetooth_DrainRx(void)
{
    while (DL_UART_Main_isRXFIFOEmpty(BT_UART_INST) == false) {
        Bluetooth_RingBuf_Put(DL_UART_Main_receiveData(BT_UART_INST));
        bluetooth_rx_count++;
    }
}

void Bluetooth_PollRx(void)
{
#if BLUETOOTH_UART_ENABLE
    NVIC_DisableIRQ(BT_UART_INST_INT_IRQN);
    Bluetooth_DrainRx();
    NVIC_EnableIRQ(BT_UART_INST_INT_IRQN);
#endif
}

uint32_t Bluetooth_GetRxCount(void)
{
    return bluetooth_rx_count;
}

uint32_t Bluetooth_GetIrqCount(void)
{
    return bluetooth_irq_count;
}

void Bluetooth_Init(void)
{
#if BLUETOOTH_UART_ENABLE
    DL_UART_Main_setRXFIFOThreshold(BT_UART_INST, DL_UART_MAIN_RX_FIFO_LEVEL_ONE_ENTRY);
    DL_UART_enableInterrupt(BT_UART_INST, DL_UART_MAIN_INTERRUPT_RX);
    NVIC_ClearPendingIRQ(BT_UART_INST_INT_IRQN);
    NVIC_EnableIRQ(BT_UART_INST_INT_IRQN);
#else
    NVIC_DisableIRQ(BT_UART_INST_INT_IRQN);
    DL_UART_Main_disableInterrupt(BT_UART_INST, DL_UART_MAIN_INTERRUPT_RX);
    DL_UART_Main_disable(BT_UART_INST);
    DL_GPIO_initDigitalInputFeatures(GPIO_BT_UART_IOMUX_TX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(GPIO_BT_UART_IOMUX_RX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
#endif
}

void Bluetooth_SendByte(uint8_t byte)
{
#if BLUETOOTH_UART_ENABLE
    DL_UART_Main_transmitDataBlocking(BT_UART_INST, byte);
#else
    (void)byte;
#endif
}

void Bluetooth_SendString(char *string)
{
    while (*string != '\0') {
        Bluetooth_SendByte((uint8_t)*string);
        string++;
    }
}

void Bluetooth_Printf(char *format, ...)
{
    char string[BLUETOOTH_PRINTF_BUF_SIZE];
    va_list arg;

    va_start(arg, format);
    (void)vsnprintf(string, sizeof(string), format, arg);
    va_end(arg);

    Bluetooth_SendString(string);
}

void BT_UART_INST_IRQHandler(void)
{
#if BLUETOOTH_UART_ENABLE
    bluetooth_irq_count++;

    switch (DL_UART_Main_getPendingInterrupt(BT_UART_INST)) {
        case DL_UART_MAIN_IIDX_RX:
        case DL_UART_MAIN_IIDX_RX_TIMEOUT_ERROR:
            Bluetooth_DrainRx();
            break;
        default:
            break;
    }
#else
    NVIC_ClearPendingIRQ(BT_UART_INST_INT_IRQN);
#endif
}

