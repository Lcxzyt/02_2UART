#include "uart.h"
#include "ti_msp_dl_config.h"
#include <stdarg.h>
#include <stdio.h>

#define SERIAL_RING_SIZE 256U
#define SERIAL_PRINTF_BUF_SIZE 256U

static volatile uint8_t serial_ring[SERIAL_RING_SIZE];
static volatile uint16_t serial_ring_head = 0U;
static volatile uint16_t serial_ring_tail = 0U;

void Serial_RingBuf_Put(uint8_t ch)
{
    uint16_t next = (uint16_t) (serial_ring_head + 1U);
    if (next >= SERIAL_RING_SIZE) next = 0U;
    if (next == serial_ring_tail) return;

    serial_ring[serial_ring_head] = ch;
    serial_ring_head = next;
}

uint8_t Serial_RingBuf_Get(void)
{
    uint8_t ch = serial_ring[serial_ring_tail];
    uint16_t next = (uint16_t) (serial_ring_tail + 1U);
    if (next >= SERIAL_RING_SIZE) next = 0U;
    serial_ring_tail = next;
    return ch;
}

uint8_t Serial_RingBuf_IsEmpty(void)
{
    return (serial_ring_head == serial_ring_tail) ? 1U : 0U;
}

void Serial_Init(void)
{
    DL_UART_enableInterrupt(UART_0_INST, DL_UART_MAIN_INTERRUPT_RX);
    NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
}

void Serial_SendByte(uint8_t Byte)
{
    DL_UART_Main_transmitDataBlocking(UART_0_INST, Byte);
}

void Serial_SendArray(uint8_t *Array, uint16_t Length)
{
    uint16_t i;
    for (i = 0U; i < Length; i++) {
        Serial_SendByte(Array[i]);
    }
}

void Serial_SendString(char *String)
{
    while (*String != '\0') {
        Serial_SendByte((uint8_t) *String);
        String++;
    }
}

uint32_t Serial_Pow(uint32_t X, uint32_t Y)
{
    uint32_t Result = 1U;
    while (Y-- > 0U) {
        Result *= X;
    }
    return Result;
}

void Serial_SendNumber(uint32_t Number, uint8_t Length)
{
    uint8_t i;
    for (i = 0U; i < Length; i++) {
        Serial_SendByte((uint8_t) (Number / Serial_Pow(10U, Length - i - 1U) % 10U + '0'));
    }
}

int fputc(int ch, FILE *f)
{
    (void) f;
    Serial_SendByte((uint8_t) ch);
    return ch;
}

void Serial_Printf(char *format, ...)
{
    char String[SERIAL_PRINTF_BUF_SIZE];
    va_list arg;

    va_start(arg, format);
    (void) vsnprintf(String, sizeof(String), format, arg);
    va_end(arg);

    Serial_SendString(String);
}

void Serial_PrintFloat(float val, uint8_t intDig, uint8_t fracDig)
{
    uint8_t i, neg = 0U;
    uint32_t factor = 1U;
    uint32_t absInt, absFrac;
    uint8_t intLen;

    if (val < 0.0f) {
        neg = 1U;
        val = -val;
    }

    for (i = 0U; i < fracDig; i++) factor *= 10U;

    {
        int32_t scaled = (int32_t) (val * (float) factor + 0.5f);
        absInt = (uint32_t) (scaled / (int32_t) factor);
        absFrac = (uint32_t) (scaled % (int32_t) factor);
    }

    intLen = 1U;
    {
        uint32_t t = absInt;
        while (t >= 10U) {
            intLen++;
            t /= 10U;
        }
    }
    if (neg) intLen++;

    for (i = intLen; i < intDig; i++) Serial_SendByte(' ');
    if (neg) Serial_SendByte('-');
    Serial_SendNumber(absInt, (uint8_t) (intLen - (neg ? 1U : 0U)));

    if (fracDig > 0U) {
        Serial_SendByte('.');
        Serial_SendNumber(absFrac, fracDig);
    }
}

void send_char(UART_Regs *uart, uint8_t data)
{
    DL_UART_Main_transmitDataBlocking(uart, data);
}

void send_str(UART_Regs *uart, uint8_t *data)
{
    while ((data != 0) && (*data != 0U)) {
        send_char(uart, *data);
        data++;
    }
}

void UART_0_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(UART_0_INST)) {
        case DL_UART_MAIN_IIDX_RX:
            while (DL_UART_Main_isRXFIFOEmpty(UART_0_INST) == false) {
                Serial_RingBuf_Put(DL_UART_Main_receiveData(UART_0_INST));
            }
            break;
        default:
            break;
    }
}
