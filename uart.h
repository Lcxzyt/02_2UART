#ifndef UART_H_
#define UART_H_

#include "ti_msp_dl_config.h"
#include "Serial.h"

void send_char(UART_Regs *uart, uint8_t data);
void send_str(UART_Regs *uart, uint8_t *data);

#endif
