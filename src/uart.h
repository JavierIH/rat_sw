#ifndef UART_H
#define UART_H

#include "stm32f1xx_hal.h"
#include <string.h>

#define UART_BAUDRATE			9600
#define UART_TIMEOUT			1000

UART_HandleTypeDef huart3;
uint8_t uart_send_buffer[100];

void UART_Init();
void send_uart(char* data);

#endif // UART_H
