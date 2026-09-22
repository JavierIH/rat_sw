#ifndef UART_H
#define UART_H

#include "stm32f1xx_hal.h"
#include <string.h>

#define UART_BAUDRATE			9600
#define UART_TIMEOUT			1000

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_tx;

void UART_Init();
void send_uart(char* data);
void print(const char *format, ...);
uint8_t uart_tx_idle(void);

// Non-blocking line-based command receive (for live tuning over Bluetooth).
void uart_start_receive(void);
uint8_t uart_read_line(char *buffer, uint8_t buffer_size);


#endif // UART_H
