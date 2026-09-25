#ifndef UART_H
#define UART_H

#include <stdint.h>

// Bluetooth console: USART3 + HC-05 at 9600 baud. Non-blocking both ways:
// TX is a DMA-driven queue of whole messages, RX assembles lines in the
// interrupt. No HAL types here, so strategy code can log on the host too.

#define UART_BAUDRATE 9600

void UART_Init(void);
void uart_start_receive(void);
// After the system clock changed (sysclock.c): the baud rate from the new bus clock.
void uart_retime(void);

// Queue a message. It is silently DROPPED if the queue is full: logging must
// never stall a control loop.
void uart_send(const char *text);
void print(const char *format, ...) __attribute__((format(printf, 1, 2)));

uint8_t uart_tx_idle(void);
uint8_t uart_tx_full(void);
void uart_flush(uint32_t timeout_ms);       // wait until everything queued was sent
void uart_wait_space(uint32_t timeout_ms);  // wait for a free slot (bulk output while stopped)

// Pops one received line (without the line ending). 0 if none pending.
uint8_t uart_read_line(char *buffer, uint8_t buffer_size);

#endif // UART_H
