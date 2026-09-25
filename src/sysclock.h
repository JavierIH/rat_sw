#ifndef SYSCLOCK_H
#define SYSCLOCK_H

#include "stm32f1xx_hal.h"
#include "error.h"

typedef enum {
    CLOCK_HSE,          // crystal x 9 = 72 MHz
    CLOCK_HSI_BOOT,     // the crystal did not start: internal oscillator, 64 MHz
    CLOCK_HSI_FAILED,   // the crystal failed while running (clock security system)
    CLOCK_HSI_FORCED,   // CLOCK HSI command
} clock_source_t;

void SystemClock_Config(void);
clock_source_t sysclock_source(void);
// Main context: after a crystal failure, brings the clock back up at 64 MHz
// on the internal oscillator. 1 once, when that happened (then the UART baud
// rate needs uart_retime()).
uint8_t sysclock_recover(void);
// Switches to the internal oscillator as after a failure (tests the recovery).
// 1 if done; then uart_retime().
uint8_t sysclock_use_hsi(void);

#endif // SYSCLOCK_H
