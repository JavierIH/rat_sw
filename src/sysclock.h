#ifndef SYSCLOCK_H
#define SYSCLOCK_H

#include "stm32f1xx_hal.h"
#include "error.h"

typedef enum {
    CLOCK_HSE,          // crystal x 9 = 72 MHz
    CLOCK_HSI_BOOT,     // the crystal did not start: internal oscillator, 64 MHz
    CLOCK_HSI_FAILED,   // the crystal failed while running (clock security system)
} clock_source_t;

void SystemClock_Config(void);
clock_source_t sysclock_source(void);
// Main context: after a crystal failure, back up at 64 MHz on the HSI; 1 once (then uart_retime()).
uint8_t sysclock_recover(void);

#endif // SYSCLOCK_H
