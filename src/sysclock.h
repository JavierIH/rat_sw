#ifndef SYSCLOCK_H
#define SYSCLOCK_H

#include "stm32f1xx_hal.h"
#include "error.h"


void SystemClock_Config(void);
void SysTick_Handler(void);

#endif // SYSCLOCK_H
