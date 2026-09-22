#ifndef MSP_H
#define MSP_H

#include "stm32f1xx_hal.h"

// HAL MSP hooks: per-peripheral pins, clocks, DMA and interrupts (msp.c).
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

#endif // MSP_H
