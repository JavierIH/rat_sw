#ifndef PWM_H
#define PWM_H

#include <stdint.h>
#include "stm32f1xx_hal.h"

// TIM4 at 72 MHz / 1001 = ~72 kHz, duty 0..PWM_MAX.
#define PWM_RIGHT   TIM_CHANNEL_3   // PB8
#define PWM_LEFT    TIM_CHANNEL_4   // PB9
#define PWM_MAX     1000

void PWM_Init(void);
void pwm_set(uint32_t channel, uint16_t duty);

#endif // PWM_H
