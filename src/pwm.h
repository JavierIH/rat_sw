#ifndef PWM_H
#define PWM_H

#define PWM_1                   TIM_CHANNEL_3
#define PWM_2                   TIM_CHANNEL_4

#include "stm32f1xx_hal.h"
#include "msp.h"
#include "uart.h"
#include "error.h"

TIM_HandleTypeDef htim4;

void PWM_Init();
void set_pwm(uint8_t pwm_channel, uint16_t duty_cycle);

#endif // PWM_H
