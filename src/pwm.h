#ifndef PWM_H
#define PWM_H

#include "stm32f1xx_hal.h"
#include "msp.h"
#include "uart.h"
#include "error.h"

TIM_HandleTypeDef htim4;

void PWM_Init();
void set_pwm(uint16_t pwm_pin, uint32_t value);

#endif // PWM_H
