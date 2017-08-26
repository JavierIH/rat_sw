#ifndef MOTOR_H
#define MOTOR_H

#include "stm32f1xx_hal.h"
#include "pwm.h"

#define MOTOR_R                 0
#define MOTOR_L                 1

#define MOTOR_R_IN1             GPIO_PIN_6
#define MOTOR_R_IN2             GPIO_PIN_7
#define MOTOR_L_IN1             GPIO_PIN_3
#define MOTOR_L_IN2             GPIO_PIN_15

#define MOTOR_R_IN1_PORT        GPIOB
#define MOTOR_R_IN2_PORT        GPIOB
#define MOTOR_L_IN1_PORT        GPIOB
#define MOTOR_L_IN2_PORT        GPIOA

void MOTOR_Init();
void set_sense(uint8_t motor, uint8_t sense);
void set_speed(uint8_t motor, int16_t speed);


#endif // MOTOR_H
