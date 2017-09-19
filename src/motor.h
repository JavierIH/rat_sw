#ifndef MOTOR_H
#define MOTOR_H

#include "stm32f1xx_hal.h"
#include "pwm.h"
#include "encoder.h"

typedef enum {MOTOR_R, MOTOR_L} motor_t;
typedef enum {FORWARD, BACKWARD, BRAKE, FREE} motor_sense_t;

#define MOTOR_R_IN1             GPIO_PIN_6
#define MOTOR_R_IN2             GPIO_PIN_7
#define MOTOR_L_IN1             GPIO_PIN_3
#define MOTOR_L_IN2             GPIO_PIN_15

#define MOTOR_R_IN1_PORT        GPIOB
#define MOTOR_R_IN2_PORT        GPIOB
#define MOTOR_L_IN1_PORT        GPIOB
#define MOTOR_L_IN2_PORT        GPIOA

int32_t _motor_speed_l;
int32_t _motor_speed_r;
motor_sense_t _motor_sense_l;
motor_sense_t _motor_sense_r;

void MOTOR_Init();
void set_sense(motor_t motor, motor_sense_t sense);
void set_speed(motor_t motor, motor_sense_t speed);
void update_speed(motor_t motor);
int32_t get_speed(motor_t motor);



#endif // MOTOR_H
