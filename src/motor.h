#ifndef MOTOR_H
#define MOTOR_H

#include "stm32f1xx_hal.h"
#include "pwm.h"
#include "encoder.h"

typedef enum {MOTOR_R, MOTOR_L} motor_t;
typedef enum {FORWARD, BACKWARD, BRAKE, FREE} motor_sense_t;

// SAFETY KILL-SWITCH: 0 = set_output() never drives the H-bridge/PWM (board powered only via
// ST-Link/USB, no battery). Set to 1 only once the robot is running from its own battery.
#define MOTORS_ENABLED 1

#define MOTOR_R_IN1             GPIO_PIN_6
#define MOTOR_R_IN2             GPIO_PIN_7
#define MOTOR_L_IN1             GPIO_PIN_3
#define MOTOR_L_IN2             GPIO_PIN_15

#define MOTOR_R_IN1_PORT        GPIOB
#define MOTOR_R_IN2_PORT        GPIOB
#define MOTOR_L_IN1_PORT        GPIOB
#define MOTOR_L_IN2_PORT        GPIOA

extern int32_t _motor_speed_l;
extern int32_t _motor_speed_r;
extern motor_sense_t _motor_sense_l;
extern motor_sense_t _motor_sense_r;

void MOTOR_Init();
void set_sense(motor_t motor, motor_sense_t sense);
void set_output(motor_t motor, int16_t speed);
void update_speed(motor_t motor);
int32_t get_output(motor_t motor);
//int32_t get_speed(motor_t motor);



#endif // MOTOR_H
