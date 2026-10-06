#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>
#include "stm32f1xx.h"

// SAFETY KILL-SWITCH: 0 = the H-bridge is never driven (bench testing on USB power); 1 = normal.
#define MOTORS_ENABLED 1

typedef enum { MOTOR_R, MOTOR_L } motor_t;

void MOTOR_Init(void);
// Signed duty -1000..1000, positive forward; 0 short-brakes (PWM low with one bridge input high).
void motor_set(motor_t motor, int16_t pwm);
int16_t motor_get(motor_t motor);   // last duty requested (recorded by CAL tests)

// Register-level stop for fault handlers: zero duty and every bridge input low.
static inline void motor_emergency_stop(void){
    TIM4->CCR3 = 0;
    TIM4->CCR4 = 0;
    GPIOB->BRR = (1u << 3) | (1u << 6) | (1u << 7);    // L_IN1, R_IN1, R_IN2
    GPIOA->BRR = (1u << 15);                            // L_IN2
}

#endif // MOTOR_H
