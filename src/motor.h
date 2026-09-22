#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>
#include "stm32f1xx.h"

// SAFETY KILL-SWITCH: 0 = motor_set() never drives the H-bridge (bench
// testing on ST-Link/USB power only). 1 = normal operation on battery.
#define MOTORS_ENABLED 1

typedef enum { MOTOR_R, MOTOR_L } motor_t;

void MOTOR_Init(void);
// Signed duty, -1000..1000 (clamped). Positive drives the robot forward;
// 0 leaves the bridge in reverse at zero duty, as it always has.
void motor_set(motor_t motor, int16_t pwm);

// Register-level stop for fault handlers, where HAL state cannot be trusted:
// zero duty on both channels and all bridge inputs low. Harmless before the
// peripherals are clocked (writes are then ignored).
static inline void motor_emergency_stop(void){
    TIM4->CCR3 = 0;
    TIM4->CCR4 = 0;
    GPIOB->BRR = (1u << 3) | (1u << 6) | (1u << 7);    // L_IN1, R_IN1, R_IN2
    GPIOA->BRR = (1u << 15);                            // L_IN2
}

#endif // MOTOR_H
