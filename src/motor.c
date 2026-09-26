#include "motor.h"
#include "stm32f1xx_hal.h"
#include "pwm.h"

#define MOTOR_R_IN1     GPIO_PIN_6      // PB6
#define MOTOR_R_IN2     GPIO_PIN_7      // PB7
#define MOTOR_L_IN1     GPIO_PIN_3      // PB3 (JTAG pin, freed in HAL_MspInit)
#define MOTOR_L_IN2     GPIO_PIN_15     // PA15 (JTAG pin, freed in HAL_MspInit)

void MOTOR_Init(void){
    GPIO_InitTypeDef gpio = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Speed = GPIO_SPEED_FREQ_MEDIUM;
    gpio.Pin = MOTOR_R_IN1 | MOTOR_R_IN2 | MOTOR_L_IN1;
    HAL_GPIO_Init(GPIOB, &gpio);
    gpio.Pin = MOTOR_L_IN2;
    HAL_GPIO_Init(GPIOA, &gpio);
}

static void set_direction(motor_t motor, uint8_t forward){
    GPIO_PinState fwd = forward ? GPIO_PIN_SET : GPIO_PIN_RESET;
    GPIO_PinState rev = forward ? GPIO_PIN_RESET : GPIO_PIN_SET;
    if(motor == MOTOR_R){
        HAL_GPIO_WritePin(GPIOB, MOTOR_R_IN1, rev);
        HAL_GPIO_WritePin(GPIOB, MOTOR_R_IN2, fwd);
    }
    else{   // the left bridge inputs are wired the other way round
        HAL_GPIO_WritePin(GPIOB, MOTOR_L_IN1, fwd);
        HAL_GPIO_WritePin(GPIOA, MOTOR_L_IN2, rev);
    }
}

static volatile int16_t requested[2];
static volatile uint32_t last_drive_ms;     // HAL tick of the last non-zero duty requested

int16_t motor_get(motor_t motor){
    return requested[motor];
}

uint32_t motor_idle_ms(void){
    return HAL_GetTick() - last_drive_ms;
}

void motor_set(motor_t motor, int16_t pwm){
    requested[motor] = pwm;
    if(pwm) last_drive_ms = HAL_GetTick();
#if MOTORS_ENABLED
    int32_t duty = pwm;
    set_direction(motor, duty > 0);
    if(duty < 0) duty = -duty;
    if(duty > PWM_MAX) duty = PWM_MAX;
    pwm_set(motor == MOTOR_L ? PWM_LEFT : PWM_RIGHT, (uint16_t)duty);
#else
    (void)motor;
    (void)pwm;
#endif
}
