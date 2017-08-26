#include "motor.h"

void MOTOR_Init(){
    GPIO_InitTypeDef GPIO_InitStruct;

    // GPIO Ports Clock Enable
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    // Configure GPIO pin : PB6 PB7 PB3
    GPIO_InitStruct.Pin = MOTOR_R_IN1|MOTOR_R_IN2|MOTOR_L_IN1;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    //GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_MEDIUM;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // Configure GPIO pins : PA15
    GPIO_InitStruct.Pin = MOTOR_L_IN2;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    //GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_MEDIUM;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void set_sense(uint8_t motor, uint8_t sense){
    if (motor == MOTOR_R){
        if(sense){
            HAL_GPIO_WritePin(GPIOB, MOTOR_R_IN1, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, MOTOR_R_IN2, GPIO_PIN_RESET);
        }
        else{
            HAL_GPIO_WritePin(GPIOB, MOTOR_R_IN1, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, MOTOR_R_IN2, GPIO_PIN_SET);
        }
    }
    else if (motor == MOTOR_L){
        if(sense){
            HAL_GPIO_WritePin(GPIOB, MOTOR_L_IN1, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOA, MOTOR_L_IN2, GPIO_PIN_SET);
        }
        else{
            HAL_GPIO_WritePin(GPIOB, MOTOR_L_IN1, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOA, MOTOR_L_IN2, GPIO_PIN_RESET);
        }
    }
}
