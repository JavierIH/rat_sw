#include "motor.h"

void MOTOR_Init(){
    GPIO_InitTypeDef GPIO_InitStruct;

    // GPIO Ports Clock Enable
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    // Configure GPIO pin : PB6 PB7 PB3
    GPIO_InitStruct.Pin = MOTOR_R_IN1|MOTOR_R_IN2|MOTOR_L_IN1;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_MEDIUM;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // Configure GPIO pins : PA15
    GPIO_InitStruct.Pin = MOTOR_L_IN2;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_MEDIUM;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    _motor_sense_l = FREE;
    _motor_sense_r = FREE;
}

void set_sense(motor_t motor, motor_sense_t sense){
    if (motor == MOTOR_R){
        if(sense == FORWARD){
            HAL_GPIO_WritePin(GPIOB, MOTOR_R_IN1, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, MOTOR_R_IN2, GPIO_PIN_SET);
        }
        else if (sense == BACKWARD){
            HAL_GPIO_WritePin(GPIOB, MOTOR_R_IN1, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, MOTOR_R_IN2, GPIO_PIN_RESET);
        }
        else if (sense == BRAKE){
            HAL_GPIO_WritePin(GPIOB, MOTOR_R_IN1, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, MOTOR_R_IN2, GPIO_PIN_SET);
        }
        else if (sense == FREE){
            HAL_GPIO_WritePin(GPIOB, MOTOR_R_IN1, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, MOTOR_R_IN2, GPIO_PIN_RESET);
        }
    }
    else if (motor == MOTOR_L){
        if(sense == FORWARD){
            HAL_GPIO_WritePin(GPIOB, MOTOR_L_IN1, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOA, MOTOR_L_IN2, GPIO_PIN_RESET);
        }
        else if (sense == BACKWARD){
            HAL_GPIO_WritePin(GPIOB, MOTOR_L_IN1, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOA, MOTOR_L_IN2, GPIO_PIN_SET);
        }
        else if (sense == BRAKE){
            HAL_GPIO_WritePin(GPIOB, MOTOR_L_IN1, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOA, MOTOR_L_IN2, GPIO_PIN_SET);
        }
        else if (sense == FREE){
            HAL_GPIO_WritePin(GPIOB, MOTOR_L_IN1, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOA, MOTOR_L_IN2, GPIO_PIN_RESET);
        }
    }
}

void update_speed(motor_t motor){
    if (motor == MOTOR_R){
        _motor_speed_r = get_encoder_delta(ENCODER_R); // /sample time 1ms
    }
    else if (motor == MOTOR_L){
        _motor_speed_l = get_encoder_delta(ENCODER_L);
    }
}


int32_t get_speed(motor_t motor){
    if (motor == MOTOR_R){
        return _motor_speed_r;
    }
    else if (motor == MOTOR_L){
        return _motor_speed_l;
    }
}
