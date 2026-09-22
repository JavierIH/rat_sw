#include "pwm.h"
#include "error.h"
#include "msp.h"

static TIM_HandleTypeDef htim4;

void PWM_Init(void){
    TIM_OC_InitTypeDef oc = {0};

    htim4.Instance = TIM4;
    htim4.Init.Prescaler = 0;
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.Period = PWM_MAX;
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.RepetitionCounter = 0;
    if(HAL_TIM_PWM_Init(&htim4) != HAL_OK) Error_Handler();

    oc.OCMode = TIM_OCMODE_PWM1;
    oc.Pulse = 0;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    if(HAL_TIM_PWM_ConfigChannel(&htim4, &oc, PWM_RIGHT) != HAL_OK) Error_Handler();
    if(HAL_TIM_PWM_ConfigChannel(&htim4, &oc, PWM_LEFT) != HAL_OK) Error_Handler();

    HAL_TIM_MspPostInit(&htim4);
    HAL_TIM_PWM_Start(&htim4, PWM_RIGHT);
    HAL_TIM_PWM_Start(&htim4, PWM_LEFT);
}

void pwm_set(uint32_t channel, uint16_t duty){
    __HAL_TIM_SET_COMPARE(&htim4, channel, duty);
}
