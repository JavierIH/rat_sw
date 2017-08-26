#include "msp.h"

void HAL_MspInit(void){
    __HAL_RCC_AFIO_CLK_ENABLE();

    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

    // System interrupt init
    // MemoryManagement_IRQn interrupt configuration
    HAL_NVIC_SetPriority(MemoryManagement_IRQn, 0, 0);
    // BusFault_IRQn interrupt configuration
    HAL_NVIC_SetPriority(BusFault_IRQn, 0, 0);
    // UsageFault_IRQn interrupt configuration
    HAL_NVIC_SetPriority(UsageFault_IRQn, 0, 0);
    // SVCall_IRQn interrupt configuration
    HAL_NVIC_SetPriority(SVCall_IRQn, 0, 0);
    // DebugMonitor_IRQn interrupt configuration
    HAL_NVIC_SetPriority(DebugMonitor_IRQn, 0, 0);
    // PendSV_IRQn interrupt configuration
    HAL_NVIC_SetPriority(PendSV_IRQn, 0, 0);
    // SysTick_IRQn interrupt configuration
    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

    // NOJTAG: JTAG-DP Disabled and SW-DP Enabled
    __HAL_AFIO_REMAP_SWJ_NOJTAG();
}

void HAL_UART_MspInit(UART_HandleTypeDef* huart){
    GPIO_InitTypeDef GPIO_InitStruct;
    if(huart->Instance==USART3){
        // Peripheral clock enable
        __HAL_RCC_USART3_CLK_ENABLE();

        // USART3 GPIO Configuration
        // PB10     ------> USART3_TX
        // PB11     ------> USART3_RX

        GPIO_InitStruct.Pin = GPIO_PIN_10;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = GPIO_PIN_11;
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* huart){
    if(huart->Instance==USART3){
        // Peripheral clock disable
        __HAL_RCC_USART3_CLK_DISABLE();

        // USART3 GPIO Configuration
        // PB10     ------> USART3_TX
        // PB11     ------> USART3_RX

        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10|GPIO_PIN_11);
    }
}

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* htim_pwm){
    if(htim_pwm->Instance==TIM4){
        __HAL_RCC_TIM4_CLK_ENABLE();
    }
}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim){
    GPIO_InitTypeDef GPIO_InitStruct;
    if(htim->Instance==TIM4){

        //TIM4 GPIO Configuration
        //PB8     ------> TIM4_CH3
        //PB9     ------> TIM4_CH4

        GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    }
}

void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef* htim_encoder){
    GPIO_InitTypeDef GPIO_InitStruct;

    if(htim_encoder->Instance==TIM1){
        /* Peripheral clock enable */
        __HAL_RCC_TIM1_CLK_ENABLE();

        /**TIM3 GPIO Configuration
        PA8     ------> TIM1_CH1
        PA9     ------> TIM2_CH2
        */
        GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    }
    else if(htim_encoder->Instance==TIM2){
        /* Peripheral clock enable */
        __HAL_RCC_TIM2_CLK_ENABLE();

        /**TIM2 GPIO Configuration
        PA0-WKUP     ------> TIM2_CH1
        PA1     ------> TIM2_CH2
        */
        GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    }
}

void HAL_TIM_Encoder_MspDeInit(TIM_HandleTypeDef* htim_encoder){
    if(htim_encoder->Instance==TIM1){
        /* Peripheral clock disable */
        __HAL_RCC_TIM1_CLK_DISABLE();

        /**TIM3 GPIO Configuration
        PA8     ------> TIM1_CH1
        PA9     ------> TIM1_CH2
        */
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_8|GPIO_PIN_9);
    }
    else if(htim_encoder->Instance==TIM2){
        /* Peripheral clock disable */
        __HAL_RCC_TIM2_CLK_DISABLE();

        /**TIM2 GPIO Configuration
        PA0-WKUP     ------> TIM2_CH1
        PA1     ------> TIM2_CH2
        */
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0|GPIO_PIN_1);
    }
}
