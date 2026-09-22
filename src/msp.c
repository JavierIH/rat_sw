#include "msp.h"
#include "error.h"

extern DMA_HandleTypeDef hdma_adc1;         // infrared.c
extern DMA_HandleTypeDef hdma_usart3_tx;    // uart.c

void HAL_MspInit(void){
    __HAL_RCC_AFIO_CLK_ENABLE();
    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
    HAL_NVIC_SetPriority(MemoryManagement_IRQn, 0, 0);
    HAL_NVIC_SetPriority(BusFault_IRQn, 0, 0);
    HAL_NVIC_SetPriority(UsageFault_IRQn, 0, 0);
    HAL_NVIC_SetPriority(SVCall_IRQn, 0, 0);
    HAL_NVIC_SetPriority(DebugMonitor_IRQn, 0, 0);
    HAL_NVIC_SetPriority(PendSV_IRQn, 0, 0);
    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

    // Keep SWD, release the JTAG-only pins PA15/PB3/PB4 (motor inputs).
    __HAL_AFIO_REMAP_SWJ_NOJTAG();
}

void HAL_UART_MspInit(UART_HandleTypeDef *huart){
    if(huart->Instance != USART3) return;
    GPIO_InitTypeDef gpio = {0};
    __HAL_RCC_USART3_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();

    gpio.Pin = GPIO_PIN_10;     // USART3_TX
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &gpio);
    gpio.Pin = GPIO_PIN_11;     // USART3_RX
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &gpio);

    // USART3_TX is hardwired to DMA1 channel 2 on the STM32F1.
    hdma_usart3_tx.Instance = DMA1_Channel2;
    hdma_usart3_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart3_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart3_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart3_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart3_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart3_tx.Init.Mode = DMA_NORMAL;
    hdma_usart3_tx.Init.Priority = DMA_PRIORITY_LOW;
    if(HAL_DMA_Init(&hdma_usart3_tx) != HAL_OK) Error_Handler();
    __HAL_LINKDMA(huart, hdmatx, hdma_usart3_tx);

    HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
    // The end of a DMA transmit (TC) and every received byte arrive on the
    // USART3 interrupt.
    HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
}

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim){
    if(htim->Instance == TIM4) __HAL_RCC_TIM4_CLK_ENABLE();
}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim){
    if(htim->Instance != TIM4) return;
    GPIO_InitTypeDef gpio = {0};
    __HAL_RCC_GPIOB_CLK_ENABLE();
    gpio.Pin = GPIO_PIN_8 | GPIO_PIN_9;     // TIM4_CH3 (right), TIM4_CH4 (left)
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &gpio);
}

void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef *htim){
    GPIO_InitTypeDef gpio = {0};
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_NOPULL;
    if(htim->Instance == TIM1){
        __HAL_RCC_TIM1_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        gpio.Pin = GPIO_PIN_8 | GPIO_PIN_9;     // TIM1_CH1/CH2: left encoder
        HAL_GPIO_Init(GPIOA, &gpio);
    }
    else if(htim->Instance == TIM2){
        __HAL_RCC_TIM2_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1;     // TIM2_CH1/CH2: right encoder
        HAL_GPIO_Init(GPIOA, &gpio);
    }
}

void HAL_ADC_MspInit(ADC_HandleTypeDef *hadc){
    if(hadc->Instance != ADC1) return;
    GPIO_InitTypeDef gpio = {0};
    __HAL_RCC_ADC1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pin = GPIO_PIN_6 | GPIO_PIN_7;     // ADC1_IN6, ADC1_IN7
    HAL_GPIO_Init(GPIOA, &gpio);
    gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1;     // ADC1_IN8, ADC1_IN9
    HAL_GPIO_Init(GPIOB, &gpio);

    // Circular, 16-bit samples. No DMA/ADC interrupts are enabled on purpose
    // (see IR_Init).
    hdma_adc1.Instance = DMA1_Channel1;
    hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc1.Init.Mode = DMA_CIRCULAR;
    hdma_adc1.Init.Priority = DMA_PRIORITY_LOW;
    if(HAL_DMA_Init(&hdma_adc1) != HAL_OK) Error_Handler();
    __HAL_LINKDMA(hadc, DMA_Handle, hdma_adc1);
}
