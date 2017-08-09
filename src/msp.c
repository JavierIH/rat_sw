#include "stm32f1xx_hal.h"

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

void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{

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