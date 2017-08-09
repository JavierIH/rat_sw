#include "led.h"

void LED_Init(){
	GPIO_InitTypeDef GPIO_InitStruct;

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin : PB13 PB14 PB15 */
	GPIO_InitStruct.Pin = LED_1|LED_2|LED_3;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_MEDIUM;
	HAL_GPIO_Init(LED_GPIO_PORT_L, &GPIO_InitStruct);

	/*Configure GPIO pins : PA3 PA4 PA5 */
	GPIO_InitStruct.Pin = LED_4|LED_5|LED_6;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_MEDIUM;
	HAL_GPIO_Init(LED_GPIO_PORT_R, &GPIO_InitStruct);
}

void set_led(uint16_t led_pin, GPIO_PinState state){
	switch (led_pin) {
		case LED_1:
		case LED_2:
		case LED_3:
			HAL_GPIO_WritePin(LED_GPIO_PORT_L, led_pin, state);
			break;
		case LED_4:
		case LED_5:
		case LED_6:
			HAL_GPIO_WritePin(LED_GPIO_PORT_R, led_pin, state);
			break;
	}
}
