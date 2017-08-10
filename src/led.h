#ifndef LED_H
#define LED_H

#include "stm32f1xx_hal.h"

#define LED_1                   GPIO_PIN_13
#define LED_2                   GPIO_PIN_14
#define LED_3                   GPIO_PIN_15
#define LED_4                   GPIO_PIN_3
#define LED_5                   GPIO_PIN_4
#define LED_6                   GPIO_PIN_5
#define LED_GPIO_PORT_L         GPIOB
#define LED_GPIO_PORT_R         GPIOA
#define LED_ON                  GPIO_PIN_SET
#define LED_OFF                 GPIO_PIN_RESET

void LED_Init();
void set_led(uint16_t led_pin, GPIO_PinState state);


#endif // LED_H
