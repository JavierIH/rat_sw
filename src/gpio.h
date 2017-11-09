#ifndef LED_H
#define LED_H

#include "stm32f1xx_hal.h"

#define LED_1                   GPIO_PIN_13
#define LED_2                   GPIO_PIN_14
#define LED_3                   GPIO_PIN_15
#define LED_4                   GPIO_PIN_3
#define LED_5                   GPIO_PIN_4
#define LED_6                   GPIO_PIN_5
#define BUTTON_START            GPIO_PIN_13
#define BUTTON_SELECT           GPIO_PIN_5

#define LED_L_PORT              GPIOB
#define LED_R_PORT              GPIOA
#define BUTTON_START_PORT       GPIOC
#define BUTTON_SELECT_PORT      GPIOB

#define LED_ON                  GPIO_PIN_SET
#define LED_OFF                 GPIO_PIN_RESET

void GPIO_Init();
void set_led(uint16_t led_pin, GPIO_PinState state);
void set_all_led(GPIO_PinState state);
void led_animation();
GPIO_PinState get_button(uint16_t button_pin);


#endif // LED_H
