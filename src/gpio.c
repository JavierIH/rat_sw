#include "gpio.h"
#include "stm32f1xx_hal.h"
#include "robot_config.h"

#define LED_L_PORT          GPIOB   // LED 1-3: PB13-PB15
#define LED_L_PINS          (GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15)
#define LED_R_PORT          GPIOA   // LED 4-6: PA3-PA5
#define LED_R_PINS          (GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5)
#define BUTTON_START_PORT   GPIOC
#define BUTTON_START_PIN    GPIO_PIN_13
#define BUTTON_SELECT_PORT  GPIOB
#define BUTTON_SELECT_PIN   GPIO_PIN_5

static const uint16_t LED_PIN[6] = {GPIO_PIN_13, GPIO_PIN_14, GPIO_PIN_15, GPIO_PIN_3, GPIO_PIN_4, GPIO_PIN_5};

typedef struct {
    uint8_t stable;             // debounced level
    uint8_t count;              // ms the raw level has differed from it
    volatile uint8_t pressed;   // latched press event
} button_state_t;

static button_state_t buttons[2];

static uint8_t button_raw(button_t button){
    if(button == BUTTON_START) return HAL_GPIO_ReadPin(BUTTON_START_PORT, BUTTON_START_PIN) == GPIO_PIN_SET;
    return HAL_GPIO_ReadPin(BUTTON_SELECT_PORT, BUTTON_SELECT_PIN) == GPIO_PIN_SET;
}

void LED_Init(void){
    GPIO_InitTypeDef gpio = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Pin = LED_L_PINS;
    HAL_GPIO_Init(LED_L_PORT, &gpio);
    gpio.Pin = LED_R_PINS;
    HAL_GPIO_Init(LED_R_PORT, &gpio);

    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_NOPULL;
    gpio.Pin = BUTTON_START_PIN;
    HAL_GPIO_Init(BUTTON_START_PORT, &gpio);
    gpio.Pin = BUTTON_SELECT_PIN;
    HAL_GPIO_Init(BUTTON_SELECT_PORT, &gpio);

    // A button already held at power-up is not a press.
    buttons[BUTTON_START].stable = button_raw(BUTTON_START);
    buttons[BUTTON_SELECT].stable = button_raw(BUTTON_SELECT);
}

void led_set(uint8_t led, uint8_t on){
    if(led < 1 || led > 6) return;
    GPIO_TypeDef *port = led <= 3 ? LED_L_PORT : LED_R_PORT;
    HAL_GPIO_WritePin(port, LED_PIN[led - 1], on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

// One BSRR write per port: atomic, so it is safe from the SysTick handler too.
void leds_set_mask(uint8_t mask){
    uint32_t left = 0, right = 0;
    for(uint8_t i = 0; i < 3; i++){
        if(mask & (1u << (5 - i))) left |= LED_PIN[i];
        if(mask & (1u << (2 - i))) right |= LED_PIN[3 + i];
    }
    LED_L_PORT->BSRR = left | ((uint32_t)(LED_L_PINS & ~left) << 16);
    LED_R_PORT->BSRR = right | ((uint32_t)(LED_R_PINS & ~right) << 16);
}

void leds_all(uint8_t on){
    leds_set_mask(on ? 0x3F : 0x00);
}

// The boot sweep of the original firmware (develop, 7dccc63 led_animation()),
// same order and timing: one LED runs 1 -> 6 -> 1, 50 ms per step.
#define LED_SWEEP_STEP_MS 50

void leds_sweep(uint8_t times){
    for(uint8_t n = 0; n < times; n++){
        led_set(1, 1);
        led_set(6, 0);
        HAL_Delay(LED_SWEEP_STEP_MS);
        for(uint8_t i = 2; i <= 6; i++){        // the next one on, then the previous off
            led_set(i, 1);
            led_set((uint8_t)(i - 1), 0);
            HAL_Delay(LED_SWEEP_STEP_MS);
        }
        for(uint8_t i = 5; i >= 1; i--){        // on the way back: off first, then on
            led_set((uint8_t)(i + 1), 0);
            led_set(i, 1);
            HAL_Delay(LED_SWEEP_STEP_MS);
        }
    }
    leds_all(0);
}

void leds_blink(uint8_t times, uint32_t half_period_ms){
    for(uint8_t i = 0; i < times; i++){
        leds_all(1);
        HAL_Delay(half_period_ms);
        leds_all(0);
        HAL_Delay(half_period_ms);
    }
}

void buttons_tick(void){
    for(uint8_t b = 0; b < 2; b++){
        button_state_t *s = &buttons[b];
        uint8_t raw = button_raw((button_t)b);
        if(raw == s->stable){
            s->count = 0;
        }
        else if(++s->count >= BUTTON_DEBOUNCE_MS){
            s->stable = raw;
            s->count = 0;
            if(raw) s->pressed = 1;
        }
    }
}

uint8_t button_take_press(button_t button){
    if(!buttons[button].pressed) return 0;
    buttons[button].pressed = 0;
    return 1;
}

void buttons_clear(void){
    buttons[BUTTON_START].pressed = 0;
    buttons[BUTTON_SELECT].pressed = 0;
}
