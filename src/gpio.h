#ifndef GPIO_H
#define GPIO_H

#include <stdint.h>

// Six LEDs (1-3 left, 4-6 right) and two buttons (high when pressed, external resistors, debounced).

typedef enum { BUTTON_START, BUTTON_SELECT } button_t;

void LED_Init(void);                        // LEDs and buttons
void led_set(uint8_t led, uint8_t on);      // led: 1..6
void leds_set_mask(uint8_t mask);           // bit 5 = LED 1 ... bit 0 = LED 6
void leds_all(uint8_t on);
void leds_blink(uint8_t times, uint32_t half_period_ms);   // blocking, every LED
void leds_flash(uint8_t a, uint8_t b, uint8_t times, uint32_t half_period_ms);   // blocking, masks a and b alternating
void leds_sweep_frame(uint32_t ms);         // boot animation at time ms: one LED 1 -> 6 -> 1

void buttons_tick(void);                    // every 1 ms (SysTick)
uint8_t button_take_press(button_t button); // 1 once per debounced press
void buttons_clear(void);

#endif // GPIO_H
