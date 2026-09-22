#include "stm32f1xx_hal.h"
#include "encoder.h"
#include "gpio.h"
#include "infrared.h"
#include "sysclock.h"
#include "uart.h"

// LED + IR + encoder live panel (env diag_test) for tools/dashboard.py.
// Motors are never driven (safe on ST-Link/USB power).
#define WALL_MM 200

int main(void){
    HAL_Init();
    SystemClock_Config();
    LED_Init();
    UART_Init();
    IR_Init();
    ENCODER_Init();

    HAL_Delay(500);

    for(;;){
        int fl = (int)ir_mm(IR_FL);
        int fr = (int)ir_mm(IR_FR);
        int sl = (int)ir_mm(IR_SL);
        int sr = (int)ir_mm(IR_SR);

        // Same layout as the robot's sensor monitor mode, so the panel also
        // checks that IR readings and LED wiring agree.
        uint8_t leds[6] = {sl < WALL_MM, fl < WALL_MM, fl < WALL_MM || fr < WALL_MM,
                           fl < WALL_MM || fr < WALL_MM, fr < WALL_MM, sr < WALL_MM};
        for(uint8_t i = 0; i < 6; i++) led_set((uint8_t)(i + 1), leds[i]);

        print("DATA,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%u,%u\n",
              leds[0], leds[1], leds[2], leds[3], leds[4], leds[5], sl, fl, fr, sr,
              encoder_raw(ENCODER_L), encoder_raw(ENCODER_R));

        HAL_Delay(150);
    }
}
