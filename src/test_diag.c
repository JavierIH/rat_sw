#include "stm32f1xx_hal.h"
#include "msp.h"
#include "sysclock.h"
#include "error.h"
#include "uart.h"
#include "gpio.h"
#include "infrared.h"
#include "encoder.h"

// Motors intentionally untouched here (board powered over USB/ST-Link only).
#define WALL_MM 200

int main(void){
    HAL_Init();
    SystemClock_Config();
    LED_Init();
    UART_Init();
    IR_Init();
    ENCODER_Init();

    HAL_Delay(500);

    while(1){
        int fl = (int)get_ir_mm(IR_FL);
        int fr = (int)get_ir_mm(IR_FR);
        int sl = (int)get_ir_mm(IR_SL);
        int sr = (int)get_ir_mm(IR_SR);

        // Mirror the real robot's wall-proximity LED indicators so the panel
        // doubles as a check that IR readings and LED wiring make sense together.
        set_led(LED_1, (sl < WALL_MM) ? LED_ON : LED_OFF);
        set_led(LED_2, (fl < WALL_MM) ? LED_ON : LED_OFF);
        set_led(LED_3, (fl < WALL_MM || fr < WALL_MM) ? LED_ON : LED_OFF);
        set_led(LED_4, (fl < WALL_MM || fr < WALL_MM) ? LED_ON : LED_OFF);
        set_led(LED_5, (fr < WALL_MM) ? LED_ON : LED_OFF);
        set_led(LED_6, (sr < WALL_MM) ? LED_ON : LED_OFF);

        int l1 = HAL_GPIO_ReadPin(LED_L_PORT, LED_1);
        int l2 = HAL_GPIO_ReadPin(LED_L_PORT, LED_2);
        int l3 = HAL_GPIO_ReadPin(LED_L_PORT, LED_3);
        int l4 = HAL_GPIO_ReadPin(LED_R_PORT, LED_4);
        int l5 = HAL_GPIO_ReadPin(LED_R_PORT, LED_5);
        int l6 = HAL_GPIO_ReadPin(LED_R_PORT, LED_6);

        int enc_l = get_encoder(ENCODER_L);
        int enc_r = get_encoder(ENCODER_R);

        print("DATA,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
              l1, l2, l3, l4, l5, l6, sl, fl, fr, sr, enc_l, enc_r);

        HAL_Delay(150);
    }
}

// not provided by main.c in this build, needed for HAL_Delay()
void SysTick_Handler(void){
    HAL_IncTick();
    HAL_SYSTICK_IRQHandler();
}
