#include "stm32f1xx_hal.h"
#include "msp.h"
#include "sysclock.h"
#include "error.h"
#include "uart.h"
#include "gpio.h"

// Visual-only diagnostic (independent of UART/Bluetooth) for the state right
// before the very first transmit attempt: 1=READY 2=BUSY_TX 3=BUSY_RX 4=BUSY_TX_RX 5=BUSY 6=other
static void blink_count(int n){
    for(int i = 0; i < n; i++){
        set_all_led(LED_ON);
        HAL_Delay(150);
        set_all_led(LED_OFF);
        HAL_Delay(150);
    }
    HAL_Delay(1500);
}

int main(void){
    HAL_Init();
    SystemClock_Config();
    LED_Init();
    UART_Init();

    HAL_Delay(2000); // give the HC-05 time to reconnect before sending anything

    HAL_UART_StateTypeDef state = HAL_UART_GetState(&huart3);
    int state_code = 6;
    if(state == HAL_UART_STATE_READY) state_code = 1;
    else if(state == HAL_UART_STATE_BUSY_TX) state_code = 2;
    else if(state == HAL_UART_STATE_BUSY_RX) state_code = 3;
    else if(state == HAL_UART_STATE_BUSY_TX_RX) state_code = 4;
    else if(state == HAL_UART_STATE_BUSY) state_code = 5;
    blink_count(state_code);

    // Sent back-to-back (microseconds apart): tells us if it's specifically
    // "the very first send ever" that's lost, regardless of BT/timing.
    print("first\n");
    print("second\n");
    HAL_Delay(2000);

    int counter = 0;
    print("UART test start\n");
    while(1){
        set_all_led(LED_ON);
        print("UART test #%d\n", counter++);
        HAL_Delay(200);
        set_all_led(LED_OFF);
        HAL_Delay(800);
    }
}

// not provided by main.c in this build, needed for HAL_Delay()
void SysTick_Handler(void){
    HAL_IncTick();
    HAL_SYSTICK_IRQHandler();
}
