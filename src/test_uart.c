#include "stm32f1xx_hal.h"
#include "gpio.h"
#include "sysclock.h"
#include "uart.h"

// UART / Bluetooth smoke test (env uart_test): motors, encoders and IR are
// never touched. The LEDs first blink the UART state right before the first
// transmit (1=READY 2=BUSY_TX 3=BUSY_RX 4=BUSY_TX_RX 5=BUSY 6=other), then a
// numbered line goes out every second.

extern UART_HandleTypeDef huart3;

static void blink_count(int n){
    leds_blink((uint8_t)n, 150);
    HAL_Delay(1500);
}

int main(void){
    HAL_Init();
    SystemClock_Config();
    LED_Init();
    UART_Init();

    HAL_Delay(2000);    // give the HC-05 time to reconnect before sending anything

    HAL_UART_StateTypeDef state = HAL_UART_GetState(&huart3);
    int state_code = 6;
    if(state == HAL_UART_STATE_READY) state_code = 1;
    else if(state == HAL_UART_STATE_BUSY_TX) state_code = 2;
    else if(state == HAL_UART_STATE_BUSY_RX) state_code = 3;
    else if(state == HAL_UART_STATE_BUSY_TX_RX) state_code = 4;
    else if(state == HAL_UART_STATE_BUSY) state_code = 5;
    blink_count(state_code);

    // Back to back (microseconds apart): shows whether specifically the very
    // first transmit is lost, independently of Bluetooth timing.
    print("first\n");
    print("second\n");
    HAL_Delay(2000);

    int counter = 0;
    print("UART test start\n");
    for(;;){
        leds_all(1);
        print("UART test #%d\n", counter++);
        HAL_Delay(200);
        leds_all(0);
        HAL_Delay(800);
    }
}
