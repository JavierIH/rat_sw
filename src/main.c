#include "main.h"
#include "sysclock.h"
#include "error.h"
#include "uart.h"
#include "led.h"


void Error_Handler(void);

int main(void) {
    HAL_Init();
    LED_Init();
    UART_Init();

    SystemClock_Config();

    char text_buffer[100];
    sprintf(text_buffer,"HELLO");

    while (1){
        //HAL_GPIO_TogglePin(LED_GPIO_PORT, LED_PIN);
        //HAL_UART_Transmit(&huart3, (uint8_t *)text_buffer, len, 1000);
        set_led(LED_6, LED_ON);
        HAL_Delay(500);
        set_led(LED_6, LED_OFF);
        HAL_Delay(500);
        send_uart(text_buffer);
    }
}
