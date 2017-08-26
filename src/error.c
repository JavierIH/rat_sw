#include "error.h"

void Error_Handler(void){
    char tbuf[100];
    sprintf(tbuf,"ERROR HANDLER\n\r");
    send_uart(tbuf);
    while(1){
        set_led(LED_1, LED_OFF);
        set_led(LED_2, LED_OFF);
        set_led(LED_3, LED_OFF);
        set_led(LED_4, LED_OFF);
        set_led(LED_5, LED_OFF);
        set_led(LED_6, LED_OFF);
        HAL_Delay(50);
        set_led(LED_1, LED_ON);
        set_led(LED_2, LED_ON);
        set_led(LED_3, LED_ON);
        set_led(LED_4, LED_ON);
        set_led(LED_5, LED_ON);
        set_led(LED_6, LED_ON);
        HAL_Delay(50);
    }
}
