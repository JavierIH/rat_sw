#include "main.h"

int main(void) {
    HAL_Init();
    LED_Init();
    UART_Init();
    PWM_Init();

    SystemClock_Config();

    char text_buffer[100];
    sprintf(text_buffer,"running...\n\r");
    int i=0;
    int up=1;
    while (1){
        set_led(LED_6, LED_ON);
        HAL_Delay(50);
        set_led(LED_6, LED_OFF);
        HAL_Delay(50);
        set_pwm(PWM_1, i);
        set_pwm(PWM_2, 1000-i);
        i = (up ? i+50 : i-50);
        if(i>=1000) up=0;
        else if(i<=0) up=1;
        sprintf(text_buffer,"Hello %d\n\r", i);
        send_uart(text_buffer);
    }
}
