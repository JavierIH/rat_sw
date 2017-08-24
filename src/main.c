#include "main.h"



int main(void) {
    HAL_Init();

    SystemClock_Config();

    LED_Init();
    UART_Init();
    PWM_Init();

    char text_buffer[200];
    sprintf(text_buffer,"running...\n\r");
    int i=0;
    int up=1;
    int mtime=30;
    while (1){
        set_led(LED_1, LED_OFF);
        set_led(LED_2, LED_ON);
        HAL_Delay(mtime);
        set_led(LED_2, LED_OFF);
        set_led(LED_3, LED_ON);
        HAL_Delay(mtime);
        set_led(LED_3, LED_OFF);
        set_led(LED_4, LED_ON);
        HAL_Delay(mtime);
        set_led(LED_4, LED_OFF);
        set_led(LED_5, LED_ON);
        HAL_Delay(mtime);
        set_led(LED_5, LED_OFF);
        set_led(LED_6, LED_ON);
        HAL_Delay(mtime*2);
        set_led(LED_6, LED_OFF);
        set_led(LED_5, LED_ON);
        HAL_Delay(mtime);
        set_led(LED_5, LED_OFF);
        set_led(LED_4, LED_ON);
        HAL_Delay(mtime);
        set_led(LED_4, LED_OFF);
        set_led(LED_3, LED_ON);
        HAL_Delay(mtime);
        set_led(LED_3, LED_OFF);
        set_led(LED_2, LED_ON);
        HAL_Delay(mtime);
        set_led(LED_2, LED_OFF);
        set_led(LED_1, LED_ON);
        HAL_Delay(mtime*2);
        set_pwm(PWM_1, i);
        set_pwm(PWM_2, 500);
        i = (up ? i+50 : i-50);
        if(i>=1000) up=0;
        else if(i<=0) up=1;
        sprintf(text_buffer,"HOLA %d\n\r", HAL_RCC_GetHCLKFreq());
        send_uart(text_buffer);
        if(get_button(BUTTON_SELECT)||get_button(BUTTON_START)){
            mtime=10;
        }
        else{
            mtime=100;
        }
    }
}
