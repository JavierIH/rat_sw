#include "main.h"



int main(void) {
    HAL_Init();

    SystemClock_Config();

    LED_Init();
    UART_Init();
    //PWM_Init();
    //MOTOR_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();
    //ENCODER_Init();

    char text_buffer[200];
    sprintf(text_buffer,"running...\n\r");
    int i=0;
    int up=1;
    int mtime=30;
    //set_sense(MOTOR_R, 1);
    //set_sense(MOTOR_L, 0);
    int x=0;
    for(x=0; x<10; x++){
        set_led(LED_1, LED_ON);
        HAL_Delay(30);
        set_led(LED_1, LED_OFF);
        HAL_Delay(30);
    }
    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
    //while(!get_button(BUTTON_START));
    while (1){
        //set_pwm(PWM_1, i);
        //set_pwm(PWM_2, 1000-i);
        /*if((int)__HAL_TIM_GetCounter(&htim1)%2){
            set_led(LED_1, LED_ON);
        }
        else{
            set_led(LED_1, LED_OFF);
        }
        /*i = (up ? i+1 : i-1);
        if(i>=1000) up=0;
        else if(i<=0) up=1;*/
        //HAL_Delay(100);
        //set_led(LED_1, LED_ON);
        HAL_Delay(100);
        //set_led(LED_1, LED_OFF);

        sprintf(text_buffer,"HOLA %d\n\r", get_encoder(ENCODER_L));
        send_uart(text_buffer);

    }
}
