#include "main.h"

int main(void) {
    HAL_Init();
    LED_Init();
    UART_Init();

    SystemClock_Config();

    PWM_Init();


    char text_buffer[100];
    sprintf(text_buffer,"running...\n\r");
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, 500);
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 500);
    while (1){
        //HAL_GPIO_TogglePin(LED_GPIO_PORT, LED_PIN);
        //HAL_UART_Transmit(&huart3, (uint8_t *)text_buffer, len, 1000);
        set_led(LED_6, LED_ON);
        HAL_Delay(50);
        set_led(LED_6, LED_OFF);
        HAL_Delay(50);
        sprintf(text_buffer,"\n\nCNT: %d\n\r", TIM4->CNT);
        send_uart(text_buffer);
        sprintf(text_buffer,"PIN B9: %d\n\r", HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9));
        send_uart(text_buffer);
    }
}
