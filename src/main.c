#include "stm32f1xx_hal.h"
#include "msp.h"
#include "sysclock.h"
#include "error.h"
#include "uart.h"
#include "pwm.h"
#include "gpio.h"
#include "motor.h"
#include "encoder.h"
#include "infrared.h"


static void MX_ADC1_Init(void);
void setup(void);
//void SysTick_Handler(void);

int main(void) {
    setup();
    char text_buffer[200];
    sprintf(text_buffer,"running...\n\r");
    int i=0;
    int up=1;
    int mtime=30;
    set_sense(MOTOR_R, FORWARD);
    set_sense(MOTOR_L, FORWARD);



    int32_t delta = 0;
    uint16_t before = 0;
    uint16_t after = 0;

    while(!get_button(BUTTON_START));
    int x=10;
    int cou=0;
    while (1){
        //sprintf(text_buffer,"%d ---\n\r", cou);
        //send_uart(text_buffer);
        //cou++;
        sprintf(text_buffer,"IR_FL: %d\t", get_ir(IR_FL));
        send_uart(text_buffer);
        sprintf(text_buffer,"IR_FR: %d\t", get_ir(IR_FR));
        send_uart(text_buffer);
        sprintf(text_buffer,"IR_SL: %d\t", get_ir(IR_SL));
        send_uart(text_buffer);
        sprintf(text_buffer,"IR_SR: %d\n\r", get_ir(IR_SR));
        send_uart(text_buffer);
        set_led(LED_1, LED_ON);
        HAL_Delay(x);
        set_led(LED_1, LED_OFF);
        HAL_Delay(x);
    }
}

void setup(void){
    HAL_Init();
    SystemClock_Config();

    // Peripheral setup
    LED_Init();
    UART_Init();
    //PWM_Init();
    //MOTOR_Init();
    //ENCODER_Init();
    IR_Init();
}

void SysTick_Handler(void){ // function executed each 1ms
    static uint16_t task_tick = 0;

    switch (task_tick++) { // each case is executed each 20ms
        case 0:
            update_speed(MOTOR_L);
            update_speed(MOTOR_R);
            break;
        case 1:
            //update speed pid
            break;
        //case 2:
            //update turn pid
        //    break;
        case 20:
            task_tick = 0;
            break;
    }
    HAL_IncTick();
    HAL_SYSTICK_IRQHandler();
}
