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
    set_led(LED_1, LED_OFF);

    while(!get_button(BUTTON_START));
    HAL_Delay(2000);

    int yaw_error;
    int x_error;
    while (1){
        /*sprintf(text_buffer,"IR_FL: %d\t", get_ir(IR_FL));
        send_uart(text_buffer);
        sprintf(text_buffer,"IR_FR: %d\t", get_ir(IR_FR));
        send_uart(text_buffer);
        sprintf(text_buffer,"IR_SL: %d\t", get_ir(IR_SL));
        send_uart(text_buffer);
        sprintf(text_buffer,"IR_SR: %d\n\r", get_ir(IR_SR));
        send_uart(text_buffer);
        set_led(LED_1, LED_ON);*/

        yaw_error = 0;

        //Si hay muro por los dos lados corrije
        if(get_ir(IR_SL) > 500 && get_ir(IR_SR) > 150){
            yaw_error = get_ir(IR_SR) - get_ir(IR_SL);
            yaw_error += 370;
            yaw_error/=4;
        }
        int x_error = get_ir(IR_FL);
        if (x_error > get_ir(IR_FR)) x_error = get_ir(IR_FR);
        int speed_L = -yaw_error;
        int speed_R = yaw_error;

        if(x_error < 800){
            set_speed(MOTOR_L, 200 + speed_L);
            set_speed(MOTOR_R, 200 + speed_R);
        }
        else{
            set_speed(MOTOR_L, 0);
            set_speed(MOTOR_R, 0);
            HAL_Delay(500);
            if (get_ir(IR_SL) < 500){ //girar izquierda
                int init_position_L = __HAL_TIM_GetCounter(&htim1);
                //int init_position_R = __HAL_TIM_GetCounter(&htim2);
                while(__HAL_TIM_GetCounter(&htim1) - init_position_L < 340){
                    set_speed(MOTOR_L, -200);
                    set_speed(MOTOR_R, 200);
                    sprintf(text_buffer,"ENCODER_L: %d\t[%d]\n\r", init_position_L, __HAL_TIM_GetCounter(&htim1) - init_position_L);
                    send_uart(text_buffer);
                }
                set_speed(MOTOR_L, 0);
                set_speed(MOTOR_R, 0);
                HAL_Delay(500);
            }
            else if (get_ir(IR_SR) < 150){ //girar derecha

            }
            else {
                int current_position_L = __HAL_TIM_GetCounter(&htim1);
                int final_position_L = current_position_L + 780;
                if (final_position_L > 65535) final_position_L -= 65535;

                int encoder_diff_L = get_encoder_diff(current_position_L, final_position_L);

                while(abs(encoder_diff_L) > 10){
                    encoder_diff_L = get_encoder_diff(__HAL_TIM_GetCounter(&htim1), final_position_L);
                    //sprintf(text_buffer,"DIFF: %d\t[%d]\n\r", encoder_diff_L, __HAL_TIM_GetCounter(&htim1));
                    //send_uart(text_buffer);
                    if(encoder_diff_L > 0){
                        set_speed(MOTOR_L, -100 -encoder_diff_L/4);
                        set_speed(MOTOR_R, +100 +encoder_diff_L/4);
                    }
                    else {
                        set_speed(MOTOR_L, +100 -encoder_diff_L/4);
                        set_speed(MOTOR_R, -100 +encoder_diff_L/4);
                    }
                    HAL_Delay(1);
                }

                set_speed(MOTOR_L, 0);
                set_speed(MOTOR_R, 0);
                HAL_Delay(500);
            } //180 grados
        }
    }
}

void setup(void){
    HAL_Init();
    SystemClock_Config();

    // Peripheral setup
    LED_Init();
    UART_Init();
    PWM_Init();
    MOTOR_Init();
    ENCODER_Init();
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
