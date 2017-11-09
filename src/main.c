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

char text_buffer[200];

void setup(void);

int main(void) {
    setup();

    while(!get_button(BUTTON_START)){
        led_animation();
    }
    set_all_led(LED_ON);
    HAL_Delay(2000);
    set_all_led(LED_OFF);

    int yaw_error = 0;
    int x_error = 0;
    int cell_progress = 0;
    int ref_cell_progress = 0;
    while (1){
        /*sprintf(text_buffer,"IR_FL: %d\t", get_ir(IR_FL));
        send_uart(text_buffer);
        sprintf(text_buffer,"IR_FR: %d\t", get_ir(IR_FR));
        send_uart(text_buffer);
        sprintf(text_buffer,"IR_SL: %d\t", get_ir(IR_SL));
        send_uart(text_buffer);
        sprintf(text_buffer,"IR_SR: %d\n\r", get_ir(IR_SR));
        send_uart(text_buffer);/**/
        /*sprintf(text_buffer,"ENCODER_R: %d\t", __HAL_TIM_GetCounter(&htim2));
        send_uart(text_buffer);
        sprintf(text_buffer,"ENCODER_L: %d\n\r", __HAL_TIM_GetCounter(&htim1));
        send_uart(text_buffer);/**/

        yaw_error = 0;

        //Si hay muro por los dos lados corrije //TODO correcciones con un solo muros
        if (get_ir(IR_SR) > 150){ //725
            yaw_error = - 725 + get_ir(IR_SR);
            //yaw_error += 370;
            yaw_error/=3;
        }
        else if(get_ir(IR_SL) > 500){ // 875
            yaw_error = 875 - get_ir(IR_SL);
            //yaw_error += 370;
            yaw_error/=3;
        }




        int x_error = get_ir(IR_FL);
        if (x_error > get_ir(IR_FR)) x_error = get_ir(IR_FR);
        int wall_correction_L = -yaw_error;
        int wall_correction_R = yaw_error;

        if(x_error < 800 && cell_progress < 1600){ //cruzando celda
            set_speed(MOTOR_L, 230 + wall_correction_L);
            set_speed(MOTOR_R, 230 + wall_correction_R);
            cell_progress = get_encoder_diff(ref_cell_progress, __HAL_TIM_GetCounter(&htim2));
            //sprintf(text_buffer,"progress: %d\n\r", cell_progress); send_uart(text_buffer);
        }
        else{ // derecha, frente, izquierda, vuelta
            cell_progress = 0;
            if (get_ir(IR_SR) < 150){ //girar derecha
                set_speed(MOTOR_L, 0);
                set_speed(MOTOR_R, 0);
                HAL_Delay(150);
                //sprintf(text_buffer,"girar derecha\n\r"); send_uart(text_buffer);
                int init_position_L = __HAL_TIM_GetCounter(&htim1);
                while(get_encoder_diff(init_position_L, __HAL_TIM_GetCounter(&htim1)) > -340){
                    //sprintf(text_buffer,"%d\n\r", get_encoder_diff(__HAL_TIM_GetCounter(&htim1), init_position_L)); send_uart(text_buffer);
                    set_speed(MOTOR_L, 200);
                    set_speed(MOTOR_R, -200);
                }
                set_speed(MOTOR_L, 0);
                set_speed(MOTOR_R, 0);
                cell_progress = 0;
                ref_cell_progress = __HAL_TIM_GetCounter(&htim2);
                set_speed(MOTOR_L, 0);
                set_speed(MOTOR_R, 0);
                HAL_Delay(150);
            }
            else if(x_error < 400){ //seguir recto
                //sprintf(text_buffer,"seguir recto\n\r"); send_uart(text_buffer);
                cell_progress = 0;
                ref_cell_progress = __HAL_TIM_GetCounter(&htim2);
            }
            else if (get_ir(IR_SL) < 500){ //girar izquierda
                set_speed(MOTOR_L, 0);
                set_speed(MOTOR_R, 0);
                HAL_Delay(150);
                //sprintf(text_buffer,"girar izquierda\n\r"); send_uart(text_buffer);
                int init_position_L = __HAL_TIM_GetCounter(&htim1);
                while(get_encoder_diff(init_position_L, __HAL_TIM_GetCounter(&htim1)) < 340){
                    set_speed(MOTOR_L, -200);
                    set_speed(MOTOR_R, 200);
                }
                cell_progress = 0;
                ref_cell_progress = __HAL_TIM_GetCounter(&htim2);
                set_speed(MOTOR_L, 0);
                set_speed(MOTOR_R, 0);
                HAL_Delay(150);
            }
            else{
                //sprintf(text_buffer,"media vuelta\n\r"); send_uart(text_buffer);
                set_speed(MOTOR_L, 0);
                set_speed(MOTOR_R, 0);
                HAL_Delay(150);
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
                cell_progress = 0;
                ref_cell_progress = __HAL_TIM_GetCounter(&htim2);
                set_speed(MOTOR_L, 0);
                set_speed(MOTOR_R, 0);
                HAL_Delay(150);
            } //180 grados
        }
    }
}

void setup(void){
    HAL_Init();
    SystemClock_Config();
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
