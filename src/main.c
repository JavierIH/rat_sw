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

#define     LANE_WIDTH              168.0 //mm
#define     MAX_SIDE_WALL_DIST      130.0 //mm
#define     TICKS_PER_CELL          1600 //ticks
#define     TICKS_PER_TURN          490 //ticks

typedef enum {UP_DIR, RIGHT_DIR, DOWN_DIR, LEFT_DIR} robot_heading_t;

robot_heading_t robot_heading = UP_DIR;
uint8_t robot_position_x = 1;
uint8_t robot_position_y = 1;
uint8_t goal_x = 8;
uint8_t goal_y = 8;

int started = 0;
float lateral_error = 0;
float prev_lateral_error = 0;
float kp = 2;
float kd = 30;
float pd_straight = 0;
int std_speed = 370;
int boost = 0;


void pid_config(int base_speed, float base_kp, float base_kd){
    set_output(MOTOR_L, 0);
    set_output(MOTOR_R, 0);
    set_all_led(LED_ON);
    print("PID configuration!\n");
    HAL_Delay(1000);
    int new_value = 0, value = 0;
    print("--- SET SPEED ---\n");
    control_all_led(0b00100000);
    reset_encoder(ENCODER_L);
    while(!get_button(BUTTON_START)){
        new_value = base_speed + (int16_t)get_encoder(ENCODER_L)/2;
        if(abs(value-new_value) >= 10){
            value = (new_value/10)*10;
            print("Speed: %d\n", value);
        }
    }
    std_speed = value;
    HAL_Delay(500);

    print("--- SET Kp ---\n");
    control_all_led(0b00010000);
    reset_encoder(ENCODER_L);
    value = 0;
    while(!get_button(BUTTON_START)){
        new_value = base_kp*10 + (int16_t)get_encoder(ENCODER_L)/20;
        if(value!=new_value){
            value = new_value;
            print("Kp: %d.%d\n", value/10, value%10);
        }
    }
    kp = value/10.0;
    HAL_Delay(500);

    print("--- SET Kd ---\n");
    control_all_led(0b00001000);
    reset_encoder(ENCODER_L);
    value = 0;
    while(!get_button(BUTTON_START)){
        new_value = base_kd*10 + (int16_t)get_encoder(ENCODER_L)/20;
        if(value!=new_value){
            value = new_value;
            print("Kd: %d.%d\n", value/10, value%10);
        }
    }
    kd = value/10.0;
    HAL_Delay(500);

    print("Speed: %d    ", std_speed);
    print("Kp: %d.%d    ", (int)kp, (int)(kp*10)%10);
    print("Kd: %d.%d\n", (int)kd, (int)(kd*10)%10);

    reset_encoder(ENCODER_L);
    reset_encoder(ENCODER_R);
}

void cross_cell(){
    int encoder_l_check = 0;
    int encoder_r_check = 0;
    int ir_check = 0;

    uint32_t initial_enc_l = get_encoder(ENCODER_L);
    uint32_t initial_enc_r = get_encoder(ENCODER_R);

    while(!(encoder_l_check || encoder_r_check || ir_check)){
        set_output(MOTOR_L, std_speed + boost + pd_straight);
        set_output(MOTOR_R, std_speed + boost - pd_straight);

        encoder_l_check = get_encoder_diff(initial_enc_l, get_encoder(ENCODER_L)) >= TICKS_PER_CELL;
        encoder_r_check = get_encoder_diff(initial_enc_r, get_encoder(ENCODER_R)) >= TICKS_PER_CELL;
        ir_check = get_ir_mm(IR_FL)<(94) && get_ir_mm(IR_FR)<(94);
    }

    // Update robot position
    switch (robot_heading) {
        case UP_DIR:    robot_position_y++; break;
        case DOWN_DIR:  robot_position_y--; break;
        case RIGHT_DIR: robot_position_x++; break;
        case LEFT_DIR:  robot_position_x--; break;
    }
}

void turn_left(){
    int encoder_l_check = 0;
    int encoder_r_check = 0;

    uint32_t initial_enc_l = get_encoder(ENCODER_L);
    uint32_t initial_enc_r = get_encoder(ENCODER_R);

    set_output(MOTOR_L, 0);
    set_output(MOTOR_R, 0);
    HAL_Delay(150);


    while(!(encoder_l_check || encoder_r_check)){
        set_output(MOTOR_L, -150);
        set_output(MOTOR_R, 150);

        encoder_l_check = get_encoder_diff(initial_enc_l, get_encoder(ENCODER_L)) <= -TICKS_PER_TURN;
        encoder_r_check = get_encoder_diff(initial_enc_r, get_encoder(ENCODER_R)) >= TICKS_PER_TURN;
    }

    set_output(MOTOR_L, 0);
    set_output(MOTOR_R, 0);
    HAL_Delay(150);

    // Update robot heading
    switch (robot_heading) {
        case UP_DIR:    robot_heading = LEFT_DIR; break;
        case DOWN_DIR:  robot_heading = RIGHT_DIR; break;
        case RIGHT_DIR: robot_heading = UP_DIR; break;
        case LEFT_DIR:  robot_heading = DOWN_DIR; break;
    }
}

void turn_right(){
    int encoder_l_check = 0;
    int encoder_r_check = 0;

    uint32_t initial_enc_l = get_encoder(ENCODER_L);
    uint32_t initial_enc_r = get_encoder(ENCODER_R);

    set_output(MOTOR_L, 0);
    set_output(MOTOR_R, 0);
    HAL_Delay(150);


    while(!(encoder_l_check || encoder_r_check)){
        set_output(MOTOR_L, 150);
        set_output(MOTOR_R, -150);

        encoder_l_check = get_encoder_diff(initial_enc_l, get_encoder(ENCODER_L)) >= TICKS_PER_TURN;
        encoder_r_check = get_encoder_diff(initial_enc_r, get_encoder(ENCODER_R)) <= -TICKS_PER_TURN;
    }

    set_output(MOTOR_L, 0);
    set_output(MOTOR_R, 0);
    HAL_Delay(150);

    // Update robot heading
    switch (robot_heading) {
        case UP_DIR:    robot_heading = RIGHT_DIR; break;
        case DOWN_DIR:  robot_heading = LEFT_DIR; break;
        case RIGHT_DIR: robot_heading = DOWN_DIR; break;
        case LEFT_DIR:  robot_heading = UP_DIR; break;
    }
}

void run_left_side(){
    robot_position_x = 1;
    robot_position_y = 1;
    robot_heading = UP_DIR;

    while(!get_button(BUTTON_START)){
        if(get_ir_mm(IR_SL) > 120){
            turn_left();
            cross_cell();
        }
        else if(get_ir_mm(IR_FL) > 120 || get_ir_mm(IR_FR) > 120){
            cross_cell();
        }
        else if(get_ir_mm(IR_SR) > 120){
            turn_right();
            cross_cell();
        }
        else{
            turn_right();
        } 
    }
}

void run_right_side(){
    robot_position_x = 1;
    robot_position_y = 1;
    robot_heading = UP_DIR;

    while(!get_button(BUTTON_START)){
        if(get_ir_mm(IR_SR) > 120){
            turn_right();
            cross_cell();
        }
        else if(get_ir_mm(IR_FL) > 120 || get_ir_mm(IR_FR) > 120){
            cross_cell();
        }
        else if(get_ir_mm(IR_SL) > 120){
            turn_left();
            cross_cell();
        }
        else{
            turn_left();
        }
    }
}


int main(void) {
    HAL_Init();
    SystemClock_Config();
    LED_Init();
    UART_Init();
    PWM_Init();
    MOTOR_Init();
    ENCODER_Init();
    IR_Init();

//    print("Press button to start\n");
//    while(!get_button(BUTTON_START)){
//        if(get_button(BUTTON_SELECT)){
//            fast_blink();
//            pid_config(std_speed, kp, kd);
//            fast_blink();
//        }
//        led_animation();
//    }
//    print("Click START to run left side or SELECT to run right side\n");

    while(1){
        if(get_button(BUTTON_START)){
            set_all_led(LED_ON);
            HAL_Delay(5000);
            set_all_led(LED_OFF);
            started = 1;
            HAL_Delay(20);
            run_left_side();
        }
        else if(get_button(BUTTON_SELECT)){
            set_all_led(LED_ON);
            HAL_Delay(5000);
            set_all_led(LED_OFF);
            started = 1;
            HAL_Delay(20);
            run_right_side();
        }
        led_animation();
    }
}

int current_speed = 0;
int diff_speed = 0;
float ir_dist = 0;

void SysTick_Handler(void){ // function executed each 1ms
    static uint16_t task_tick = 0;

    if(started) switch (task_tick++) { // loop runs at 100Hz
        case 0: // Boost
            //boost = 0;
            //if(get_ir_mm(IR_FL) > 200) boost = 100;
            break;

        case 1: // Calculate PID
            lateral_error = 0;
            ir_dist = get_ir_mm(IR_SR);
            if(ir_dist < MAX_SIDE_WALL_DIST){
                control_all_led(0b00000111);
                lateral_error = ir_dist - LANE_WIDTH/2.0;
            }
            else{
                ir_dist = get_ir_mm(IR_SL);
                if(ir_dist < MAX_SIDE_WALL_DIST){
                    control_all_led(0b00111000);
                    lateral_error = -ir_dist + LANE_WIDTH/2.0;
                }
                else{
                    set_all_led(LED_OFF);
                }
            }
            pd_straight = lateral_error*kp + (lateral_error-prev_lateral_error)*kd;
            prev_lateral_error = lateral_error; 
            break;

        case 9:
            task_tick = 0;
            break;
    }
    HAL_IncTick();
    HAL_SYSTICK_IRQHandler();
}
