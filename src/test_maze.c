#include "stm32f1xx_hal.h"
#include "msp.h"
#include "sysclock.h"
#include "error.h"
#include "uart.h"
#include "gpio.h"
#include "infrared.h"
#include "maze.h"

// Motors intentionally untouched (board powered over USB/ST-Link only).
// SELECT cycles the simulated heading, START senses walls at a fixed test
// cell and reports the result, so wall-sensing can be validated with loose
// walls (boxes/books) before the real maze arrives.
#define WALL_DETECT_MM 120
#define TEST_X 5
#define TEST_Y 5

static const char *heading_name(robot_heading_t h){
    switch(h){
        case UP_DIR:    return "UP(N)";
        case RIGHT_DIR: return "RIGHT(E)";
        case DOWN_DIR:  return "DOWN(S)";
        case LEFT_DIR:  return "LEFT(W)";
        default:        return "?";
    }
}

static uint8_t ir_confirms_wall(ir_sensor_t ir){
    int close_count = 0;
    for(int i = 0; i < 3; i++){
        if(get_ir_mm(ir) < WALL_DETECT_MM) close_count++;
        HAL_Delay(5);
    }
    return close_count >= 2;
}

int main(void){
    HAL_Init();
    SystemClock_Config();
    LED_Init();
    UART_Init();
    IR_Init();
    maze_init();

    HAL_Delay(1500);
    robot_heading_t heading = UP_DIR;
    print("Maze wall-sensing test. Test cell (%d,%d)\n", TEST_X, TEST_Y);
    print("SELECT = cambiar orientacion, START = sensar paredes\n");
    print("Orientacion actual: %s\n", heading_name(heading));

    uint8_t prev_select = 0, prev_start = 0;
    uint32_t last_heartbeat = HAL_GetTick();

    while(1){
        uint8_t select_now = get_button(BUTTON_SELECT);
        if(select_now && !prev_select){
            heading = (robot_heading_t)((heading + 1) % 4);
            print("Orientacion actual: %s\n", heading_name(heading));
            set_all_led(LED_ON);
            HAL_Delay(80);
            set_all_led(LED_OFF);
        }
        prev_select = select_now;

        uint8_t start_now = get_button(BUTTON_START);
        if(start_now && !prev_start){
            uint8_t front = ir_confirms_wall(IR_FL) && ir_confirms_wall(IR_FR);
            uint8_t left  = ir_confirms_wall(IR_SL);
            uint8_t right = ir_confirms_wall(IR_SR);

            if(front) maze_set_wall(TEST_X, TEST_Y, heading);
            if(left)  maze_set_wall(TEST_X, TEST_Y, maze_left_of(heading));
            if(right) maze_set_wall(TEST_X, TEST_Y, maze_right_of(heading));

            print("Sensado heading=%s front=%d left=%d right=%d -> paredes celda N=%d E=%d S=%d W=%d\n",
                  heading_name(heading), front, left, right,
                  maze_has_wall(TEST_X, TEST_Y, UP_DIR),
                  maze_has_wall(TEST_X, TEST_Y, RIGHT_DIR),
                  maze_has_wall(TEST_X, TEST_Y, DOWN_DIR),
                  maze_has_wall(TEST_X, TEST_Y, LEFT_DIR));
        }
        prev_start = start_now;

        if(HAL_GetTick() - last_heartbeat > 3000){
            print("(esperando SELECT/START... orientacion=%s)\n", heading_name(heading));
            last_heartbeat = HAL_GetTick();
        }

        HAL_Delay(50);
    }
}

// not provided by main.c in this build, needed for HAL_Delay()
void SysTick_Handler(void){
    HAL_IncTick();
    HAL_SYSTICK_IRQHandler();
}
