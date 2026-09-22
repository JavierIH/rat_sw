#include "stm32f1xx_hal.h"
#include "msp.h"
#include "sysclock.h"
#include "error.h"
#include "uart.h"
#include "gpio.h"
#include "maze.h"

// Pure logic test: no IR/motors/encoders needed, board can sit still. Builds
// a small 4x3 maze (matching the real 4x3 practice maze) with one internal
// wall segment that forces a detour, then prints the flood-fill grid and the
// path the robot would take -- everything verifiable by hand.

#define TW 4 // width  (x: 0..3)
#define TH 3 // height (y: 0..2)

static const char *heading_name(robot_heading_t h){
    switch(h){
        case UP_DIR:    return "UP";
        case RIGHT_DIR: return "RIGHT";
        case DOWN_DIR:  return "DOWN";
        case LEFT_DIR:  return "LEFT";
        default:        return "?";
    }
}

// Keep test output sequential and visible before advancing.
static void wait_uart_ready(void){
    while(!uart_tx_idle());
}

int main(void){
    HAL_Init();
    SystemClock_Config();
    LED_Init();
    UART_Init();
    maze_init();

    HAL_Delay(1500);

    // Enclose the 4x3 practice area (x=0 / y=0 are already real-maze borders
    // from maze_init(); fence the right/top edges of this test area too).
    for(int x = 0; x < TW; x++) maze_set_wall(x, TH - 1, UP_DIR);
    for(int y = 0; y < TH; y++) maze_set_wall(TW - 1, y, RIGHT_DIR);

    // Internal wall: only the bottom row connects the left half (x=0,1) to
    // the right half (x=2,3) -- forces a real detour, not just a Manhattan path.
    maze_set_wall(1, 1, RIGHT_DIR);
    maze_set_wall(1, 2, RIGHT_DIR);

    maze_set_goal(TW - 1, TH - 1, TW - 1, TH - 1); // single-cell goal, top-right corner
    maze_compute_flood();

    print("Flood fill test (maze 4x3, meta esquina sup-derecha)\n");
    wait_uart_ready();
    print("Valores de flood por celda (fila y=2 arriba, y=0 abajo):\n");
    wait_uart_ready();
    for(int y = TH - 1; y >= 0; y--){
        for(int x = 0; x < TW; x++){
            print("%4u", maze_flood[x][y]);
            wait_uart_ready();
        }
        print("\n");
        wait_uart_ready();
    }

    print("Camino simulado desde (0,0) mirando UP:\n");
    wait_uart_ready();
    int8_t x = 0, y = 0;
    robot_heading_t heading = UP_DIR;
    int steps = 0;
    while(!maze_is_goal(x, y) && steps < 20){
        heading = maze_next_move(x, y, heading);
        print("  (%d,%d) flood=%u heading=%s -> ", x, y, maze_flood[x][y], heading_name(heading));
        wait_uart_ready();
        switch(heading){
            case UP_DIR:    y++; break;
            case RIGHT_DIR: x++; break;
            case DOWN_DIR:  y--; break;
            case LEFT_DIR:  x--; break;
        }
        print("(%d,%d)\n", x, y);
        wait_uart_ready();
        steps++;
    }
    print("Llegada a la meta en %d pasos, flood final=%u\n", steps, maze_flood[x][y]);
    wait_uart_ready();

    while(1){
        set_all_led(LED_ON);
        HAL_Delay(300);
        set_all_led(LED_OFF);
        HAL_Delay(700);
    }
}

// not provided by main.c in this build, needed for HAL_Delay()
void SysTick_Handler(void){
    HAL_IncTick();
    HAL_SYSTICK_IRQHandler();
}
