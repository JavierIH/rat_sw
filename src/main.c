#include "stm32f1xx_hal.h"
#include "app.h"
#include "commands.h"
#include "encoder.h"
#include "gpio.h"
#include "infrared.h"
#include "maze.h"
#include "motion.h"
#include "motor.h"
#include "pwm.h"
#include "robot_config.h"
#include "search.h"
#include "storage.h"
#include "sysclock.h"
#include "uart.h"

static const char *const MODE_NAME[MODE_COUNT + 1] = {
    "?", "BUSQUEDA", "CARRERA RAPIDA", "SEGUIDOR IZQ", "SEGUIDOR DER", "SENSORES", "BORRAR MAPA",
};

static uint8_t mode = MODE_SEARCH;
static uint8_t run_active;
static volatile uint8_t start_requested;

uint8_t app_run_active(void){ return run_active; }
uint8_t app_mode(void){ return mode; }
void app_request_start(void){ start_requested = 1; }

uint8_t app_set_mode(uint8_t m){
    if(m < 1 || m > MODE_COUNT) return 0;
    mode = m;
    return 1;
}

const char *app_mode_name(uint8_t m){
    return m <= MODE_COUNT ? MODE_NAME[m] : "?";
}

// Every millisecond, from SysTick.
void app_systick(void){
    encoder_tick();
    buttons_tick();
    motion_tick_1ms();
}

// Idle: the selected mode's LED, briefly off once a second as a heartbeat.
static void show_mode(void){
    uint8_t on = (HAL_GetTick() % 1000u) >= 100u;
    leds_set_mask(on ? (uint8_t)(1u << (MODE_COUNT - mode)) : 0u);
}

// Live sensor check (e.g. in the maze before a run). Motors stay off.
static void sensor_monitor(void){
    print("Monitor de sensores: cualquier boton o STOP para salir\n");
    uint32_t next_print = HAL_GetTick();
    for(;;){
        commands_poll();
        if(motion_abort_requested()) break;
        if(button_take_press(BUTTON_START) || button_take_press(BUTTON_SELECT)) break;
        uint8_t fl = ir_mm(IR_FL) < WALL_DETECT_MM;
        uint8_t fr = ir_mm(IR_FR) < WALL_DETECT_MM;
        uint8_t sl = ir_mm(IR_SL) < WALL_DETECT_MM;
        uint8_t sr = ir_mm(IR_SR) < WALL_DETECT_MM;
        // LED 1 left wall, 2 front-left, 3-4 any front, 5 front-right, 6 right wall.
        leds_set_mask((uint8_t)((sl << 5) | (fl << 4) | ((fl || fr) ? 0x0Cu : 0u) | (fr << 1) | sr));
        if((int32_t)(HAL_GetTick() - next_print) >= 0){
            next_print += 250;
            print("IR mm FL=%d FR=%d SL=%d SR=%d\n",
                  (int)ir_mm(IR_FL), (int)ir_mm(IR_FR), (int)ir_mm(IR_SL), (int)ir_mm(IR_SR));
        }
    }
    leds_all(0);
}

// Erasing needs a second START press within 3 s, so a mis-selected mode
// cannot wipe a map by accident.
static void erase_map_confirmed(void){
    print("BORRAR MAPA: pulsa START otra vez en 3 s para confirmar\n");
    uint32_t start = HAL_GetTick();
    while(HAL_GetTick() - start < 3000u){
        commands_poll();
        leds_all((uint8_t)(((HAL_GetTick() - start) / 100u) & 1u));
        if(button_take_press(BUTTON_START)){
            maze_init();
            leds_all(0);
            print(storage_save() ? "Mapa borrado\n" : "!! error escribiendo la flash\n");
            return;
        }
        if(button_take_press(BUTTON_SELECT) || motion_abort_requested()) break;
    }
    leds_all(0);
    print("borrado cancelado\n");
}

static void run_mode(uint8_t m){
    motion_clear_abort();
    buttons_clear();
    run_active = 1;
    if(m == MODE_SENSORS){
        sensor_monitor();
    }
    else if(m == MODE_ERASE){
        erase_map_confirmed();
    }
    else if(m == MODE_FAST && search_fast_path_cost() == PLAN_INF){
        print("Sin camino verificado salida->meta: haz antes una busqueda (modo 1)\n");
    }
    else{
        print("Modo %u %s: arranca en %u ms (START o STOP cancela)\n", m, MODE_NAME[m], START_DELAY_MS);
        leds_all(1);
        uint8_t go = motion_wait(START_DELAY_MS);   // hands away
        leds_all(0);
        if(!go){
            print("cancelado\n");
        }
        else{
            run_result_t r = RUN_FAILED;
            switch(m){
                case MODE_SEARCH:       r = search_explore(); break;
                case MODE_FAST:         r = search_fast_run(); break;
                case MODE_FOLLOW_LEFT:  r = search_wall_follow(1); break;
                case MODE_FOLLOW_RIGHT: r = search_wall_follow(0); break;
                default: break;
            }
            motion_stop();
            static const char *const RESULT[] = {"OK", "ABORTADO", "FALLO"};
            print("Fin: %s | %s\n", RESULT[r],
                  search_ready() ? "robot en la salida, listo" : "coloca el robot en la salida");
        }
    }
    motion_stop();
    run_active = 0;
    motion_clear_abort();
    buttons_clear();
}

static void print_banner(storage_status_t stored){
    uint8_t g[4];
    maze_get_goal(g);
    print("\nrat_sw %s %s | modo %u %s\n", __DATE__, __TIME__, mode, MODE_NAME[mode]);
    if(stored == STORAGE_LOADED){
        uint16_t cost = search_fast_path_cost();
        if(cost == PLAN_INF){
            print("Mapa en flash: %u celdas, sin camino rapido (ERASE si es otro laberinto)\n", maze_visited_count());
        }
        else{
            print("Mapa en flash: %u celdas, camino rapido coste %u (ERASE si es otro laberinto)\n",
                  maze_visited_count(), cost);
        }
    }
    else{
        print("Flash: %s\n", storage_status_name(stored));
    }
    print("Meta (%u,%u)-(%u,%u). SELECT cambia de modo, START lo lanza, HELP lista comandos\n",
          g[0], g[1], g[2], g[3]);
}

int main(void){
    HAL_Init();
    SystemClock_Config();
    LED_Init();
    UART_Init();
    PWM_Init();
    MOTOR_Init();
    ENCODER_Init();
    IR_Init();
    maze_init();
    storage_status_t stored = storage_load();
    uart_start_receive();
    print_banner(stored);

    for(;;){
        commands_poll();
        if(button_take_press(BUTTON_SELECT)){
            mode = (uint8_t)(mode % MODE_COUNT + 1);
            print("modo %u: %s\n", mode, MODE_NAME[mode]);
        }
        uint8_t pressed = button_take_press(BUTTON_START);
        if(pressed) search_set_home();  // someone is at the robot: it stands at the start
        if(pressed || start_requested){
            start_requested = 0;
            run_mode(mode);
        }
        show_mode();
    }
}
