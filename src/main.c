#include <stdio.h>
#include "stm32f1xx_hal.h"
#include "app.h"
#include "calib.h"
#include "commands.h"
#include "encoder.h"
#include "flash_store.h"
#include "gpio.h"
#include "health.h"
#include "light_check.h"
#include "infrared.h"
#include "maze.h"
#include "motion.h"
#include "motor.h"
#include "params.h"
#include "pwm.h"
#include "robot_config.h"
#include "search.h"
#include "storage.h"
#include "sysclock.h"
#include "telemetry.h"
#include "uart.h"

static const char *const MODE_NAME[MODE_COUNT + 1] = {
    "?", "SEARCH", "LEFT FOLLOWER", "RIGHT FOLLOWER", "SAFE RACE", "RACE", "ERASE MAP", "NO CURVES",
    "MID RACE",
};
// The race menu's order (app.h).
static const uint8_t RACE_MODE[RACE_COUNT + 1] = {
    0, MODE_FOLLOW_LEFT, MODE_FOLLOW_RIGHT, MODE_NO_CURVES, MODE_FAST_SAFE, MODE_FAST_MID, MODE_FAST,
};

static uint8_t mode = MODE_SEARCH;
static uint8_t menu = MENU_SEARCH;
static uint8_t race = 1;
static uint8_t race_menu;           // 1 once START opened it: SELECT cycles the races
static uint8_t mode_chosen;         // 0 after boot: the LEDs sweep until SELECT (or MODE, START)
static uint8_t run_active;
static volatile uint8_t start_requested;
static volatile uint8_t check_requested;
#if DEV_TOOLS
static volatile uint8_t cal_requested;
static cal_test_t cal_test;
static int32_t cal_a, cal_b;
#endif

uint8_t app_run_active(void){ return run_active; }
uint8_t app_mode(void){ return mode; }
void app_request_start(void){ start_requested = 1; }
void app_request_check(void){ check_requested = 1; }

#if DEV_TOOLS
void app_request_cal(cal_test_t test, int32_t a, int32_t b){
    cal_test = test;
    cal_a = a;
    cal_b = b;
    cal_requested = 1;
}

static volatile uint8_t remote_requested;
static volatile uint8_t remote_waiting;
static int8_t remote_turn;

void app_request_remote(void){ remote_requested = 1; }

uint8_t app_remote_go(int8_t quarter_turns){
    if(!remote_waiting) return 0;
    remote_turn = quarter_turns;
    remote_waiting = 0;
    return 1;
}

// REMOTE's decisions: the next GO from the console; STOP or START end the run. First the cell, its walls and the IR
// as they read now (mm), for a driver that judges the walls itself. The robot is stopped: wait for the queue to empty
// first, as print drops a line when it is full and the driver waits for both (10-10: the IR line lost at (0,2)).
static int8_t console_decide(const char walls[4], void *ctx){
    static const char HEADING[4] = {'N', 'E', 'S', 'W'};
    (void)ctx;
    uint8_t x, y;
    heading_t h;
    search_pose(&x, &y, &h);
    uart_flush(1000);
    print("remote (%u,%u)%c walls front=%c left=%c right=%c back=%c\n", x, y, HEADING[h],
          walls[0], walls[1], walls[2], walls[3]);
    print("remote ir FL=%d FR=%d SL=%d SR=%d\n", (int)ir_mm(IR_FL), (int)ir_mm(IR_FR), (int)ir_mm(IR_SL),
          (int)ir_mm(IR_SR));
    uint8_t go_on = 1;
    remote_waiting = 1;
    while(remote_waiting && go_on) go_on = motion_checkpoint();
    remote_waiting = 0;
    return go_on ? remote_turn : REMOTE_STOP;
}
#endif

uint8_t app_set_mode(uint8_t m, uint8_t r){
    if(m < 1 || m > MENU_COUNT || r > RACE_COUNT) return 0;
    menu = m;
    if(r) race = r;
    race_menu = m == MENU_RACE && (r || race_menu);     // MODE 2 n over the console opens it too
    mode = m == MENU_SEARCH ? MODE_SEARCH : m == MENU_ERASE ? MODE_ERASE : RACE_MODE[race];
    mode_chosen = 1;
    if(mode == MODE_FAST_SAFE || mode == MODE_NO_CURVES){
        params.fast_speed = FAST_SAFE_SPEED;
        params.curve_speed = FAST_SAFE_CURVE;
    }
    else if(mode == MODE_FAST_MID){
        params.fast_speed = FAST_MID_SPEED;
        params.curve_speed = FAST_MID_CURVE;
    }
    else if(mode == MODE_FAST){
        params.fast_speed = FAST_FULL_SPEED;
        params.curve_speed = FAST_FULL_CURVE;
    }
    telemetry_mode(mode);
    return 1;
}

const char *app_mode_label(void){
    static char label[24];
    if(menu == MENU_RACE) snprintf(label, sizeof(label), "2.%u %s", race, MODE_NAME[mode]);
    else snprintf(label, sizeof(label), "%u %s", menu, MODE_NAME[mode]);
    return label;
}

static void sync_telemetry(telemetry_activity_t activity){
    uint8_t x, y;
    heading_t h;
    search_pose(&x, &y, &h);
    telemetry_sync(mode, activity, x, y, h);
}

void app_telemetry_sync(void){
    sync_telemetry(TM_IDLE);
}

// Every millisecond, from SysTick.
void app_systick(void){
    encoder_tick();
    buttons_tick();
    motion_tick_1ms();
    calib_tick_1ms();
}

// Idle LEDs: the boot sweep, then the mode's pair with a heartbeat; in the race menu LEDs 1..n blinking slowly.
static void show_mode(void){
    const uint32_t t = HAL_GetTick();
    if(!mode_chosen){
        leds_sweep_frame(t);
        return;
    }
    uint8_t mask, on;       // bit 5 = LED 1 ... bit 0 = LED 6
    if(race_menu){
        mask = (uint8_t)((0x3Fu << (6u - race)) & 0x3Fu);
        on = (t % 1000u) < 600u;
    }
    else{
        mask = (uint8_t)(3u << (6u - 2u * menu));
        on = (t % 500u) >= 50u;
    }
    leds_set_mask(on ? mask : 0u);
}

// Hands away: every LED blinking fast. 0 if START or STOP cancelled it.
static uint8_t countdown(uint32_t ms){
    const uint32_t start = HAL_GetTick();
    uint8_t go = 1;
    while(go && HAL_GetTick() - start < ms){
        leds_all((uint8_t)((((HAL_GetTick() - start) / COUNTDOWN_BLINK_MS) & 1u) ^ 1u));
        go = motion_wait(5);
    }
    leds_all(0);
    return go;
}

// The main loops say they are alive and report what the health checks and the clock security caught.
static void report_health(void){
    health_alive();
    uint32_t stall_ms, stall_pc, stall_lr;
    if(health_take_stall(&stall_ms, &stall_pc, &stall_lr)){
        print("!! the program stalled %lu ms at PC=0x%08lx LR=0x%08lx\n", (unsigned long)stall_ms,
              (unsigned long)stall_pc, (unsigned long)stall_lr);
    }
    if(sysclock_recover()){
        uart_retime();
        print("!! crystal failure: run aborted, internal clock at 64 MHz\n");
    }
}

// Erasing needs a second START within 3 s.
static void erase_map_confirmed(void){
    print("ERASE MAP: press START again within 3 s to confirm\n");
    uint32_t start = HAL_GetTick();
    while(HAL_GetTick() - start < 3000u){
        report_health();    // the wait is the UI's, not a stall
        commands_poll();
        leds_all((uint8_t)(((HAL_GetTick() - start) / 100u) & 1u));
        if(button_take_press(BUTTON_START)){
            maze_erase();
            leds_all(0);
            const uint8_t failed = app_save_now() == STORAGE_FAILED;
            print(failed ? "Map erased in RAM; !! error writing the flash\n" : "Map erased, default goal\n");
            if(failed) motion_indicate(IND_NOT_SAVED);
            sync_telemetry(TM_ERASE);
            return;
        }
        if(button_take_press(BUTTON_SELECT) || motion_abort_requested()) break;
    }
    leds_all(0);
    print("erase cancelled\n");
}

static void run_mode(uint8_t m){
    mode_chosen = 1;
    motion_clear_abort();
    buttons_clear();
    run_active = 1;
    if(m == MODE_ERASE){
        telemetry_activity(TM_ERASE);
        erase_map_confirmed();
    }
    else if((m == MODE_FAST || m == MODE_FAST_MID || m == MODE_FAST_SAFE || m == MODE_NO_CURVES)
            && search_fast_path_cost() == PLAN_INF){
        print("No verified start-goal path: run a search first (mode 1)\n");
        motion_indicate(IND_REFUSED);   // START did something: no path
    }
    else{
        print("Mode %s: starts in %u ms (START or STOP cancels)\n", m == MODE_REMOTE ? "REMOTE" : app_mode_label(),
              START_DELAY_MS);
        leds_all(1);
        uint32_t countdown_start = HAL_GetTick();
        sync_telemetry(TM_COUNTDOWN);   // the monitor starts the run with the full map
        uint32_t spent = HAL_GetTick() - countdown_start;
        uint8_t go = countdown(spent < START_DELAY_MS ? START_DELAY_MS - spent : 0);
        if(!go){
            print("cancelled\n");
        }
        else{
            run_result_t r = RUN_FAILED;
            motion_leg_timing_reset();
            switch(m){
                case MODE_SEARCH:       r = search_explore(); break;
                case MODE_FOLLOW_LEFT:  r = search_wall_follow(1); break;
                case MODE_FOLLOW_RIGHT: r = search_wall_follow(0); break;
                case MODE_NO_CURVES:    r = search_fast_run(0); break;
                case MODE_FAST_SAFE:
                case MODE_FAST_MID:
                case MODE_FAST:         r = search_fast_run(1); break;
#if DEV_TOOLS
                case MODE_REMOTE:       r = search_remote(console_decide, NULL); break;
#endif
                default: break;
            }
            motion_stop();
            motion_leg_timing_report();
            static const char *const RESULT[] = {"OK", "ABORTED", "FAILED"};
            print("End: %s | %s\n", RESULT[r],
                  search_ready() ? "robot at the start, ready" : "place the robot at the start");
        }
    }
    motion_stop();
    calib_run_finished();
    run_active = 0;
    motion_clear_abort();
    buttons_clear();
    uint8_t x, y;
    heading_t h;
    search_pose(&x, &y, &h);
    telemetry_activity(TM_IDLE);
    telemetry_pose(x, y, h);
}

// CAL tests and CHECK: a run as far as STOP and the console are concerned.
static void test_begin(void){
    motion_clear_abort();
    buttons_clear();
    run_active = 1;
    telemetry_activity(TM_CALIBRATE);
}

static void test_end(void){
    motion_stop();
    run_active = 0;
    motion_clear_abort();
    buttons_clear();
    telemetry_activity(TM_IDLE);
}

// CHECK: back facing north after four quarter turns, or lost.
static void run_check(void){
    test_begin();
    print("CHECK: the robot turns in %u ms (START or STOP cancels)\n", CAL_DELAY_MS);
    if(!countdown(CAL_DELAY_MS)) print("cancelled\n");
    else if(light_check() == LIGHT_ABORTED) search_set_lost();
    test_end();
}

#if DEV_TOOLS
static void run_calibration(void){
    test_begin();
    uint8_t go = 1;
    if(calib_moves(cal_test)){
        print("CAL: the robot moves in %u ms (START or STOP cancels)\n", CAL_DELAY_MS);
        go = countdown(CAL_DELAY_MS);
    }
    if(go){
        if(calib_moves(cal_test)) search_set_lost();
        calib_run(cal_test, cal_a, cal_b);
    }
    else{
        print("cancelled\n");
    }
    test_end();
}
#endif

static void print_banner(storage_status_t stored){
    uint8_t g[4];
    maze_get_goal(g);
    print("\nrat_sw %s %s | mode %s\n", __DATE__, __TIME__, app_mode_label());
#ifdef VIRTUAL_ROBOT
    print("VIRTUAL ROBOT: simulated 16x16 maze, not for competing\n");
#endif
    if(stored == STORAGE_LOADED || stored == STORAGE_NEW_DEFAULTS){
        uint16_t cost = search_fast_path_cost();
        if(cost == PLAN_INF){
            print("Map in flash: %u cells, no fast path (ERASE if it is another maze)\n", maze_visited_count());
        }
        else{
            print("Map in flash: %u cells, fast path cost %u (ERASE if it is another maze)\n",
                  maze_visited_count(), cost);
        }
        if(stored == STORAGE_NEW_DEFAULTS) print("Parameters: the firmware's new defaults (not the saved ones)\n");
    }
    else{
        print("Flash: %s\n", storage_status_name(stored));
    }
    if(sysclock_source() == CLOCK_HSI_BOOT) print("!! the crystal did not start: internal clock at 64 MHz\n");
    print("Reset: %s\n", health_reset_cause());
    print("Goal (%u,%u)-(%u,%u). SELECT changes mode, START launches it (on 2, opens the races)\n",
          g[0], g[1], g[2], g[3]);
}

storage_save_t app_save_now(void){
    const storage_save_t r = storage_save(0);
    if(r != STORAGE_FULL) return r;
    print("flash full: compacting (erases 2 pages, ~50 ms)\n");
    if(!storage_compact()) return STORAGE_FAILED;
    return storage_save(0);
}

// The boot compacts the store: the only erase, never during a run.
static void compact_store(void){
    if(storage_needs_compact() && !storage_compact()) print("!! flash: could not compact\n");
}

int main(void){
    health_init();
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
    compact_store();
    sync_telemetry(TM_IDLE);

    for(;;){
        report_health();
        commands_poll();
        if(button_take_press(BUTTON_SELECT)){
            if(!mode_chosen) app_set_mode(MENU_SEARCH, 0);
            else if(race_menu) app_set_mode(MENU_RACE, (uint8_t)(race % RACE_COUNT + 1));
            else app_set_mode((uint8_t)(menu % MENU_COUNT + 1), 0);
            print("mode %s\n", app_mode_label());
        }
        uint8_t pressed = button_take_press(BUTTON_START) && mode_chosen;    // no mode yet: ignored
        if(pressed && menu == MENU_RACE && !race_menu){
            race_menu = 1;      // only a reset leaves it
            print("races: SELECT picks, START launches | mode %s\n", app_mode_label());
            pressed = 0;
        }
        if(pressed) search_set_home();  // someone is at the robot: it stands at the start
        if(pressed || start_requested){
            start_requested = 0;
            run_mode(mode);
        }
        if(check_requested){
            check_requested = 0;
            run_check();
        }
#if DEV_TOOLS
        if(cal_requested){
            cal_requested = 0;
            run_calibration();
        }
        if(remote_requested){
            remote_requested = 0;
            run_mode(MODE_REMOTE);
        }
#endif
        show_mode();
    }
}
