// Virtual robot (env:virtual): the robot firmware (main.c, the console, the
// search, the races, the store on the real flash) on the real chip, with the
// motors never driven and motion.c replaced by the host tests' simulator
// (test/host/sim.c): every move is resolved, cell by cell, in a 16x16 maze
// built in, and takes its time for real. For testing the saves on the chip
// without a maze. Every run starts with the virtual robot at the start
// facing north (as if carried there).
#define SIM_ON_ROBOT
#include "../test/host/sim.c"

#include "stm32f1xx_hal.h"
#include "commands.h"
#include "gpio.h"
#include "health.h"
#include "uart.h"

#define VIRTUAL_SEED    2u      // the maze: truth_generate() + truth_competition_goal()
#define VIRTUAL_OPENINGS 30u    // walls knocked out to make loops

// ---- Run control (as motion.c) ---------------------------------------------------------

static volatile uint8_t abort_flag;
static volatile uint8_t paused;
static uint8_t step_mode;
static uint8_t moved;   // a move ran since the last checkpoint
static uint8_t maze_built;

void motion_request_abort(void){ abort_flag = 1; }
uint8_t motion_abort_requested(void){ return abort_flag; }
uint8_t motion_is_paused(void){ return paused; }
uint8_t motion_step_mode(void){ return step_mode; }
void motion_set_paused(uint8_t on){ paused = on; }

void motion_set_step_mode(uint8_t on){
    step_mode = on;
    if(!on) paused = 0;
}

// A new run (or its end): the virtual robot stands at the start.
void motion_clear_abort(void){
    abort_flag = 0;
    paused = 0;
    moved = 0;
    if(!maze_built){
        truth_generate(VIRTUAL_SEED, VIRTUAL_OPENINGS);
        truth_competition_goal(VIRTUAL_SEED);
        maze_built = 1;
    }
    sim_reset(0.0, 1);
}

static void poll_inputs(void){
    health_alive();
    commands_poll();
    if(button_take_press(BUTTON_START)) abort_flag = 1;
}

uint8_t motion_wait(uint32_t ms){
    const uint32_t start = HAL_GetTick();
    while(HAL_GetTick() - start < ms){
        poll_inputs();
        if(abort_flag) return 0;
    }
    return 1;
}

uint8_t motion_checkpoint(void){
    poll_inputs();
    if(step_mode && moved && !abort_flag){
        paused = 1;
        print("-- paso completado: RESUME para seguir --\n");
    }
    moved = 0;
    while(paused && !abort_flag) poll_inputs();
    return !abort_flag;
}

// The move's time, for real. STOP or START cut the wait, but the move ends
// as planned (in the virtual maze): the run stops at the next checkpoint.
void virtual_elapse(double seconds){
    moved = 1;
    motion_wait((uint32_t)(seconds * 1000.0));
}

void motion_indicate(indication_t what){
    switch(what){
        case IND_GOAL:
            leds_all(1);
            motion_wait(200);
            leds_all(0);
            break;
        case IND_DONE:
            leds_blink(3, 150);
            break;
        case IND_FAIL:
            leds_blink(3, 60);
            break;
    }
}

// ---- Robot only: nothing to drive -------------------------------------------------------

void motion_tick_1ms(void){}
void motion_stop(void){}
void motion_leg_timing_reset(void){}
void motion_leg_timing_report(void){}
float motion_ticks_per_mm(void){ return WHEEL_TICKS_PER_MM; }
void motion_curve_info(uint8_t line){ (void)line; }

void motion_reference(float *fwd_mm, float *rot_deg, uint8_t *move_id){
    *fwd_mm = 0.0f;
    *rot_deg = 0.0f;
    *move_id = 0;
}

void motion_tune_list(void){
    print("robot virtual: sin control que ajustar\n");
}

void motion_tune_set(const char *name, float value){
    (void)name;
    (void)value;
    motion_tune_list();
}

move_result_t motion_drive_straight(int16_t speed, int32_t mm){
    (void)speed;
    (void)mm;
    return MOVE_ABORTED;
}
