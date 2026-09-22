#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>
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
#include "maze.h"

#define     LANE_WIDTH              168.0 //mm
#define     MAX_SIDE_WALL_DIST      130.0 //mm
#define     TICKS_PER_CELL          1760 //ticks (was 1600; wall_correct data showed a consistent ~16-20mm FWD shortfall after ENC-stops)
#define     TICKS_PER_TURN          430 //ticks (was 490; measured turns landing at 100-105 deg, scaled down for ~90 deg)
#define     WALL_DETECT_MM          140 //mm, threshold to consider a wall present (was 120, too close to reliable detection range)
#define     MOVE_TIMEOUT_MS         10000 //safety net: never wait forever for encoder ticks that may never come (generous, to not cut off legit low-speed test runs)
#define     PD_STRAIGHT_MAX         150 //clamp: keep the lateral PD correction from swamping std_speed on a bad IR reading
#define     FRONT_WALL_REF_MM       94 //mm, expected front IR distance once correctly positioned at a wall (matches cross_cell's close-stop threshold)
#define     TICKS_PER_MM            9 //ticks/mm, from real measurement (~1424 ticks / 158mm)
#define     DRIFT_CORRECT_MAX_MM    30 //mm, safety clamp: skip the wall-based drift correction if the implied error looks unreliable rather than real drift
#define     DRIFT_CORRECT_SPEED     80 //gentle, precise nudge speed for the drift correction only
#define     FRONT_IR_MAX_DIFF_MM    30
#define     FRONT_STOP_MIN_TICKS    (TICKS_PER_CELL * 3 / 4)
#define     ENCODER_MAX_DIFF_TICKS  300
#define     ACCEL_STEP_PER_MS       4 //PWM units/ms allowed toward target - ramps the start instead of snapping straight to full speed (tune on hardware: higher = snappier, lower = softer)

robot_heading_t robot_heading = UP_DIR;
uint8_t robot_position_x = 0;
uint8_t robot_position_y = 0;

volatile int started = 0;
volatile float lateral_error = 0;
volatile float prev_lateral_error = 0;
volatile float kp = 2;
volatile float kd = 30;
volatile float pd_straight = 0;
int std_speed = 150; // raised from 80 now that basic moves are confirmed working
int turn_speed = 110; // TEMP: reduced for first real-motion tests, was 150 (60 was too low to move the wheels at all)
int boost = 0;

volatile uint8_t uart_start_requested = 0;
volatile uint8_t paused = 0;
volatile uint8_t debug_step_mode = 0;
static uint8_t localization_valid = 1;

// Live tuning/control over Bluetooth (one command per line):
// SPD n | TURN n | KP f | KD f | RESET | START | PAUSE | RESUME | DEBUG ON | DEBUG OFF
// Avoids a reflash cycle every time a parameter needs adjusting.

// Wait only where a state announcement must already be visible before the
// next action (especially reset); routine logs remain queued and non-blocking.
static void wait_uart_ready(){
    uint32_t deadline = HAL_GetTick() + 1500;
    while(!uart_tx_idle() && HAL_GetTick() < deadline);
}

// Parses a non-negative gain like "2" or "2.5" from the text right after a
// command keyword. sscanf's %f is a no-op on this toolchain's nano-libc (the
// same missing-float-support issue that also broke %f in print()), so this
// parses the integer and optional fractional part with %d instead.
static uint8_t parse_gain(const char *rest, float *out){
    while(*rest == ' ') rest++;
    if(!isdigit((unsigned char)*rest)) return 0;

    float value = 0.0f;
    while(isdigit((unsigned char)*rest)){
        value = value * 10.0f + (float)(*rest - '0');
        rest++;
    }
    if(*rest == '.'){
        float place = 0.1f;
        rest++;
        if(!isdigit((unsigned char)*rest)) return 0;
        while(isdigit((unsigned char)*rest)){
            value += (float)(*rest - '0') * place;
            place *= 0.1f;
            rest++;
        }
    }
    while(*rest == ' ') rest++;
    if(*rest != '\0') return 0;
    *out = value;
    return 1;
}

void process_uart_commands(){
    char line[32];
    if(!uart_read_line(line, sizeof(line))) return;

    for(int i = 0; line[i] != '\0'; i++){
        line[i] = (char)toupper((unsigned char)line[i]);
    }

    int ival;
    float fval;
    if(sscanf(line, "SPD %d", &ival) == 1 && ival >= 0 && ival <= 1000){
        std_speed = ival;
        print("std_speed=%d\n", std_speed);
    }
    else if(sscanf(line, "TURN %d", &ival) == 1 && ival >= 0 && ival <= 1000){
        turn_speed = ival;
        print("turn_speed=%d\n", turn_speed);
    }
    else if(strncmp(line, "KP ", 3) == 0 && parse_gain(line + 3, &fval) && fval <= 100.0f){
        kp = fval;
        print("kp=%d.%02d\n", (int)kp, (int)(kp*100)%100);
    }
    else if(strncmp(line, "KD ", 3) == 0 && parse_gain(line + 3, &fval) && fval <= 1000.0f){
        kd = fval;
        print("kd=%d.%02d\n", (int)kd, (int)(kd*100)%100);
    }
    else if(strcmp(line, "RESET") == 0){
        print("reiniciando micro...\n");
        wait_uart_ready();
        NVIC_SystemReset(); // full MCU reset: re-runs main() from scratch, like power-cycling
    }
    else if(strcmp(line, "START") == 0){
        if(started){
            print("run already active\n");
        }
        else if(!localization_valid){
            print("START rechazado: coloca el robot en el origen y manda RESET\n");
        }
        else{
            uart_start_requested = 1;
            print("start requested\n");
        }
    }
    else if(strcmp(line, "PAUSE") == 0){
        paused = 1;
        set_output(MOTOR_L, 0);
        set_output(MOTOR_R, 0);
        print("paused\n");
    }
    else if(strcmp(line, "RESUME") == 0){
        paused = 0;
        print("resumed\n");
    }
    else if(strcmp(line, "DEBUG ON") == 0){
        debug_step_mode = 1;
        print("debug step mode ON (pauses after every cell, send RESUME to continue)\n");
    }
    else if(strcmp(line, "DEBUG OFF") == 0){
        debug_step_mode = 0;
        paused = 0;
        print("debug step mode OFF\n");
    }
    else{
        print("? comandos: SPD n | TURN n | KP f | KD f | RESET | START | PAUSE | RESUME | DEBUG ON/OFF\n");
    }
}

static void honor_movement_pause(uint32_t *deadline){
    if(!paused) return;
    set_output(MOTOR_L, 0);
    set_output(MOTOR_R, 0);
    while(paused){
        process_uart_commands();
        HAL_Delay(10);
    }
    *deadline = HAL_GetTick() + MOVE_TIMEOUT_MS;
}


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

static int16_t ramp_speed_l = 0;
static int16_t ramp_speed_r = 0;
static uint32_t ramp_tick_l = 0;
static uint32_t ramp_tick_r = 0;

// Call once right before a movement's control loop starts, so the ramp
// begins from the real physical speed (0 - every movement starts from a
// settled stop) instead of continuing wherever a previous movement left off.
static void reset_speed_ramp(){
    ramp_speed_l = 0;
    ramp_speed_r = 0;
    ramp_tick_l = HAL_GetTick();
    ramp_tick_r = ramp_tick_l;
}

// Drives `motor` toward `target` at a bounded rate (ACCEL_STEP_PER_MS per
// elapsed ms) instead of jumping straight to it, so the robot eases out of a
// stop smoothly instead of snapping the wheels to full speed. Stopping still
// goes through set_output(motor, 0) directly for an immediate, predictable
// halt - ramping the stop down would risk overshooting the encoder/IR target.
static void set_output_ramped(motor_t motor, int16_t target){
    int16_t *speed = (motor == MOTOR_L) ? &ramp_speed_l : &ramp_speed_r;
    uint32_t *last_tick = (motor == MOTOR_L) ? &ramp_tick_l : &ramp_tick_r;

    uint32_t now = HAL_GetTick();
    int32_t max_step = ACCEL_STEP_PER_MS * (int32_t)(now - *last_tick);
    *last_tick = now;
    if(max_step < 1) max_step = 1;

    int32_t diff = (int32_t)target - (int32_t)*speed;
    if(diff > max_step) diff = max_step;
    else if(diff < -max_step) diff = -max_step;
    *speed = (int16_t)(*speed + diff);

    set_output(motor, *speed);
}

// Returns 1 if the cell was actually crossed (encoder or IR confirmed it),
// 0 on timeout. On 0 the robot's real position is unknown - the caller must
// NOT trust/update robot_position_x/y, or the maze map desyncs from reality
// and later decisions get made against a fictional position.
uint8_t cross_cell(){
    int encoder_check = 0;
    int ir_check = 0;
    int ir_close_count = 0;

    uint32_t initial_enc_l = get_encoder(ENCODER_L);
    uint32_t initial_enc_r = get_encoder(ENCODER_R);

    uint32_t deadline = HAL_GetTick() + MOVE_TIMEOUT_MS;
    reset_speed_ramp();
    while(!(encoder_check || ir_check) && HAL_GetTick() < deadline){
        process_uart_commands();
        honor_movement_pause(&deadline);

        set_output_ramped(MOTOR_L, std_speed + boost + pd_straight);
        set_output_ramped(MOTOR_R, std_speed + boost - pd_straight);

        // Use the average of both wheels (the robot's actual center displacement)
        // rather than whichever wheel gets there first - the two sides drift
        // apart under lateral PD correction, and stopping on the first one to
        // reach TICKS_PER_CELL was leaving the slower side (and so the real
        // center of the robot) consistently short of the target distance.
        int32_t diff_l = get_encoder_diff(initial_enc_l, get_encoder(ENCODER_L));
        int32_t diff_r = get_encoder_diff(initial_enc_r, get_encoder(ENCODER_R));
        int32_t diff_avg = (diff_l + diff_r) / 2;
        int32_t wheel_diff = diff_l - diff_r;
        if(wheel_diff < 0) wheel_diff = -wheel_diff;
        encoder_check = diff_avg >= TICKS_PER_CELL && wheel_diff <= ENCODER_MAX_DIFF_TICKS;

        // Ignore the front-wall stop until real ground is covered, and require
        // 3 consecutive close readings: a single noisy IR sample right after
        // starting could otherwise false-trigger and stop within a few mm.
          if(diff_avg >= FRONT_STOP_MIN_TICKS &&
              get_ir_mm(IR_FL) < FRONT_WALL_REF_MM &&
              get_ir_mm(IR_FR) < FRONT_WALL_REF_MM){
            ir_close_count++;
        } else {
            ir_close_count = 0;
        }
        ir_check = ir_close_count >= 3;
    }
    set_output(MOTOR_L, 0);
    set_output(MOTOR_R, 0);

    int32_t final_l = get_encoder_diff(initial_enc_l, get_encoder(ENCODER_L));
    int32_t final_r = get_encoder_diff(initial_enc_r, get_encoder(ENCODER_R));
    uint8_t confirmed = encoder_check || ir_check;
    print("cross_cell: L=%ld R=%ld obj=%d stop=%s IR(FL=%d FR=%d SL=%d SR=%d)\n",
          (long)final_l, (long)final_r, TICKS_PER_CELL,
          ir_check ? "IR" : (confirmed ? "ENC" : "TIMEOUT"),
          (int)get_ir_mm(IR_FL), (int)get_ir_mm(IR_FR), (int)get_ir_mm(IR_SL), (int)get_ir_mm(IR_SR));

    if(!confirmed) return 0;

    int next_x = robot_position_x;
    int next_y = robot_position_y;
    switch (robot_heading) {
        case UP_DIR:    next_y++; break;
        case DOWN_DIR:  next_y--; break;
        case RIGHT_DIR: next_x++; break;
        case LEFT_DIR:  next_x--; break;
    }
    if(next_x < 0 || next_x >= MAZE_SIZE || next_y < 0 || next_y >= MAZE_SIZE) return 0;
    robot_position_x = (uint8_t)next_x;
    robot_position_y = (uint8_t)next_y;
    return 1;
}

// If there's a front wall in range, nudge forward/back so the resulting front
// IR distance matches FRONT_WALL_REF_MM. Uses the wall as an absolute
// reference to cancel small drift (wheel slip, encoder calibration error)
// that cross_cell()'s tick count alone can't detect. No-ops if there's no
// wall to reference, or if the implied error is implausibly large (unreliable
// reading rather than real drift).
void correct_against_front_wall(){
    float fl = get_ir_mm(IR_FL);
    float fr = get_ir_mm(IR_FR);
    if(fl >= WALL_DETECT_MM || fr >= WALL_DETECT_MM) return;
    int front_diff = (int)fl - (int)fr;
    if(front_diff > FRONT_IR_MAX_DIFF_MM || front_diff < -FRONT_IR_MAX_DIFF_MM) return;

    int error_mm = (int)((fl + fr) / 2.0f) - FRONT_WALL_REF_MM;
    if(error_mm == 0 || error_mm > DRIFT_CORRECT_MAX_MM || error_mm < -DRIFT_CORRECT_MAX_MM) return;

    int8_t forward = error_mm > 0; // farther than expected -> drive forward to close the gap
    int32_t target_ticks = (forward ? error_mm : -error_mm) * TICKS_PER_MM;
    int16_t speed = forward ? DRIFT_CORRECT_SPEED : -DRIFT_CORRECT_SPEED;

    uint32_t initial_enc_l = get_encoder(ENCODER_L);
    uint32_t initial_enc_r = get_encoder(ENCODER_R);
    uint32_t deadline = HAL_GetTick() + 1000; // short and bounded - a small nudge, not a full move
    int32_t diff_l = 0, diff_r = 0;
    int done = 0;
    reset_speed_ramp();
    while(!done && HAL_GetTick() < deadline){
        process_uart_commands();
        honor_movement_pause(&deadline);

        set_output_ramped(MOTOR_L, speed);
        set_output_ramped(MOTOR_R, speed);
        diff_l = get_encoder_diff(initial_enc_l, get_encoder(ENCODER_L));
        diff_r = get_encoder_diff(initial_enc_r, get_encoder(ENCODER_R));
        int32_t abs_l = diff_l < 0 ? -diff_l : diff_l;
        int32_t abs_r = diff_r < 0 ? -diff_r : diff_r;
        done = abs_l >= target_ticks || abs_r >= target_ticks;
    }
    set_output(MOTOR_L, 0);
    set_output(MOTOR_R, 0);

    print("wall_correct: err=%dmm dir=%s L=%ld R=%ld\n",
          error_mm, forward ? "FWD" : "BACK", (long)diff_l, (long)diff_r);
}

// Majority-of-5 debounce per sensor, spread over a wider time window, so a
// short transient reflection/noise blip can't outvote the real steady-state
// reading (was majority-of-3, still too easily swayed by a brief blip).
static uint8_t ir_confirms_wall(ir_sensor_t ir){
    int close_count = 0;
    for(int i = 0; i < 5; i++){
        if(get_ir_mm(ir) < WALL_DETECT_MM) close_count++;
        HAL_Delay(8);
    }
    return close_count >= 4;
}

// BUTTON_START has no pull resistor (GPIO_NOPULL) and can read spurious
// presses from motor PWM electrical noise; require it held across several
// samples before treating it as a real abort request.
static uint8_t abort_requested(){
    for(int i = 0; i < 5; i++){
        if(!get_button(BUTTON_START)) return 0;
        HAL_Delay(5);
    }
    return 1;
}

void sense_walls_here(){
    // Let residual chassis vibration/rocking from the stop settle before
    // trusting the IR readings - sampling right as the motors cut out was a
    // source of the transient close reads that fed false walls into the map.
    HAL_Delay(30);

    uint8_t front = ir_confirms_wall(IR_FL) && ir_confirms_wall(IR_FR);
    uint8_t left  = ir_confirms_wall(IR_SL);
    uint8_t right = ir_confirms_wall(IR_SR);

    maze_observe_wall(robot_position_x, robot_position_y, robot_heading, front);
    maze_observe_wall(robot_position_x, robot_position_y, maze_left_of(robot_heading), left);
    maze_observe_wall(robot_position_x, robot_position_y, maze_right_of(robot_heading), right);

    maze_visited[robot_position_x][robot_position_y] = 1;
}

// Returns 1 if the turn was confirmed by the encoders, 0 on timeout (see
// cross_cell(): heading must not be updated on an unconfirmed turn).
uint8_t turn_left(){
    int encoder_check = 0;

    uint32_t initial_enc_l = get_encoder(ENCODER_L);
    uint32_t initial_enc_r = get_encoder(ENCODER_R);

    set_output(MOTOR_L, 0);
    set_output(MOTOR_R, 0);
    HAL_Delay(150);

    uint32_t deadline = HAL_GetTick() + MOVE_TIMEOUT_MS;
    reset_speed_ramp();
    while(!encoder_check && HAL_GetTick() < deadline){
        process_uart_commands();
        honor_movement_pause(&deadline);

        set_output_ramped(MOTOR_L, -turn_speed);
        set_output_ramped(MOTOR_R, turn_speed);

        int32_t diff_l = get_encoder_diff(initial_enc_l, get_encoder(ENCODER_L));
        int32_t diff_r = get_encoder_diff(initial_enc_r, get_encoder(ENCODER_R));
        encoder_check = (-diff_l + diff_r) / 2 >= TICKS_PER_TURN;
    }

    set_output(MOTOR_L, 0);
    set_output(MOTOR_R, 0);
    HAL_Delay(150);

    int32_t final_l = get_encoder_diff(initial_enc_l, get_encoder(ENCODER_L));
    int32_t final_r = get_encoder_diff(initial_enc_r, get_encoder(ENCODER_R));
    uint8_t confirmed = encoder_check;
    print("turn_left: L=%ld R=%ld obj=%d stop=%s IR(FL=%d FR=%d SL=%d SR=%d)\n",
          (long)final_l, (long)final_r, TICKS_PER_TURN,
          confirmed ? "ENC" : "TIMEOUT",
          (int)get_ir_mm(IR_FL), (int)get_ir_mm(IR_FR), (int)get_ir_mm(IR_SL), (int)get_ir_mm(IR_SR));

    if(!confirmed) return 0;

    // Update robot heading
    switch (robot_heading) {
        case UP_DIR:    robot_heading = LEFT_DIR; break;
        case DOWN_DIR:  robot_heading = RIGHT_DIR; break;
        case RIGHT_DIR: robot_heading = UP_DIR; break;
        case LEFT_DIR:  robot_heading = DOWN_DIR; break;
    }
    return 1;
}

// Returns 1 if the turn was confirmed by the encoders, 0 on timeout (see
// cross_cell(): heading must not be updated on an unconfirmed turn).
uint8_t turn_right(){
    int encoder_check = 0;

    uint32_t initial_enc_l = get_encoder(ENCODER_L);
    uint32_t initial_enc_r = get_encoder(ENCODER_R);

    set_output(MOTOR_L, 0);
    set_output(MOTOR_R, 0);
    HAL_Delay(150);

    uint32_t deadline = HAL_GetTick() + MOVE_TIMEOUT_MS;
    reset_speed_ramp();
    while(!encoder_check && HAL_GetTick() < deadline){
        process_uart_commands();
        honor_movement_pause(&deadline);

        set_output_ramped(MOTOR_L, turn_speed);
        set_output_ramped(MOTOR_R, -turn_speed);

        int32_t diff_l = get_encoder_diff(initial_enc_l, get_encoder(ENCODER_L));
        int32_t diff_r = get_encoder_diff(initial_enc_r, get_encoder(ENCODER_R));
        encoder_check = (diff_l - diff_r) / 2 >= TICKS_PER_TURN;
    }

    set_output(MOTOR_L, 0);
    set_output(MOTOR_R, 0);
    HAL_Delay(150);

    int32_t final_l = get_encoder_diff(initial_enc_l, get_encoder(ENCODER_L));
    int32_t final_r = get_encoder_diff(initial_enc_r, get_encoder(ENCODER_R));
    uint8_t confirmed = encoder_check;
    print("turn_right: L=%ld R=%ld obj=%d stop=%s IR(FL=%d FR=%d SL=%d SR=%d)\n",
          (long)final_l, (long)final_r, TICKS_PER_TURN,
          confirmed ? "ENC" : "TIMEOUT",
          (int)get_ir_mm(IR_FL), (int)get_ir_mm(IR_FR), (int)get_ir_mm(IR_SL), (int)get_ir_mm(IR_SR));

    if(!confirmed) return 0;

    // Update robot heading
    switch (robot_heading) {
        case UP_DIR:    robot_heading = RIGHT_DIR; break;
        case DOWN_DIR:  robot_heading = LEFT_DIR; break;
        case RIGHT_DIR: robot_heading = DOWN_DIR; break;
        case LEFT_DIR:  robot_heading = UP_DIR; break;
    }
    return 1;
}

void run_left_side(){
    robot_position_x = 0;
    robot_position_y = 0;
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
    robot_position_x = 0;
    robot_position_y = 0;
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

// Explore + map the maze using flood fill: sense walls at the current cell,
// recompute flood, turn towards the lowest-flood open neighbor, cross into
// it, repeat until the goal is reached (or START aborts the run).
void explore_run(){
    robot_position_x = 0;
    robot_position_y = 0;
    robot_heading = UP_DIR;

    while(!maze_is_goal(robot_position_x, robot_position_y) && !abort_requested()){
        process_uart_commands();
        if(paused){
            set_output(MOTOR_L, 0);
            set_output(MOTOR_R, 0);
            HAL_Delay(10);
            continue;
        }

        sense_walls_here();
        maze_compute_flood();
        robot_heading_t next = maze_next_move(robot_position_x, robot_position_y, robot_heading);

        print("pos=(%d,%d) heading=%d flood=%u -> next=%d\n",
              robot_position_x, robot_position_y, robot_heading,
              maze_flood[robot_position_x][robot_position_y], next);

        if(maze_flood[robot_position_x][robot_position_y] == 0xFFFF){
            // Goal unreachable per the known walls - continuing would just bounce
            // between cells forever (a real dead end, or more likely a corrupted
            // map from a previous aborted/mispositioned run). Stop instead of
            // wasting time/battery looping with no possible progress.
            wait_uart_ready();
            print("!! ABORT: meta inalcanzable segun el mapa conocido (mapa corrupto o sin salida) !!\n");
            print("!! coloca el robot en la celda de salida y manda RESET antes de reintentar !!\n");
            fast_blink();
            localization_valid = 0;
            break;
        }

        uint8_t move_ok = 1;
        if(next == maze_left_of(robot_heading)){
            move_ok = turn_left();
        }
        else if(next == maze_right_of(robot_heading)){
            move_ok = turn_right();
        }
        else if(next != robot_heading){
            move_ok = turn_right() && turn_right();
        }
        if(move_ok) move_ok = cross_cell();
        if(move_ok) correct_against_front_wall();

        if(!move_ok){
            // Turn/cross timed out: real position/heading is now unknown, so the
            // map can't be trusted anymore. Stop instead of continuing to explore
            // blind - that desync is how it used to end up ramming a wall it
            // thought (incorrectly) was an open passage.
            wait_uart_ready();
            print("!! ABORT: movimiento no confirmado (timeout), posicion ya no es fiable !!\n");
            print("!! coloca el robot en la celda de salida y manda RESET antes de reintentar (START solo no limpia el mapa) !!\n");
            fast_blink();
            localization_valid = 0;
            break;
        }

        if(debug_step_mode){
            paused = 1;
            wait_uart_ready();
            print("-- debug: celda completada, manda RESUME para continuar --\n");
        }
    }
    set_output(MOTOR_L, 0);
    set_output(MOTOR_R, 0);

    if(maze_is_goal(robot_position_x, robot_position_y)){
        wait_uart_ready();
        print("Meta alcanzada en (%d,%d)!\n", robot_position_x, robot_position_y);
        set_all_led(LED_ON);
    }
    localization_valid = 0;
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
    maze_init();
#if PRACTICE_MAZE
    maze_set_goal(3, 2, 3, 2);
#endif
    uart_start_receive();

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
        process_uart_commands();
        if((get_button(BUTTON_START) || uart_start_requested) && localization_valid){
            uart_start_requested = 0;
            set_all_led(LED_ON);
            HAL_Delay(2000);
            set_all_led(LED_OFF);
            started = 1;
            HAL_Delay(20);
            explore_run();
            started = 0;
        }
        else if(get_button(BUTTON_SELECT)){
            set_all_led(LED_ON);
            HAL_Delay(2000);
            set_all_led(LED_OFF);
            started = 1;
            HAL_Delay(20);
            run_right_side();
            started = 0;
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
            if(pd_straight > PD_STRAIGHT_MAX) pd_straight = PD_STRAIGHT_MAX;
            if(pd_straight < -PD_STRAIGHT_MAX) pd_straight = -PD_STRAIGHT_MAX;
            prev_lateral_error = lateral_error; 
            break;

        case 9:
            task_tick = 0;
            break;
    }
    HAL_IncTick();
    HAL_SYSTICK_IRQHandler();
}
