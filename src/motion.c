#include "motion.h"
#include "stm32f1xx_hal.h"
#include "commands.h"
#include "encoder.h"
#include "gpio.h"
#include "infrared.h"
#include "motor.h"
#include "params.h"
#include "robot_config.h"
#include "uart.h"

// Every movement is a loop paced at exactly 1 kHz. Positions come from the
// 32-bit encoder odometry, steering from the 100 Hz controller in SysTick.

static int32_t abs32(int32_t v){
    return v < 0 ? -v : v;
}

static void motors_off(void){
    motor_set(MOTOR_L, 0);
    motor_set(MOTOR_R, 0);
}

static void wait_next_ms(void){
    uint32_t now = HAL_GetTick();
    while(HAL_GetTick() == now){}
}

// ---- Run control -----------------------------------------------------------------

static volatile uint8_t abort_flag;
static volatile uint8_t paused;
static uint8_t step_mode;
static uint8_t moved;   // an action ran since the last checkpoint

void motion_request_abort(void){ abort_flag = 1; }
void motion_clear_abort(void){ abort_flag = 0; paused = 0; }
uint8_t motion_abort_requested(void){ return abort_flag; }
uint8_t motion_is_paused(void){ return paused; }
uint8_t motion_step_mode(void){ return step_mode; }

void motion_set_paused(uint8_t on){
    paused = on;
    if(on) motors_off();
}

void motion_set_step_mode(uint8_t on){
    step_mode = on;
    if(!on) paused = 0;
}

// Commands and the START button (= stop the run) are serviced from every
// wait and control loop, so the robot reacts within a millisecond.
static void poll_inputs(void){
    commands_poll();
    if(button_take_press(BUTTON_START)) abort_flag = 1;
}

uint8_t motion_wait(uint32_t ms){
    uint32_t start = HAL_GetTick();
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

// ---- Acceleration ramp ------------------------------------------------------------

static int16_t ramp_pwm[2];
static uint32_t ramp_ms[2];

static void ramp_reset(void){
    ramp_pwm[MOTOR_L] = 0;
    ramp_pwm[MOTOR_R] = 0;
    ramp_ms[MOTOR_L] = ramp_ms[MOTOR_R] = HAL_GetTick();
}

// Moves the duty towards `target` by at most ACCEL_STEP_PER_MS per elapsed
// ms, so the wheels never snap from standstill (and slip). Stops bypass the
// ramp on purpose: the calibrated stopping points assume a hard brake.
static void drive_ramped(motor_t motor, int16_t target){
    uint32_t now = HAL_GetTick();
    int32_t max_step = ACCEL_STEP_PER_MS * (int32_t)(now - ramp_ms[motor]);
    ramp_ms[motor] = now;
    if(max_step < 1) max_step = 1;
    int32_t diff = (int32_t)target - ramp_pwm[motor];
    if(diff > max_step) diff = max_step;
    else if(diff < -max_step) diff = -max_step;
    ramp_pwm[motor] = (int16_t)(ramp_pwm[motor] + diff);
    motor_set(motor, ramp_pwm[motor]);
}

// ---- Steering (100 Hz, SysTick) ---------------------------------------------------------

typedef enum { REF_NONE, REF_RIGHT, REF_LEFT, REF_RESTART } steer_ref_t;

static volatile uint8_t steer_on;
static volatile float steer_out;
static volatile steer_ref_t steer_prev_ref;
static float steer_prev_error;
static int32_t steer_heading_ref;
static uint8_t steer_divider;

static void steer_start(void){
    steer_prev_ref = REF_RESTART;
    steer_out = 0.0f;
    steer_on = 1;
}

static void steer_stop(void){
    steer_on = 0;       // from now on SysTick leaves the LEDs alone
    steer_out = 0.0f;
    leds_set_mask(0);
}

// PD on the distance to one side wall (the right one first, as calibrated).
// With no wall in range it holds the heading from the encoder difference
// instead (gain KE; 0 = coast straight as before).
static void steer_update(void){
    if(!steer_on) return;
    steer_ref_t ref = REF_NONE;
    float error = 0.0f;
    uint8_t leds = 0x00;
    float d = ir_mm(IR_SR);
    if(d < SIDE_WALL_TRACK_MM){
        ref = REF_RIGHT;
        error = d - LANE_WIDTH_MM / 2.0f;
        leds = 0x07;
    }
    else{
        d = ir_mm(IR_SL);
        if(d < SIDE_WALL_TRACK_MM){
            ref = REF_LEFT;
            error = LANE_WIDTH_MM / 2.0f - d;
            leds = 0x38;
        }
    }

    float out;
    if(ref != REF_NONE){
        // A new reference (start of a move, or the other wall) must not
        // produce a derivative kick.
        if(ref != steer_prev_ref) steer_prev_error = error;
        out = params.kp * error + params.kd * (error - steer_prev_error);
        steer_prev_error = error;
    }
    else{
        int32_t twist = encoder_total(ENCODER_L) - encoder_total(ENCODER_R);
        if(steer_prev_ref != REF_NONE) steer_heading_ref = twist;  // walls just lost: hold this heading
        out = -params.ke * (float)(twist - steer_heading_ref);
    }
    steer_prev_ref = ref;

    if(out > PD_STRAIGHT_MAX) out = PD_STRAIGHT_MAX;
    else if(out < -PD_STRAIGHT_MAX) out = -PD_STRAIGHT_MAX;
    steer_out = out;
    leds_set_mask(leds);
}

void motion_tick_1ms(void){
    if(++steer_divider >= 10){
        steer_divider = 0;
        steer_update();
    }
}

// ---- Move supervision ----------------------------------------------------------------------

typedef struct {
    int32_t l0, r0;     // encoder totals at the start
    int32_t dl, dr;     // travel since then
} odo_t;

static void odo_start(odo_t *o){
    o->l0 = encoder_total(ENCODER_L);
    o->r0 = encoder_total(ENCODER_R);
    o->dl = 0;
    o->dr = 0;
}

static void odo_update(odo_t *o){
    o->dl = encoder_total(ENCODER_L) - o->l0;
    o->dr = encoder_total(ENCODER_R) - o->r0;
}

static int32_t odo_travel(const odo_t *o){
    return abs32(o->dl) + abs32(o->dr);
}

typedef struct {
    uint32_t timeout_ms, deadline, progress_ms;
    int32_t progress;
} guard_t;

static void guard_arm(guard_t *g, int32_t travel){
    uint32_t now = HAL_GetTick();
    g->deadline = now + g->timeout_ms;
    g->progress_ms = now;
    g->progress = travel;
}

static void guard_start(guard_t *g, uint32_t timeout_ms){
    g->timeout_ms = timeout_ms;
    guard_arm(g, 0);
}

// Once per control step: commands, pause, abort, stall and timeout.
static move_result_t guard_check(guard_t *g, int32_t travel){
    poll_inputs();
    if(paused && !abort_flag){
        motors_off();
        while(paused && !abort_flag) poll_inputs();
        ramp_reset();           // resume smoothly from standstill
        guard_arm(g, travel);
    }
    if(abort_flag) return MOVE_ABORTED;
    uint32_t now = HAL_GetTick();
    if(travel - g->progress >= STALL_MIN_TICKS){
        g->progress = travel;
        g->progress_ms = now;
    }
    else if(now - g->progress_ms > STALL_TIMEOUT_MS){
        return MOVE_STALLED;
    }
    if((int32_t)(now - g->deadline) >= 0) return MOVE_TIMEOUT;
    return MOVE_OK;
}

// Extra duty while the wheels have not started moving (static friction).
static int16_t breakaway(const guard_t *g, int16_t *max_used){
    uint32_t idle = HAL_GetTick() - g->progress_ms;
    int32_t boost = idle > BREAKAWAY_DELAY_MS ? (int32_t)(idle - BREAKAWAY_DELAY_MS) / BREAKAWAY_MS_PER_PWM : 0;
    if(boost > BREAKAWAY_MAX_PWM) boost = BREAKAWAY_MAX_PWM;
    if(boost > *max_used) *max_used = (int16_t)boost;
    return (int16_t)boost;
}

static void print_boost(int16_t boost){
    if(boost > 0) print("  arranque dificil: hizo falta +%d PWM\n", boost);
}

// ---- Straight moves ---------------------------------------------------------------------------

// Cruise, then brake linearly over FAST_DECEL_TICKS so that the last
// FAST_APPROACH_TICKS run at the search speed, where stops are calibrated.
static int16_t profile_speed(int32_t remaining, int16_t cruise, int16_t final_speed){
    if(cruise <= final_speed || remaining <= FAST_APPROACH_TICKS) return final_speed;
    int32_t over = remaining - FAST_APPROACH_TICKS;
    if(over >= FAST_DECEL_TICKS) return cruise;
    return (int16_t)(final_speed + (int32_t)(cruise - final_speed) * over / FAST_DECEL_TICKS);
}

// After an early obstacle stop: reverse to where the move started, which is
// the center of the cell the robot never left.
static move_result_t back_up(odo_t *move){
    odo_t own;
    guard_t g;
    move_result_t result = MOVE_BLOCKED;
    int16_t max_boost = 0;
    odo_start(&own);
    guard_start(&g, 2 * DRIFT_CORRECT_TIMEOUT_MS);
    ramp_reset();
    for(;;){
        wait_next_ms();
        odo_update(move);
        odo_update(&own);
        if(move->dl + move->dr <= 0) break;
        move_result_t check = guard_check(&g, odo_travel(&own));
        if(check != MOVE_OK){
            result = check == MOVE_ABORTED ? MOVE_ABORTED : MOVE_LOST;
            break;
        }
        int16_t duty = (int16_t)(DRIFT_CORRECT_SPEED + breakaway(&g, &max_boost));
        drive_ramped(MOTOR_L, (int16_t)-duty);
        drive_ramped(MOTOR_R, (int16_t)-duty);
    }
    motors_off();
    print("obstaculo delante: marcha atras %s (L=%ld R=%ld)\n",
          result == MOVE_BLOCKED ? "OK" : move_result_name(result), (long)move->dl, (long)move->dr);
    print_boost(max_boost);
    return result;
}

move_result_t motion_forward(uint8_t cells, int16_t cruise_speed){
    if(!cells) return MOVE_OK;
    moved = 1;
    const int32_t target = TICKS_FOR_CELLS(cells);
    const int16_t final_speed = params.search_speed;
    const int32_t max_wheel_diff = ENCODER_MAX_DIFF_TICKS + (int32_t)(cells - 1) * ENCODER_MAX_DIFF_PER_CELL;

    odo_t o;
    guard_t g;
    uint8_t ir_close = 0, ir_emergency = 0;
    int16_t max_boost = 0;
    const char *stop = "ENC";
    move_result_t result;
    odo_start(&o);
    guard_start(&g, MOVE_TIMEOUT_BASE_MS + (uint32_t)cells * MOVE_TIMEOUT_PER_CELL_MS);
    ramp_reset();
    steer_start();
    for(;;){
        wait_next_ms();
        odo_update(&o);
        result = guard_check(&g, odo_travel(&o));
        if(result != MOVE_OK){
            stop = move_result_name(result);
            break;
        }

        // Robot center = average of both wheels: under steering one wheel
        // runs ahead, and stopping on it left the robot short.
        int32_t traveled = (o.dl + o.dr) / 2;
        int32_t remaining = target - traveled;
        if(remaining <= 0){
            result = abs32(o.dl - o.dr) <= max_wheel_diff ? MOVE_OK : MOVE_SLIPPED;
            if(result != MOVE_OK) stop = move_result_name(result);
            break;
        }

        float fl = ir_mm(IR_FL);
        float fr = ir_mm(IR_FR);
        if(remaining <= FRONT_STOP_ZONE_TICKS){
            // Final approach: a front wall is the best position reference.
            ir_close = (fl < FRONT_WALL_REF_MM && fr < FRONT_WALL_REF_MM) ? (uint8_t)(ir_close + 1) : 0;
            if(ir_close >= FRONT_STOP_CONFIRM_MS){
                stop = "IR";
                break;
            }
        }
        else{
            // Something this close before the final approach was not in the
            // plan: stop before touching it.
            ir_emergency = (fl < FRONT_EMERGENCY_MM && fr < FRONT_EMERGENCY_MM) ? (uint8_t)(ir_emergency + 1) : 0;
            if(ir_emergency >= FRONT_STOP_CONFIRM_MS){
                result = traveled < CELL_TICKS / 2 ? MOVE_BLOCKED : MOVE_LOST;
                stop = "OBSTACULO";
                break;
            }
        }

        float speed = (float)(profile_speed(remaining, cruise_speed, final_speed) + breakaway(&g, &max_boost));
        float steer = steer_out;
        drive_ramped(MOTOR_L, (int16_t)(speed + steer));
        drive_ramped(MOTOR_R, (int16_t)(speed - steer));
    }
    motors_off();
    steer_stop();
    odo_update(&o);

    if(params.log_level >= 2){
        print("avance %u: L=%ld R=%ld obj=%ld fin=%s IR(FL=%d FR=%d SL=%d SR=%d)\n",
              cells, (long)o.dl, (long)o.dr, (long)target, stop,
              (int)ir_mm(IR_FL), (int)ir_mm(IR_FR), (int)ir_mm(IR_SL), (int)ir_mm(IR_SR));
    }
    print_boost(max_boost);
    if(result == MOVE_BLOCKED) result = back_up(&o);
    return result;
}

move_result_t motion_drive_straight(int16_t pwm, int32_t ticks){
    odo_t o;
    guard_t g;
    move_result_t result = MOVE_OK;
    int16_t max_boost = 0;
    moved = 1;
    odo_start(&o);
    guard_start(&g, MOVE_TIMEOUT_BASE_MS);
    ramp_reset();
    for(;;){
        wait_next_ms();
        odo_update(&o);
        if(abs32((o.dl + o.dr) / 2) >= ticks) break;
        result = guard_check(&g, odo_travel(&o));
        if(result != MOVE_OK) break;
        int16_t boost = breakaway(&g, &max_boost);
        int16_t duty = (int16_t)(pwm >= 0 ? pwm + boost : pwm - boost);
        drive_ramped(MOTOR_L, duty);
        drive_ramped(MOTOR_R, duty);
    }
    motors_off();
    print_boost(max_boost);
    return result;
}

void motion_align_front(void){
    float fl = ir_mm(IR_FL);
    float fr = ir_mm(IR_FR);
    if(fl >= WALL_DETECT_MM || fr >= WALL_DETECT_MM) return;
    if(abs32((int32_t)fl - (int32_t)fr) > FRONT_IR_MAX_DIFF_MM) return;
    int32_t error_mm = (int32_t)((fl + fr) / 2.0f) - FRONT_WALL_REF_MM;
    if(error_mm == 0 || abs32(error_mm) > DRIFT_CORRECT_MAX_MM) return;

    // Farther than expected: forward. Closer: back.
    const int32_t target = abs32(error_mm) * TICKS_PER_MM;
    const int16_t sign = error_mm > 0 ? 1 : -1;
    odo_t o;
    guard_t g;
    move_result_t result = MOVE_OK;
    int16_t max_boost = 0;
    odo_start(&o);
    guard_start(&g, DRIFT_CORRECT_TIMEOUT_MS);
    ramp_reset();
    for(;;){
        wait_next_ms();
        odo_update(&o);
        if(abs32(o.dl) >= target || abs32(o.dr) >= target) break;
        result = guard_check(&g, odo_travel(&o));
        if(result != MOVE_OK) break;
        int16_t duty = (int16_t)(sign * (DRIFT_CORRECT_SPEED + breakaway(&g, &max_boost)));
        drive_ramped(MOTOR_L, duty);
        drive_ramped(MOTOR_R, duty);
    }
    motors_off();
    if(params.log_level >= 2){
        print("alineado frontal: err=%ldmm L=%ld R=%ld%s%s\n", (long)error_mm, (long)o.dl, (long)o.dr,
              result == MOVE_OK ? "" : " ", result == MOVE_OK ? "" : move_result_name(result));
        print_boost(max_boost);
    }
}

// ---- Turns -------------------------------------------------------------------------------------

static move_result_t turn_quarter(int8_t dir){
    odo_t o;
    guard_t g;
    move_result_t result;
    int16_t max_boost = 0;
    const int16_t speed = params.turn_speed;
    odo_start(&o);
    guard_start(&g, MOVE_TIMEOUT_BASE_MS);
    ramp_reset();
    for(;;){
        wait_next_ms();
        odo_update(&o);
        result = guard_check(&g, odo_travel(&o));
        if(result != MOVE_OK) break;
        // Half the wheel difference, exactly as TICKS_PER_TURN was calibrated.
        if(dir * (o.dl - o.dr) / 2 >= TICKS_PER_TURN) break;
        int16_t duty = (int16_t)(speed + breakaway(&g, &max_boost));
        drive_ramped(MOTOR_L, (int16_t)(dir * duty));
        drive_ramped(MOTOR_R, (int16_t)(-dir * duty));
    }
    motors_off();
    // The turn calibration includes this pause before the next move.
    if(result == MOVE_OK && !motion_wait(TURN_SETTLE_MS)) result = MOVE_ABORTED;
    odo_update(&o);
    if(params.log_level >= 2){
        print("giro %s: L=%ld R=%ld obj=%d fin=%s\n", dir > 0 ? "der" : "izq",
              (long)o.dl, (long)o.dr, TICKS_PER_TURN, result == MOVE_OK ? "ENC" : move_result_name(result));
    }
    print_boost(max_boost);
    return result;
}

move_result_t motion_turn(int8_t quarter_turns){
    int8_t dir = quarter_turns > 0 ? 1 : -1;
    uint8_t count = (uint8_t)(quarter_turns > 0 ? quarter_turns : -quarter_turns);
    moved = 1;
    for(uint8_t i = 0; i < count; i++){
        move_result_t r = turn_quarter(dir);
        if(r != MOVE_OK) return r;
    }
    return MOVE_OK;
}

// ---- Sensing and signalling ----------------------------------------------------------------------

move_result_t motion_sense_walls(wall_sense_t *out){
    // Let the chassis stop rocking first: sampling right at the stop was a
    // source of phantom walls.
    if(!motion_wait(SENSE_SETTLE_MS)) return MOVE_ABORTED;
    uint8_t votes[IR_COUNT] = {0};
    float sum[IR_COUNT] = {0.0f};
    for(uint8_t i = 0; i < WALL_SAMPLES; i++){
        if(i && !motion_wait(WALL_SAMPLE_MS)) return MOVE_ABORTED;
        for(uint8_t s = 0; s < IR_COUNT; s++){
            float mm = ir_mm((ir_sensor_t)s);
            sum[s] += mm;
            if(mm < WALL_DETECT_MM) votes[s]++;
        }
    }
    uint8_t fl_seen = votes[IR_FL] >= WALL_VOTES, fr_seen = votes[IR_FR] >= WALL_VOTES;
    out->front = fl_seen && fr_seen ? SEEN_PRESENT : SEEN_ABSENT;
    out->left = votes[IR_SL] >= WALL_VOTES ? SEEN_PRESENT : SEEN_ABSENT;
    out->right = votes[IR_SR] >= WALL_VOTES ? SEEN_PRESENT : SEEN_ABSENT;
    motion_doubt_sides(out, fl_seen, fr_seen, sum[IR_FL] / WALL_SAMPLES, sum[IR_FR] / WALL_SAMPLES,
                       FRONT_SQUARE_OFFSET_MM, SIDE_YAW_DOUBT_MM, FRONT_WALL_REF_MM - SIDE_CLOSE_DOUBT_MM);
    return MOVE_OK;
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

void motion_stop(void){
    steer_stop();
    motors_off();
}
