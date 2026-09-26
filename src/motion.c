#include "motion.h"
#include <math.h>
#include <string.h>
#include "stm32f1xx_hal.h"
#include "commands.h"
#include "control.h"
#include "encoder.h"
#include "gpio.h"
#include "infrared.h"
#include "health.h"
#include "motor.h"
#include "params.h"
#include "robot_config.h"
#include "uart.h"

// Every move is a motion profile that the speed control (control.h) follows:
// SysTick steps it every millisecond and drives the motors, so the robot
// keeps moving smoothly while this code, in the main context, watches the
// sensors, moves the target (front wall) and decides when the move is over.

#define FRONT_CONFIRM_MS    3       // consecutive 1 ms readings for a front-wall decision
#define FRONT_ZONE_MM       (CELL_MM / 2)   // the front wall at the end of a move is tracked in its last half cell
#define FRONT_EARLY_MAX_MM  (CELL_MM / 2)   // the IR may end a move this much before the encoders say...
#define FRONT_LATE_MAX_MM   30              // ...or this much after
#define STEER_FADE_MM       40      // the centring fades out over the last mm of a move: it stops parallel
#define SQUARE_SPEED_DIV    2       // squaring rotates at half the turn speed

static void motors_off(void){
    motor_set(MOTOR_L, 0);
    motor_set(MOTOR_R, 0);
}

static void wait_next_ms(void){
    health_alive();
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
}

void motion_set_step_mode(uint8_t on){
    step_mode = on;
    if(!on) paused = 0;
}

// Commands and the START button (= stop the run) are serviced from every
// wait and control loop, so the robot reacts within a millisecond.
static void poll_inputs(void){
    health_alive();
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

// ---- Speed control (SysTick) ---------------------------------------------------------

static control_config_t control_cfg = {
    .ticks_per_mm = WHEEL_TICKS_PER_MM,
    .mm_per_deg = TICKS_PER_TURN / 90.0f / WHEEL_TICKS_PER_MM,
    .kv_l = MOTOR_KV_L,
    .kv_r = MOTOR_KV_R,
    .tau = MOTOR_TAU_S,
    .ks = MOTOR_KS_PWM,
    .fwd_kp = FWD_KP,
    .fwd_kd = FWD_KD,
    .rot_kp = ROT_KP,
    .rot_kd = ROT_KD,
    .rot_ki = ROT_KI,
    .rot_i_max = ROT_I_MAX,
    .pwm_limit = CONTROL_PWM_LIMIT,
    .settle_ki_fwd = SETTLE_KI_FWD,
    .settle_ki_rot = SETTLE_KI_ROT,
    .settle_i_max = SETTLE_I_MAX,
};

static steer_config_t steer_cfg = {
    .max_deg = STEER_MAX_DEG,
    .curve_deg = STEER_CURVE_DEG_PER_MM,
    .slew_mm = STEER_SLEW_MM_PER_MS,
    .track_mm = SIDE_WALL_TRACK_MM,
    .center_l_mm = SIDE_CENTER_L_MM,
    .center_r_mm = SIDE_CENTER_R_MM,
    .error_max_mm = STEER_ERROR_MAX_MM, .bias_window_mm = STEER_BIAS_WINDOW_MM,
    .observer_mm = STEER_OBSERVER_MM,
    // average_steps, delay_steps: set per move (TUNE STEER_AVG, IR_DELAY)
};

// Live-tunable (TUNE) values that are not in the two configs above.
static float ir_delay = IR_DELAY_MS;            // ms
static float front_track = FRONT_TRACK_MM;      // mm
static float front_ref = FRONT_TRACK_REF_MM;    // mm
static float sense_settle = SENSE_SETTLE_MS;    // ms
static float steer_average = STEER_AVERAGE_MS;  // ms
static float settle_mm = SETTLE_MM, settle_deg = SETTLE_DEG;
static float steer_vref = STEER_VREF_MM_S;      // mm/s
static float curve_radius = CURVE_RADIUS_MM, curve_ramp = CURVE_RAMP_MM, curve_angle = CURVE_ANGLE_DEG;
static float curve_pre = CURVE_PRE_ADJUST_MM, curve_post = CURVE_POST_ADJUST_MM;
// Fault injection (TUNE MOTOR_SCALE): the motors get this share of the PWM
// the control asks, behind its back, as with a low battery.
static float motor_scale = 1.0f;

// Owned by SysTick while control_on; the main context only reads them, and
// writes fwd.target (one aligned float store) to move the end of a straight.
static profile_t fwd, rot;
static control_t ctl;
static steer_t steer;
static volatile uint8_t control_on, steer_on;
static volatile float steer_gain;
// A path run (motion_run_path) steps `run` instead of the two profiles; it
// writes them, so everything else reads fwd/rot as in any move. The main
// context only writes run.stop_at and run.hold (one aligned store each).
static path_run_t run;
static volatile uint8_t path_on;
static uint8_t steer_blind;         // centring suspended by a curve (SysTick)

// NMI (sysclock.c): the crystal failed and every timing is 9 times off until
// the main context recovers the clock. Stop driving now and end the run.
void clock_failure_hook(void){
    control_on = 0;
    path_on = 0;
    abort_flag = 1;
    motor_emergency_stop();
}
static int32_t tick_l, tick_r;      // encoder totals at the last SysTick
// Where the robot was each of the last TRAIL_LEN ms (forward axis): the IR
// report the past (IR_DELAY_MS), and must be added to the position then.
#define TRAIL_LEN 64
#define IR_DELAY_MAX 60     // TUNE limit
_Static_assert(IR_DELAY_MAX < TRAIL_LEN && IR_DELAY_MAX + STEER_AVERAGE_MAX / 2 < STEER_DELAY_MAX, "IR delay too long");
static float trail[TRAIL_LEN];
static volatile uint8_t trail_slot;

// Where the robot was when the IR readings now in were taken (this move).
static float fwd_at_ir(void){
    const uint8_t ms = (uint8_t)ir_delay;
    return trail[(trail_slot + TRAIL_LEN - 1u - ms) % TRAIL_LEN];
}

void motion_tick_1ms(void){
    const int32_t l = encoder_total(ENCODER_L), r = encoder_total(ENCODER_R);
    const int32_t dl = l - tick_l, dr = r - tick_r;
    tick_l = l;
    tick_r = r;
    if(!control_on) return;
    if(path_on){
        run.lag = ctl.fwd_error;
        path_step(&run, &fwd, &rot, CONTROL_DT_S);
    }
    else{
        profile_step(&fwd, CONTROL_DT_S);
        profile_step(&rot, CONTROL_DT_S);
    }
    float heading = 0.0f;
    if(steer_on){
        // In a curve the side walls say nothing about the centre line, and
        // right after it the IR still report the curve (IR_DELAY_MS): hold
        // the heading offset, and start afresh in the new corridor.
        if(path_on && !(run.s < run.curve_start && fwd_at_ir() >= run.last_curve_end)){
            steer_blind = 1;
            heading = steer.heading;
            leds_set_mask(0);
        }
        else{
            if(steer_blind){
                steer_restart(&steer);
                steer_blind = 0;
            }
            const float ds = 0.5f * fabsf((float)(dl + dr)) / WHEEL_TICKS_PER_MM;
            // Heading since the move started, relative to the corridor (the reference's turns taken out).
            const float to_corridor = ctl.steer_prev - ctl.rot_error;
            heading = steer_step(&steer, &steer_cfg, ir_mm(IR_SL), ir_mm(IR_SR), ds, to_corridor, steer_gain);
            static const uint8_t WALL_LEDS[4] = {0x00, 0x07, 0x38, 0x3F};   // none, right, left, both
            leds_set_mask(WALL_LEDS[steer.wall & 3u]);
        }
    }
    control_step(&ctl, &control_cfg, &fwd, &rot, heading, dl, dr, CONTROL_DT_S);
    motor_set(MOTOR_L, (int16_t)((float)ctl.pwm_l * motor_scale));
    motor_set(MOTOR_R, (int16_t)((float)ctl.pwm_r * motor_scale));
    trail[trail_slot] = fwd.pos - ctl.fwd_error;
    trail_slot = (uint8_t)((trail_slot + 1u) % TRAIL_LEN);
}

static uint8_t move_id;     // changes with every move (CAL recordings rebase their reference)

void motion_reference(float *fwd_mm, float *rot_deg, uint8_t *id){
    *fwd_mm = fwd.pos;
    *rot_deg = rot.pos + ctl.steer_prev;
    *id = move_id;
}

// A move from rest. Nothing drives until control_go(); SysTick leaves the
// state alone meanwhile.
static void control_begin(uint8_t steering){
    control_on = 0;
    path_on = 0;
    steer_blind = 0;
    move_id++;
    steer_cfg.average_steps = (uint8_t)steer_average;
    steer_cfg.delay_steps = (uint8_t)(ir_delay + steer_average / 2);  // the averaging delays by half its window
    control_cfg.mm_per_deg = (float)params.turn_ticks / 90.0f / WHEEL_TICKS_PER_MM;
    steer_cfg.kp = params.kp;
    steer_cfg.ki = params.ki * 0.001f;     // per m travelled -> per mm
    profile_reset(&fwd);
    profile_reset(&rot);
    control_reset(&ctl);
    steer_reset(&steer);
    steer_gain = 1.0f;
    steer_on = steering;
    for(uint8_t i = 0; i < TRAIL_LEN; i++) trail[i] = 0.0f;
}

static void control_go(void){
    control_on = 1;
}

// Short brake: the move is over (or cut short).
static void control_end(void){
    control_on = 0;
    path_on = 0;
    motors_off();
    if(steer_on){
        steer_on = 0;
        leds_set_mask(0);
    }
}

// Where the robot really is on each axis: the reference minus the error.
static float fwd_actual(void){
    return fwd.pos - ctl.fwd_error;
}

// ---- Move supervision ----------------------------------------------------------------------

// Side walls sampled on the way into the last cell of a forward move, used by
// the next motion_sense_walls() instead of reading them at the stop. Any other
// action makes them stale.
static struct { uint8_t valid, n, votes_l, votes_r; } side_pass;

static void side_pass_clear(void){
    side_pass.valid = 0;
    side_pass.n = 0;
    side_pass.votes_l = 0;
    side_pass.votes_r = 0;
}

typedef struct {
    uint32_t start, deadline, done_at;
    uint8_t done;               // the profiles finished at done_at
    int32_t l0, r0;             // encoder totals at the start
    float fwd_err_max, rot_err_max;
} guard_t;

static void guard_start(guard_t *g, uint32_t timeout_ms){
    g->start = HAL_GetTick();
    g->l0 = encoder_total(ENCODER_L);
    g->r0 = encoder_total(ENCODER_R);
    g->deadline = g->start + timeout_ms;
    g->done = 0;
    g->done_at = 0;
    g->fwd_err_max = g->rot_err_max = 0.0f;
}

// PAUSE mid-move: brake and wait; on RESUME, carry on from where the robot
// actually is, from standstill, to the same target.
static void hold_while_paused(guard_t *g){
    const uint32_t since = HAL_GetTick();
    if(path_on){
        // Brake to a stop on the path (in a curve too) and hold there: the
        // path carries on from the same point, since its heading depends on
        // the distance alone.
        run.hold = 1;
        while(paused && !abort_flag) poll_inputs();
        run.hold = 0;
        g->deadline += HAL_GetTick() - since;
        return;
    }
    control_on = 0;
    motors_off();
    while(paused && !abort_flag) poll_inputs();
    if(abort_flag) return;
    profile_resume(&fwd, fwd_actual());
    profile_resume(&rot, rot.pos - ctl.rot_error);
    control_clear_errors(&ctl);
    g->deadline += HAL_GetTick() - since;
    control_on = 1;
}

// Once per ms of every move: commands, pause, abort, timeout, and the
// following error. Far behind the reference means something holds the robot
// (a wall, a post) or the wheels slip: pushing on would only make it worse.
static move_result_t guard_check(guard_t *g){
    poll_inputs();
    if(paused && !abort_flag) hold_while_paused(g);
    if(abort_flag) return MOVE_ABORTED;
    const float fe = fabsf(ctl.fwd_error), re = fabsf(ctl.rot_error);
    if(fe > g->fwd_err_max) g->fwd_err_max = fe;
    if(re > g->rot_err_max) g->rot_err_max = re;
    if(fe > FWD_ERROR_MAX_MM) return MOVE_STALLED;
    if(re > ROT_ERROR_MAX_DEG) return MOVE_SLIPPED;
    if((int32_t)(HAL_GetTick() - g->deadline) >= 0) return MOVE_TIMEOUT;
    return MOVE_OK;
}

// 1 once the profiles are done and the robot has caught up with them (or
// SETTLE_MAX_MS later: a few tenths of a mm of friction are not worth more).
static uint8_t guard_settled(guard_t *g){
    if(fwd.active || rot.active) return 0;
    const uint32_t now = HAL_GetTick();
    if(!g->done){
        g->done = 1;
        g->done_at = now;
    }
    if(fabsf(ctl.fwd_error) < settle_mm && fabsf(ctl.rot_error) < settle_deg && encoder_idle_ms() >= 5) return 1;
    return now - g->done_at >= SETTLE_MAX_MS;
}

// Runs the move set up in fwd/rot until it settles or fails.
static move_result_t run_to_end(guard_t *g){
    move_result_t r;
    control_go();
    for(;;){
        wait_next_ms();
        r = guard_check(g);
        if(r != MOVE_OK || guard_settled(g)) break;
    }
    control_end();
    return r;
}

static void print_errors(const guard_t *g){
    char fe[12], re[12];
    print(" t=%lums err=%smm/%sdeg\n", (unsigned long)(HAL_GetTick() - g->start),
          format_fixed2(fe, sizeof(fe), g->fwd_err_max), format_fixed2(re, sizeof(re), g->rot_err_max));
}

// ---- Straight moves ---------------------------------------------------------------------------

// Straight without walls: alignment nudges, backing up, the IR calibration.
static move_result_t drive(float mm, float speed, guard_t *g){
    control_begin(0);
    profile_start(&fwd, mm, speed, 0.0f, (float)params.accel);
    guard_start(g, MOVE_TIMEOUT_BASE_MS);
    return run_to_end(g);
}

// After an obstacle stop: reverse to the last cell centre passed.
static move_result_t back_up(float traveled){
    guard_t g;
    move_result_t r = drive(-traveled, ALIGN_SPEED, &g);
    move_result_t result = r == MOVE_OK ? MOVE_BLOCKED : r == MOVE_ABORTED ? MOVE_ABORTED : MOVE_LOST;
    char mm[12];
    print("obstaculo delante: marcha atras %s (%smm)\n", result == MOVE_BLOCKED ? "OK" : move_result_name(result),
          format_fixed2(mm, sizeof(mm), traveled));
    return result;
}

static uint8_t curve_from_tuning(curve_t *c){
    return curve_setup(c, curve_radius, curve_ramp, curve_angle, curve_pre, curve_post, CELL_MM);
}

// Fastest curve the motors can follow: the outer wheel's feedforward where a
// ramp meets the arc (running v (1 + h / R) and accelerating h v^2 / (R
// ramp), h the half track) within CURVE_PWM_SHARE of the limit.
static float curve_speed_limit(const curve_t *c){
    const float h = control_cfg.mm_per_deg * 57.29578f;
    const float kv = fmaxf(control_cfg.kv_l, control_cfg.kv_r);
    const float a = kv * control_cfg.tau * h / (c->radius * c->ramp);
    const float b = kv * (1.0f + h / c->radius);
    const float room = CURVE_PWM_SHARE * control_cfg.pwm_limit - control_cfg.ks;
    return (sqrtf(b * b + 4.0f * a * room) - b) / (2.0f * a);
}

// ---- Search legs (motion_explore) ---------------------------------------------------------

typedef struct {
    next_cell_fn decide;
    void *ctx;
    uint8_t decided;                // cells decided: the next one to decide
    uint8_t stopping;               // a stop was decided (or forced): no more cells
    float entry;                    // along the path: entry edge of the next cell to decide
    uint32_t next_sample;           // HAL tick of the next reading
    uint8_t samples, walls_l, walls_r;      // side readings of the next cell
    uint8_t last_l, last_r;         // sighting_t of the sides of the last cell decided
} explorer_t;

// Unanimous readings, enough of them, or doubtful.
static uint8_t sighting(uint8_t samples, uint8_t walls){
    if(samples < SEARCH_MIN_READINGS) return SEEN_DOUBTFUL;
    return walls == samples ? SEEN_PRESENT : walls == 0 ? SEEN_ABSENT : SEEN_DOUBTFUL;
}

// SysTick steps the path: grow it with SysTick masked (a few us).
static uint8_t grow_path(void){
    __disable_irq();
    const uint8_t ok = path_grow(&run);
    __enable_irq();
    return ok;
}

// Every ms of a search leg: side readings of the next cell, and its decision
// when due, with its front wall in view.
static void explore_step(explorer_t *ex, guard_t *g, float v, float ir_at, uint8_t front_seen){
    if(ex->stopping) return;
    const uint32_t now = HAL_GetTick();
    if((int32_t)(now - ex->next_sample) >= 0){
        ex->next_sample = now + WALL_SAMPLE_MS;
        if(ir_at >= ex->entry - SEARCH_SIDE_FROM_MM && ir_at <= ex->entry + SEARCH_SIDE_TO_MM){
            ex->samples++;
            if(ir_mm(IR_SL) < WALL_DETECT_MM) ex->walls_l++;
            if(ir_mm(IR_SR) < WALL_DETECT_MM) ex->walls_r++;
        }
    }
    // Due just before the reference must start braking for where it ends
    // now: the cell's centre, or where a wall seen in front moved it.
    if(!run.done && run.s < run.stop_at - v * v / (2.0f * run.accel) - SEARCH_LATE_MARGIN_MM) return;

    wall_sense_t w;
    w.left = sighting(ex->samples, ex->walls_l);
    w.right = sighting(ex->samples, ex->walls_r);
    const float avg = 0.5f * (ir_mm(IR_FL) + ir_mm(IR_FR));
    w.front = front_seen ? SEEN_PRESENT : avg > SEARCH_FRONT_OPEN_MM ? SEEN_ABSENT : SEEN_DOUBTFUL;
    w.moving = 1;
    next_move_t next = ex->decide(&w, ex->ctx);
    ex->last_l = w.left;
    ex->last_r = w.right;
    ex->samples = ex->walls_l = ex->walls_r = 0;
    ex->decided++;
    // Straight on: one cell more, unless a wall in front already moved the
    // end (then it stops in this cell, as if told to).
    if(next == NEXT_STRAIGHT && grow_path()){
        g->deadline += MOVE_TIMEOUT_PER_CELL_MS;
        ex->entry += CELL_MM;
        return;
    }
    ex->stopping = 1;
}

// Every forward move: a whole path in one go (path.h), straights and smooth
// curves, a plain straight being a path without curves. SysTick steps the
// reference; this loop watches the sensors and decides when it is over. The
// front sensors are used on straights only, with readings taken on them: to
// stop at the right distance from a wall at the end, and to stop short, at a
// cell centre, if a wall shows up where the (verified) map had a passage.
static move_result_t run_path(const run_path_t *path, int16_t cruise_speed, int16_t curve_speed, uint8_t *entered,
                              explorer_t *ex){
    side_pass_clear();
    *entered = 0;
    if(!path->cells) return MOVE_OK;
    curve_t curve;
    control_begin(1);
    // TUNE only takes curves that fit, and search.c builds valid paths.
    if(!curve_from_tuning(&curve)
       || !path_start(&run, path, &curve, CELL_MM, (float)cruise_speed,
                      fminf((float)curve_speed, curve_speed_limit(&curve)), (float)params.accel)){
        print("!! ruta no valida\n");
        return MOVE_LOST;
    }
    moved = 1;
    guard_t g;
    uint8_t ir_seen = 0, ir_emergency = 0, emergency = 0, short_stop = 0, stop_cell = 0;
    const char *stop = "ENC";
    move_result_t result;
    float vmax = 0.0f, at = 0.0f, planned_end = run.length;
    path_on = 1;
    guard_start(&g, MOVE_TIMEOUT_BASE_MS + (uint32_t)path->cells * MOVE_TIMEOUT_PER_CELL_MS);
    control_go();
    for(;;){
        wait_next_ms();
        result = guard_check(&g);
        if(result != MOVE_OK){
            stop = move_result_name(result);
            break;
        }
        at = fwd_actual();
        const float v = run.v, ir_at = fwd_at_ir();
        if(v > vmax) vmax = v;
        // Centring gain: KP up to steer_vref, then as 1/speed (see
        // STEER_VREF_MM_S), fading out over the last STEER_FADE_MM before
        // every curve and the end, so the robot gets there parallel.
        const float remaining = fminf(run.curve_start, run.stop_at) - at;
        const float fade = remaining < STEER_FADE_MM ? fmaxf(remaining, 0.0f) / STEER_FADE_MM : 1.0f;
        steer_gain = fade * (v > steer_vref ? steer_vref / v : 1.0f);
        if(ex){
            if(!short_stop) planned_end = run.length;     // it grows as the search decides
            explore_step(ex, &g, v, ir_at, ir_seen >= FRONT_CONFIRM_MS);
        }

        if(run.s >= run.curve_start || !path_on_straight(&run, ir_at)){
            ir_seen = ir_emergency = 0;     // turning, or the readings are from the curve
            if(guard_settled(&g)) break;
            continue;
        }
        // On the straight into the last cell (or into a short stop) the end
        // of the straight is the end of the move.
        const uint8_t last_cell = run.next >= run.path.cells;
        const uint8_t ending = last_cell || short_stop;
        const float expected = short_stop ? planned_end : path_straight_end(&run);

        // Side walls of the last cell, read on the way in: here the angled
        // beams hit the middle of its walls. At the stop they aim a couple
        // of cm from the next post, and caught it as phantom walls. The
        // readings are IR_DELAY_MS old: count from where they were taken.
        if(last_cell && !short_stop && run.stop_at - ir_at <= SIDE_PASS_MM && side_pass.n < WALL_SAMPLES){
            side_pass.n++;
            if(ir_mm(IR_SL) < WALL_DETECT_MM) side_pass.votes_l++;
            if(ir_mm(IR_SR) < WALL_DETECT_MM) side_pass.votes_r++;
        }
        const float fl = ir_mm(IR_FL), fr = ir_mm(IR_FR);
        const uint8_t wall = fl < front_track && fr < front_track
            && fabsf(fl - fr - (float)FRONT_SQUARE_OFFSET_MM) < FRONT_IR_MAX_DIFF_MM;
        ir_seen = wall ? (uint8_t)(ir_seen < 255u ? ir_seen + 1u : ir_seen) : 0u;
        if(ir_seen >= FRONT_CONFIRM_MS && !run.done){
            // Where the robot stops centred before this wall (FL/FR average,
            // as in motion_align_front()). The reading is IR_DELAY_MS old:
            // the wall is that far from where the robot was then (within 0.6
            // mm on the robot; the plain reading put it 18 mm too far at 400
            // mm/s).
            float end = ir_at + 0.5f * (fl + fr) - front_ref;
            const float soonest = run.s + v * v / (4.0f * run.accel);     // braking twice as hard
            float centre;
            if(end < expected - 0.5f * CELL_MM){
                // Not the wall this straight leads to: one the map had as
                // open. Stop at the centre of the cell before it.
                if(!short_stop && path_straight_centre(&run, end, 1, &stop_cell, &centre) && centre >= soonest){
                    planned_end = run.stop_at = centre;
                    short_stop = 1;
                    stop = "PARED";
                    if(ex) ex->stopping = 1;
                }
            }
            else if(ending && run.stop_at - at <= FRONT_ZONE_MM){
                // The wall at the end is the best position reference: aim
                // the stop at it. The reference brakes into the new end, so
                // the robot stops there instead of coasting past a trigger.
                end = fminf(fmaxf(end, planned_end - FRONT_EARLY_MAX_MM), planned_end + FRONT_LATE_MAX_MM);
                run.stop_at = fmaxf(end, soonest);
                if(!short_stop) stop = "IR";
            }
        }
        // Something this close well before the end of the straight, with no
        // room left to stop at a cell centre: brake now, farther out when
        // going faster. The readings are IR_DELAY_MS old: count the way since.
        if(ir_at < expected - FRONT_ZONE_MM){
            const float near_mm = FRONT_EMERGENCY_MM + (at - ir_at) + v * v / (2.0f * EMERGENCY_DECEL);
            ir_emergency = (fl < near_mm && fr < near_mm) ? (uint8_t)(ir_emergency + 1u) : 0u;
            if(ir_emergency >= FRONT_CONFIRM_MS){
                result = MOVE_LOST;
                emergency = 1;
                stop = "OBSTACULO";
                break;
            }
        }
        if(guard_settled(&g)) break;
    }
    control_end();
    at = fwd_actual();
    if(result == MOVE_OK){
        *entered = short_stop ? stop_cell : run.path.cells;
        if(short_stop) result = MOVE_BLOCKED;
        side_pass.valid = !short_stop && side_pass.n >= WALL_SAMPLES;
    }
    if(ex && (result == MOVE_OK || result == MOVE_BLOCKED)){
        if(result == MOVE_OK && run.path.cells > ex->decided){
            result = MOVE_LOST;     // stopped in a cell the search never decided: cannot happen in time
            stop = "SIN DECIDIR";
        }
        // The sides of the cell it stopped in, as read on the way in.
        side_pass.valid = *entered == ex->decided && *entered > 0
                       && ex->last_l != SEEN_DOUBTFUL && ex->last_r != SEEN_DOUBTFUL;
        side_pass.n = WALL_SAMPLES;
        side_pass.votes_l = ex->last_l == SEEN_PRESENT ? WALL_SAMPLES : 0;
        side_pass.votes_r = ex->last_r == SEEN_PRESENT ? WALL_SAMPLES : 0;
    }

    if(params.log_level >= 2){
        char dist[12], end[12];
        const char sides[3] = {side_pass.valid ? (side_pass.votes_l >= WALL_VOTES ? '1' : '0') : '-',
                               side_pass.valid ? (side_pass.votes_r >= WALL_VOTES ? '1' : '0') : '-', '\0'};
        format_fixed2(dist, sizeof(dist), at);
        format_fixed2(end, sizeof(end), run.stop_at);
        if(path->turn){
            print("ruta %u celdas, %u curvas: fin=%s dist=%s/%smm v=%d/%d IR(FL=%d FR=%d) lados=%s", run.path.cells,
                  run.curves, stop, dist, end, (int)vmax, (int)run.v_curve, (int)ir_mm(IR_FL), (int)ir_mm(IR_FR), sides);
        }
        else{
            print("%s %u: fin=%s dist=%smm obj=%smm vmax=%dmm/s IR(FL=%d FR=%d SL=%d SR=%d) lados=%s",
                  ex ? "exploracion" : "avance", run.path.cells,
                  stop, dist, end, (int)vmax, (int)ir_mm(IR_FL), (int)ir_mm(IR_FR), (int)ir_mm(IR_SL),
                  (int)ir_mm(IR_SR), sides);
        }
        if(run.scale_min < 0.995f) print(" ritmo=%d%%", (int)(100.0f * run.scale_min + 0.5f));
        print_errors(&g);
    }
    if(emergency){
        // Braked hard on a straight: back to the last cell centre passed,
        // which the encoders know exactly.
        side_pass_clear();
        float centre;
        if(path_straight_centre(&run, at, 0, &stop_cell, &centre)){
            result = back_up(at - centre);
            if(result == MOVE_BLOCKED) *entered = stop_cell;
        }
    }
    return result;
}

move_result_t motion_run_path(const run_path_t *path, int16_t cruise_speed, int16_t curve_speed, uint8_t *entered){
    return run_path(path, cruise_speed, curve_speed, entered, NULL);
}

// A search leg: a straight path that starts one cell long and grows as
// explore_step() asks the search about each next cell.
move_result_t motion_explore(int16_t speed, next_cell_fn decide, void *ctx, uint8_t *entered){
    explorer_t ex = {0};
    ex.decide = decide;
    ex.ctx = ctx;
    ex.entry = 0.5f * CELL_MM;
    ex.next_sample = HAL_GetTick();
    ex.last_l = ex.last_r = SEEN_DOUBTFUL;
    const run_path_t leg = {NULL, 1};
    return run_path(&leg, speed, speed, entered, &ex);
}

// A straight is a path without curves.
move_result_t motion_forward(uint8_t cells, int16_t cruise_speed){
    const run_path_t straight = {NULL, cells};
    uint8_t entered;
    const move_result_t r = motion_run_path(&straight, cruise_speed, cruise_speed, &entered);
    // Stopped short at a cell centre the caller could not tell from the start.
    return r == MOVE_BLOCKED && entered ? MOVE_LOST : r;
}

move_result_t motion_drive_straight(int16_t speed, int32_t mm){
    guard_t g;
    moved = 1;
    side_pass_clear();
    return drive(speed < 0 ? -(float)mm : (float)mm, (float)(speed < 0 ? -speed : speed), &g);
}

// ---- Turns -------------------------------------------------------------------------------------

// In place, `deg` clockwise (negative = left). The forward loop holds the
// robot on its spot meanwhile.
static move_result_t rotate(float deg, float speed, guard_t *g){
    control_begin(0);
    profile_start(&rot, deg, speed, 0.0f, (float)params.turn_accel);
    guard_start(g, MOVE_TIMEOUT_BASE_MS);
    return run_to_end(g);
}

move_result_t motion_turn(int8_t quarter_turns){
    if(!quarter_turns) return MOVE_OK;
    guard_t g;
    moved = 1;
    side_pass_clear();     // the sides are other walls now
    const float deg = 90.0f * (float)quarter_turns;
    move_result_t r = rotate(deg, (float)params.turn_speed, &g);
    if(params.log_level >= 2){
        // Angle the encoders saw: exactly the target unless it failed.
        const float dl = (float)(encoder_total(ENCODER_L) - g.l0), dr = (float)(encoder_total(ENCODER_R) - g.r0);
        char turned[12];
        print("giro %s: %sdeg de %d fin=%s TURNTICKS=%d", quarter_turns > 0 ? "der" : "izq",
              format_fixed2(turned, sizeof(turned), 0.5f * (dl - dr) / WHEEL_TICKS_PER_MM / control_cfg.mm_per_deg),
              (int)deg, r == MOVE_OK ? "OK" : move_result_name(r), params.turn_ticks);
        print_errors(&g);
    }
    return r;
}

// ---- Front alignment -------------------------------------------------------------------------

static float front_skew(void){
    return ir_mm(IR_FL) - ir_mm(IR_FR) - (float)FRONT_SQUARE_OFFSET_MM;
}

// Rotates in place by the yaw the front sensors see, so each stop facing a
// wall resets the heading error the moves leave behind.
static void square_to_front(void){
    const float skew = front_skew();
    if(fabsf(skew) <= SQUARE_TOL_MM || fabsf(skew) > SQUARE_MAX_SKEW_MM) return;
    guard_t g;
    const float deg = skew / SQUARE_MM_PER_DEG;     // FL farther: yawed left, rotate right
    move_result_t r = rotate(deg, (float)params.turn_speed / SQUARE_SPEED_DIV, &g);
    if(params.log_level >= 2){
        char a[12];
        print("escuadrado: sesgo=%dmm giro=%sdeg -> %dmm%s%s", (int)skew, format_fixed2(a, sizeof(a), deg),
              (int)front_skew(), r == MOVE_OK ? "" : " ", r == MOVE_OK ? "" : move_result_name(r));
        print_errors(&g);
    }
}

void motion_align_front(void){
    float fl = ir_mm(IR_FL);
    float fr = ir_mm(IR_FR);
    if(fl >= WALL_DETECT_MM || fr >= WALL_DETECT_MM) return;
    square_to_front();
    fl = ir_mm(IR_FL);
    fr = ir_mm(IR_FR);
    if(fl >= WALL_DETECT_MM || fr >= WALL_DETECT_MM) return;
    if(fabsf(fl - fr - (float)FRONT_SQUARE_OFFSET_MM) > FRONT_IR_MAX_DIFF_MM) return;
    const float error_mm = 0.5f * (fl + fr) - (float)FRONT_WALL_REF_MM;
    if(fabsf(error_mm) <= ALIGN_DEADBAND_MM || fabsf(error_mm) > ALIGN_MAX_MM) return;
    guard_t g;
    move_result_t r = drive(error_mm, ALIGN_SPEED, &g);     // farther than expected: forward
    if(params.log_level >= 2){
        char e[12];
        print("alineado frontal: err=%smm%s%s", format_fixed2(e, sizeof(e), error_mm),
              r == MOVE_OK ? "" : " ", r == MOVE_OK ? "" : move_result_name(r));
        print_errors(&g);
    }
}

// ---- Sensing and signalling ----------------------------------------------------------------------

move_result_t motion_sense_walls(wall_sense_t *out){
    // Let the chassis stop rocking first: sampling right at the stop was a
    // source of phantom walls.
    if(!motion_wait((uint32_t)sense_settle)) return MOVE_ABORTED;
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
    // Just arrived from a straight: the sides read on the way in are better
    // than any reading from here (see SIDE_PASS_MM).
    out->moving = side_pass.valid;
    if(side_pass.valid){
        out->left = side_pass.votes_l >= WALL_VOTES ? SEEN_PRESENT : SEEN_ABSENT;
        out->right = side_pass.votes_r >= WALL_VOTES ? SEEN_PRESENT : SEEN_ABSENT;
    }
    side_pass_clear();
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
    control_end();
    side_pass_clear();
}

// ---- Live tuning (TUNE) ----------------------------------------------------------------

typedef struct {
    const char *name;
    float *value;
    float min, max;
    uint8_t decimals;
} tunable_t;

static const tunable_t TUNABLES[] = {
    {"FWD_KP", &control_cfg.fwd_kp, 0.0f, 300.0f, 1},
    {"FWD_KD", &control_cfg.fwd_kd, 0.0f, 10.0f, 2},
    {"ROT_KP", &control_cfg.rot_kp, 0.0f, 200.0f, 1},
    {"ROT_KD", &control_cfg.rot_kd, 0.0f, 10.0f, 2},
    {"ROT_KI", &control_cfg.rot_ki, 0.0f, 2000.0f, 0},
    {"KV_L", &control_cfg.kv_l, 0.3f, 3.0f, 3},
    {"KV_R", &control_cfg.kv_r, 0.3f, 3.0f, 3},
    {"TAU", &control_cfg.tau, 0.0f, 0.3f, 3},
    {"KS", &control_cfg.ks, 0.0f, 200.0f, 1},
    {"SETTLE_KI_FWD", &control_cfg.settle_ki_fwd, 0.0f, 20000.0f, 0},
    {"SETTLE_KI_ROT", &control_cfg.settle_ki_rot, 0.0f, 20000.0f, 0},
    {"SETTLE_I_MAX", &control_cfg.settle_i_max, 0.0f, 400.0f, 0},
    {"STEER_MAX", &steer_cfg.max_deg, 0.0f, 30.0f, 1},
    {"STEER_CURVE", &steer_cfg.curve_deg, 0.01f, 5.0f, 2},
    {"STEER_AVG", &steer_average, 1.0f, STEER_AVERAGE_MAX, 0},
    {"BIAS_WIN", &steer_cfg.bias_window_mm, 0.0f, 30.0f, 1},
    {"OBSERVER", &steer_cfg.observer_mm, 0.0f, 500.0f, 0},
    {"STEER_VREF", &steer_vref, 100.0f, 3000.0f, 0},
    {"SETTLE_MM", &settle_mm, 0.1f, 5.0f, 2},
    {"SETTLE_DEG", &settle_deg, 0.1f, 5.0f, 2},
    {"CENTER_L", &steer_cfg.center_l_mm, 40.0f, 130.0f, 1},
    {"CENTER_R", &steer_cfg.center_r_mm, 40.0f, 130.0f, 1},
    {"IR_DELAY", &ir_delay, 0.0f, IR_DELAY_MAX, 0},
    {"FRONT_TRACK", &front_track, 100.0f, 250.0f, 0},
    {"FRONT_REF", &front_ref, 60.0f, 130.0f, 1},
    {"WHEEL_DIFF", &control_cfg.wheel_diff, -0.05f, 0.05f, 3},
    {"SENSE_SETTLE", &sense_settle, 0.0f, 200.0f, 0},
    {"CURVE_R", &curve_radius, 30.0f, 120.0f, 1},
    {"CURVE_RAMP", &curve_ramp, 1.0f, 120.0f, 1},
    {"CURVE_ANGLE", &curve_angle, 80.0f, 100.0f, 2},
    {"CURVE_PRE", &curve_pre, -40.0f, 40.0f, 1},
    {"CURVE_POST", &curve_post, -40.0f, 40.0f, 1},
    {"MOTOR_SCALE", &motor_scale, 0.5f, 1.0f, 2},
};

#define TUNABLE_COUNT (sizeof(TUNABLES) / sizeof(TUNABLES[0]))

void motion_curve_info(uint8_t line){
    curve_t c;
    if(!curve_from_tuning(&c)) return;
    char a[12], b[12], d[12], e[12];
    if(line == 0){
        print("@D INFO curve_r=%s curve_ramp=%s curve_angle=%s curve_len=%s curve_vmax=%d\n",
              format_fixed(a, sizeof(a), c.radius, 1), format_fixed(b, sizeof(b), c.ramp, 1),
              format_fixed(d, sizeof(d), c.angle, 2), format_fixed(e, sizeof(e), c.length, 1),
              (int)curve_speed_limit(&c));
    }
    else{
        print("@D INFO curve_pre=%s curve_post=%s curve_pre_adj=%s curve_post_adj=%s\n",
              format_fixed(a, sizeof(a), c.pre, 1), format_fixed(b, sizeof(b), c.post, 1),
              format_fixed(d, sizeof(d), curve_pre, 1), format_fixed(e, sizeof(e), curve_post, 1));
    }
}

void motion_tune_list(void){
    char v[16];
    for(uint8_t i = 0; i < TUNABLE_COUNT; i++){
        uart_wait_space(200);
        print("%s=%s\n", TUNABLES[i].name, format_fixed(v, sizeof(v), *TUNABLES[i].value, TUNABLES[i].decimals));
    }
}

void motion_tune_set(const char *name, float value){
    char v[16], lo[16], hi[16];
    for(uint8_t i = 0; i < TUNABLE_COUNT; i++){
        const tunable_t *t = &TUNABLES[i];
        if(strcmp(t->name, name) != 0) continue;
        if(value < t->min || value > t->max){
            print("%s %s-%s\n", t->name, format_fixed(lo, sizeof(lo), t->min, t->decimals),
                  format_fixed(hi, sizeof(hi), t->max, t->decimals));
            return;
        }
        const float old = *t->value;
        *t->value = value;      // one aligned store: SysTick sees the old or the new value
        curve_t c;
        const uint8_t shape = strncmp(name, "CURVE", 5) == 0;
        if(shape && !curve_from_tuning(&c)){
            *t->value = old;
            print("%s: la curva no cabe en la celda (se queda en %s)\n", t->name,
                  format_fixed(v, sizeof(v), old, t->decimals));
            return;
        }
        print("%s=%s (hasta reiniciar; en robot_config.h para siempre)\n", t->name,
              format_fixed(v, sizeof(v), value, t->decimals));
        if(shape){
            char len[12], pre[12], post[12];
            print("curva: %smm, recta antes %smm y despues %smm\n", format_fixed(len, sizeof(len), c.length, 1),
                  format_fixed(pre, sizeof(pre), c.pre, 1), format_fixed(post, sizeof(post), c.post, 1));
        }
        return;
    }
    print("TUNE: %s no existe (TUNE solo: lista)\n", name);
}
