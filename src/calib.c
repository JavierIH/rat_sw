#include "calib.h"
#include "health.h"
#include <math.h>
#include <stdio.h>
#include "stm32f1xx_hal.h"
#include "commands.h"
#include "encoder.h"
#include "flash_store.h"
#include "infrared.h"
#include "motion.h"
#include "motor.h"
#include "params.h"
#include "robot_config.h"
#include "storage.h"
#include "uart.h"

#define CAL_CAPACITY    320     // 6.4 KB of RAM
#define CAL_MAX_PERIOD_MS 50    // coarsest resolution before a recording gives up
#define CAL_COAST_MS    300     // keep recording after the motion ends
#define CAL_IR_SPEED    100     // mm/s backing away from the wall
#define CAL_TURN_GAP_MS 100     // pause between the quarter turns of CAL TURN

typedef struct {
    int16_t enc_l, enc_r;       // ticks since the recording started
    int16_t pwm_l, pwm_r;       // duty requested
    uint16_t ir[IR_COUNT];      // raw ADC (averaged)
    int16_t ref_fwd;            // forward reference, 0.1 mm since the recording started
    int16_t ref_rot;            // rotation reference incl. centring, 0.01 deg
} sample_t;

_Static_assert(sizeof(sample_t) == 20, "sample layout");

static sample_t samples[CAL_CAPACITY];
static volatile uint16_t count;
static volatile uint8_t recording, full;
static uint8_t period_ms, divider;
static int32_t enc_l0, enc_r0;
static float ref_fwd0, ref_rot0, ref_fwd_last, ref_rot_last;
static uint8_t ref_id;
static char description[32];
static const char *outcome = "sin datos";
static uint8_t run_armed;               // CAL RUN: 1 armed, 2 recording a run's move, 3 to dump
static volatile uint16_t coast_left;    // ms still to record after that move

static void dump(void);

void calib_tick_1ms(void){
    if(!recording) return;
    if(coast_left && --coast_left == 0){
        recording = 0;
        return;
    }
    if(divider == 0){
        if(count >= CAL_CAPACITY){
            // Full: halve the resolution instead of losing the end of the
            // test. The even samples stay evenly spaced at twice the period,
            // and this one lands exactly on the next slot of the new grid.
            if(period_ms * 2u > CAL_MAX_PERIOD_MS){
                recording = 0;
                full = 1;       // the dump says so
                return;
            }
            for(uint16_t i = 0; i < CAL_CAPACITY / 2; i++) samples[i] = samples[2 * i];
            count = CAL_CAPACITY / 2;
            period_ms = (uint8_t)(period_ms * 2u);
        }
        sample_t *s = &samples[count];
        s->enc_l = (int16_t)(encoder_total(ENCODER_L) - enc_l0);
        s->enc_r = (int16_t)(encoder_total(ENCODER_R) - enc_r0);
        s->pwm_l = motor_get(MOTOR_L);
        s->pwm_r = motor_get(MOTOR_R);
        for(uint8_t i = 0; i < IR_COUNT; i++) s->ir[i] = ir_raw((ir_sensor_t)i);
        // Every move restarts its profiles at 0: carry on from where the
        // last one ended, so the recorded reference is continuous.
        float f, r;
        uint8_t id;
        motion_reference(&f, &r, &id);
        if(id != ref_id){
            ref_fwd0 += ref_fwd_last;
            ref_rot0 += ref_rot_last;
            ref_id = id;
        }
        ref_fwd_last = f;
        ref_rot_last = r;
        s->ref_fwd = (int16_t)lroundf((ref_fwd0 + f) * 10.0f);
        s->ref_rot = (int16_t)lroundf((ref_rot0 + r) * 100.0f);
        count++;
    }
    if(++divider >= period_ms) divider = 0;
}

static void record_start(uint32_t period){
    recording = 0;
    full = 0;
    count = 0;
    divider = 0;
    period_ms = (uint8_t)(period < 1 ? 1 : period > CAL_MAX_PERIOD_MS ? CAL_MAX_PERIOD_MS : period);
    enc_l0 = encoder_total(ENCODER_L);
    enc_r0 = encoder_total(ENCODER_R);
    float f, r;
    motion_reference(&f, &r, &ref_id);
    ref_fwd0 = -f;      // the recording starts at 0 whatever the profiles hold
    ref_rot0 = -r;
    ref_fwd_last = f;
    ref_rot_last = r;
    recording = 1;
}

static void record_stop(void){
    recording = 0;
}

uint8_t calib_moves(cal_test_t test){
    return test != CAL_NOISE && test != CAL_DUMP && test != CAL_RUN;
}

void calib_path_start(void){
    if(run_armed != 1) return;
    snprintf(description, sizeof(description), "run");
    coast_left = 0;
    record_start(2);    // halves by itself if the move runs longer
    run_armed = 2;
}

void calib_path_end(const char *result){
    if(run_armed != 2) return;
    outcome = result;
    coast_left = CAL_COAST_MS;
    run_armed = 3;
}

void calib_run_finished(void){
    if(run_armed < 2) return;
    record_stop();
    coast_left = 0;
    if(run_armed == 2) outcome = "ABORTADO";
    run_armed = 0;
    print("CAL run: %s, %u muestras cada %u ms%s\n", outcome, count, period_ms,
          full ? " (buffer lleno: solo el principio)" : "");
    dump();
}

// ---- Dump ----------------------------------------------------------------------------

// Waits for a free UART slot while the console keeps working. 0 on STOP.
static uint8_t wait_slot(void){
    while(uart_tx_full()){
        health_alive();
        commands_poll();
        if(motion_abort_requested()) return 0;
    }
    return !motion_abort_requested();
}

static void dump(void){
    char kp[12], ki[12];
    if(!count){
        print("CAL: no hay datos grabados\n");
        return;
    }
    // Every constant the analysis may need, so each file stands on its own.
    if(!wait_slot()) goto interrupted;
    print("@D BEGIN %s\n", description);
    if(!wait_slot()) goto interrupted;
    print("@D INFO period_ms=%u samples=%u capacity=%u result=%s build=\"%s %s\"\n",
          period_ms, count, CAL_CAPACITY, outcome, __DATE__, __TIME__);
    if(!wait_slot()) goto interrupted;
    {
        char tpm[12], kv_l[12], kv_r[12], tau[12];
        print("@D INFO ticks_per_mm=%s cell_mm=%u turn_ticks=%d kv_l=%s kv_r=%s tau_ms=%s ks=%u\n",
              format_fixed2(tpm, sizeof(tpm), WHEEL_TICKS_PER_MM), CELL_MM, params.turn_ticks,
              format_fixed2(kv_l, sizeof(kv_l), MOTOR_KV_L), format_fixed2(kv_r, sizeof(kv_r), MOTOR_KV_R),
              format_fixed2(tau, sizeof(tau), MOTOR_TAU_S * 1000.0f), (unsigned)MOTOR_KS_PWM);
    }
    if(!wait_slot()) goto interrupted;
    print("@D INFO spd=%d fast=%d curve=%d accel=%d turn=%d turn_accel=%d kp=%s ki=%s\n",
          params.search_speed, params.fast_speed, params.curve_speed, params.accel, params.turn_speed,
          params.turn_accel, format_fixed2(kp, sizeof(kp), params.kp), format_fixed2(ki, sizeof(ki), params.ki));
    for(uint8_t i = 0; i < 2; i++){
        if(!wait_slot()) goto interrupted;
        motion_curve_info(i);
    }
    if(!wait_slot()) goto interrupted;
    {
        char cl[12], cr[12];
        // print() truncates at its buffer: keep every @D line well under it.
        print("@D INFO wall_detect_mm=%u front_ref_mm=%u front_emergency_mm=%u side_track_mm=%u\n", WALL_DETECT_MM,
              FRONT_WALL_REF_MM, FRONT_EMERGENCY_MM, SIDE_WALL_TRACK_MM);
        if(!wait_slot()) goto interrupted;
        print("@D INFO lane_mm=%u center_l=%s center_r=%s\n", (unsigned)LANE_WIDTH_MM,
              format_fixed(cl, sizeof(cl), SIDE_CENTER_L_MM, 1), format_fixed(cr, sizeof(cr), SIDE_CENTER_R_MM, 1));
    }
    if(!wait_slot()) goto interrupted;
    print("@D INFO front_square_offset_mm=%d side_yaw_doubt_mm=%u side_close_doubt_mm=%u wall_samples=%u wall_votes=%u\n",
          FRONT_SQUARE_OFFSET_MM, SIDE_YAW_DOUBT_MM, SIDE_CLOSE_DOUBT_MM, WALL_SAMPLES, WALL_VOTES);
    static const char *const NAME[IR_COUNT] = {"fl", "fr", "sl", "sr"};
    for(uint8_t i = 0; i < IR_COUNT; i++){
        if(!wait_slot()) goto interrupted;
        print("@D INFO ir_cal_%s=\"%s\"\n", NAME[i], ir_calibration_text((ir_sensor_t)i));
    }
    if(!wait_slot()) goto interrupted;
    print("@D COLS t_ms,enc_l,enc_r,pwm_l,pwm_r,raw_fl,raw_fr,raw_sl,raw_sr,ref_fwd,ref_rot\n");
    for(uint16_t i = 0; i < count; i++){
        const sample_t *s = &samples[i];
        if(!wait_slot()) goto interrupted;
        print("@D %lu,%d,%d,%d,%d,%u,%u,%u,%u,%d,%d\n", (unsigned long)i * period_ms, s->enc_l, s->enc_r,
              s->pwm_l, s->pwm_r, s->ir[IR_FL], s->ir[IR_FR], s->ir[IR_SL], s->ir[IR_SR], s->ref_fwd, s->ref_rot);
    }
    if(!wait_slot()) goto interrupted;
    print("@D END result=%s samples=%u\n", outcome, count);
    print("CAL: %u muestras enviadas (%s)\n", count, description);
    return;

interrupted:
    // The queue is usually full right now (that is what the dump was waiting
    // for): wait for room, or this message would be dropped.
    uart_wait_space(1000);
    print("CAL: envio interrumpido (CAL DUMP lo repite)\n");
}

// ---- Experiments -----------------------------------------------------------------------

static move_result_t run_step(int16_t pwm, uint32_t ms){
    // Open loop on purpose: no ramp, no steering, to see the bare motor.
    motor_set(MOTOR_L, pwm);
    motor_set(MOTOR_R, pwm);
    move_result_t r = motion_wait(ms) ? MOVE_OK : MOVE_ABORTED;
    motion_stop();
    return r;
}

// CAL FLASH: the freeze reproducer (docs/freezes.md). Every write that
// wedged the flash began within ms of the end of a move: `n` half turns in
// place (alternating, so it ends as it started), each followed `ms` later
// by a real write of the record, timed. The probe before each erase makes
// a wedged flash a ~20 ms stall and a report instead of ~200 s.
static move_result_t flash_cycles(int32_t n, int32_t ms){
    flash_store_settle((uint32_t)ms);
    move_result_t r = MOVE_OK;
    for(int32_t i = 0; i < n && r == MOVE_OK; i++){
        r = motion_turn(i & 1 ? -2 : 2);
        if(r != MOVE_OK) break;
        const uint8_t ok = storage_rewrite();
        const flash_timing_t *t = flash_store_timing();
        print("CAL flash %ld/%ld: %s, prueba %lu us, borrado %lu ms, escritura %lu ms\n", (long)(i + 1), (long)n,
              ok ? "OK" : "FALLO", (unsigned long)t->probe_us, (unsigned long)t->erase_ms,
              (unsigned long)t->program_ms);
        if(!ok) break;
    }
    flash_store_settle(FLASH_SETTLE_MS);
    return r;
}

void calib_run(cal_test_t test, int32_t a, int32_t b){
    move_result_t r = MOVE_OK;
    switch(test){
        case CAL_NOISE:
            snprintf(description, sizeof(description), "noise %ld", (long)a);
            record_start(10);
            if(!motion_wait((uint32_t)a)) r = MOVE_ABORTED;
            break;
        case CAL_STRAIGHT:
            snprintf(description, sizeof(description), "straight %ld %ld", (long)a, (long)b);
            record_start(2);    // halves by itself if the move runs longer
            r = motion_forward((uint8_t)a, (int16_t)b);
            break;
        case CAL_TURN:
            snprintf(description, sizeof(description), "turn %ld", (long)a);
            record_start(2);
            // One quarter at a time, like the turns in the maze.
            for(int32_t i = 0; i < (a < 0 ? -a : a) && r == MOVE_OK; i++){
                r = motion_turn(a < 0 ? -1 : 1);
                if(r == MOVE_OK && !motion_wait(CAL_TURN_GAP_MS)) r = MOVE_ABORTED;
            }
            break;
        case CAL_CURVE:{
            // As in a speed run: straight into the next cell, curve inside
            // it, stop at the centre of the cell after it.
            static int8_t turns[2];
            turns[0] = (int8_t)a;
            const run_path_t path = {turns, 2};
            uint8_t entered;
            snprintf(description, sizeof(description), "curve %ld %ld", (long)a, (long)b);
            record_start(2);
            r = motion_run_path(&path, (int16_t)b, (int16_t)b, &entered);
            break;
        }
        case CAL_STEP:
            snprintf(description, sizeof(description), "step %ld %ld", (long)a, (long)b);
            record_start(((uint32_t)b + CAL_COAST_MS + CAL_CAPACITY - 1) / CAL_CAPACITY);
            r = run_step((int16_t)a, (uint32_t)b);
            break;
        case CAL_IR:
            snprintf(description, sizeof(description), "ir %ld", (long)a);
            record_start(10);
            r = motion_drive_straight(-CAL_IR_SPEED, a);
            break;
        case CAL_DUMP:
            dump();
            return;
        case CAL_RUN:
            run_armed = 1;
            print("CAL RUN: se grabara el proximo movimiento continuo del run\n");
            return;
        case CAL_FLASH:
            r = flash_cycles(a, b);
            print("CAL flash: %s\n", move_result_name(r));
            return;
    }
    motion_stop();
    if(calib_moves(test) && r != MOVE_ABORTED) motion_wait(CAL_COAST_MS);     // coasting, overshoot
    record_stop();
    outcome = move_result_name(r);
    print("CAL %s: %s, %u muestras cada %u ms%s\n", description, outcome, count, period_ms,
          full ? " (buffer lleno: prueba mas corta)" : "");
    dump();
}
