#include "calib.h"
#include <stdio.h>
#include "stm32f1xx_hal.h"
#include "commands.h"
#include "encoder.h"
#include "infrared.h"
#include "motion.h"
#include "motor.h"
#include "params.h"
#include "robot_config.h"
#include "uart.h"

#define CAL_CAPACITY    384     // 6 KB of RAM
#define CAL_COAST_MS    300     // keep recording after the motion ends

typedef struct {
    int16_t enc_l, enc_r;       // ticks since the recording started
    int16_t pwm_l, pwm_r;       // duty requested
    uint16_t ir[IR_COUNT];      // raw ADC (averaged)
} sample_t;

_Static_assert(sizeof(sample_t) == 16, "sample layout");

static sample_t samples[CAL_CAPACITY];
static volatile uint16_t count;
static volatile uint8_t recording;
static uint8_t period_ms, divider;
static int32_t enc_l0, enc_r0;
static char description[32];
static const char *outcome = "sin datos";

void calib_tick_1ms(void){
    if(!recording) return;
    if(divider == 0){
        if(count >= CAL_CAPACITY){
            recording = 0;      // full: the dump says so
            return;
        }
        sample_t *s = &samples[count];
        s->enc_l = (int16_t)(encoder_total(ENCODER_L) - enc_l0);
        s->enc_r = (int16_t)(encoder_total(ENCODER_R) - enc_r0);
        s->pwm_l = motor_get(MOTOR_L);
        s->pwm_r = motor_get(MOTOR_R);
        for(uint8_t i = 0; i < IR_COUNT; i++) s->ir[i] = ir_raw((ir_sensor_t)i);
        count++;
    }
    if(++divider >= period_ms) divider = 0;
}

static void record_start(uint32_t period){
    recording = 0;
    count = 0;
    divider = 0;
    period_ms = (uint8_t)(period < 1 ? 1 : period > 50 ? 50 : period);
    enc_l0 = encoder_total(ENCODER_L);
    enc_r0 = encoder_total(ENCODER_R);
    recording = 1;
}

static void record_stop(void){
    recording = 0;
}

uint8_t calib_moves(cal_test_t test){
    return test != CAL_NOISE && test != CAL_DUMP;
}

// ---- Dump ----------------------------------------------------------------------------

// Waits for a free UART slot while the console keeps working. 0 on STOP.
static uint8_t wait_slot(void){
    while(uart_tx_full()){
        commands_poll();
        if(motion_abort_requested()) return 0;
    }
    return !motion_abort_requested();
}

static void dump(void){
    char kp[12], kd[12], ke[12];
    if(!count){
        print("CAL: no hay datos grabados\n");
        return;
    }
    // Every constant the analysis may need, so each file stands on its own.
    if(!wait_slot()) return;
    print("@D BEGIN %s\n", description);
    if(!wait_slot()) return;
    print("@D INFO period_ms=%u samples=%u capacity=%u result=%s build=\"%s %s\"\n",
          period_ms, count, CAL_CAPACITY, outcome, __DATE__, __TIME__);
    if(!wait_slot()) return;
    print("@D INFO ticks_per_mm=%u cell_ticks=%u move_extra_ticks=%u ticks_per_turn=%u turn_settle_ms=%u\n",
          TICKS_PER_MM, CELL_TICKS, MOVE_EXTRA_TICKS, TICKS_PER_TURN, TURN_SETTLE_MS);
    if(!wait_slot()) return;
    print("@D INFO spd=%d fast=%d turn=%d kp=%s kd=%s ke=%s pd_max=%u accel_step_per_ms=%u\n",
          params.search_speed, params.fast_speed, params.turn_speed, format_fixed2(kp, sizeof(kp), params.kp),
          format_fixed2(kd, sizeof(kd), params.kd), format_fixed2(ke, sizeof(ke), params.ke),
          PD_STRAIGHT_MAX, ACCEL_STEP_PER_MS);
    if(!wait_slot()) return;
    print("@D INFO wall_detect_mm=%u front_ref_mm=%u front_emergency_mm=%u side_track_mm=%u lane_mm=%u\n",
          WALL_DETECT_MM, FRONT_WALL_REF_MM, FRONT_EMERGENCY_MM, SIDE_WALL_TRACK_MM, (unsigned)LANE_WIDTH_MM);
    static const char *const NAME[IR_COUNT] = {"fl", "fr", "sl", "sr"};
    for(uint8_t i = 0; i < IR_COUNT; i++){
        if(!wait_slot()) return;
        print("@D INFO ir_cal_%s=\"%s\"\n", NAME[i], ir_calibration_text((ir_sensor_t)i));
    }
    if(!wait_slot()) return;
    print("@D COLS t_ms,enc_l,enc_r,pwm_l,pwm_r,raw_fl,raw_fr,raw_sl,raw_sr\n");
    for(uint16_t i = 0; i < count; i++){
        const sample_t *s = &samples[i];
        if(!wait_slot()){
            print("CAL: envio interrumpido (CAL DUMP lo repite)\n");
            return;
        }
        print("@D %lu,%d,%d,%d,%d,%u,%u,%u,%u\n", (unsigned long)i * period_ms, s->enc_l, s->enc_r,
              s->pwm_l, s->pwm_r, s->ir[IR_FL], s->ir[IR_FR], s->ir[IR_SL], s->ir[IR_SR]);
    }
    if(!wait_slot()) return;
    print("@D END result=%s samples=%u\n", outcome, count);
    print("CAL: %u muestras enviadas (%s)\n", count, description);
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
            record_start(a > 1 ? 10 : 5);
            r = motion_forward((uint8_t)a, (int16_t)b);
            break;
        case CAL_TURN:
            snprintf(description, sizeof(description), "turn %ld", (long)a);
            record_start(5);
            r = motion_turn((int8_t)a);
            break;
        case CAL_STEP:
            snprintf(description, sizeof(description), "step %ld %ld", (long)a, (long)b);
            record_start(((uint32_t)b + CAL_COAST_MS + CAL_CAPACITY - 1) / CAL_CAPACITY);
            r = run_step((int16_t)a, (uint32_t)b);
            break;
        case CAL_IR:
            snprintf(description, sizeof(description), "ir %ld", (long)a);
            record_start(10);
            r = motion_drive_straight(-DRIFT_CORRECT_SPEED, a * TICKS_PER_MM);
            break;
        case CAL_DUMP:
            dump();
            return;
    }
    motion_stop();
    if(calib_moves(test) && r != MOVE_ABORTED) motion_wait(CAL_COAST_MS);     // coasting, overshoot
    record_stop();
    outcome = move_result_name(r);
    print("CAL %s: %s, %u muestras cada %u ms%s\n", description, outcome, count, period_ms,
          count >= CAL_CAPACITY ? " (buffer lleno: prueba mas corta)" : "");
    dump();
}
