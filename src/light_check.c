#include "light_check.h"
#include <math.h>
#include <stddef.h>
#include "commands.h"
#include "control.h"
#include "infrared.h"
#include "motion.h"
#include "robot_config.h"
#include "uart.h"

#define STILL_MS        100u    // after a turn, before sampling
#define SAMPLES         1000u   // one a ms, stopped at each heading
#define OFFSET_RANGE    1000.0f // counts either way: the bisection's span
#define SATURATED_RAW   4000u   // near the ADC's 4095: no offset can be read

enum { H_N, H_E, H_S, H_W, HEADINGS };

// Per sensor: the headings that face opposite walls (the hand placement cancels in their mean) and open space.
typedef struct { uint8_t wall_a, wall_b, open; float ref_mm; } plan_t;
static const plan_t PLAN[IR_COUNT] = {
    [IR_FL] = {H_E, H_W, H_N, FRONT_WALL_REF_MM + 0.5f * FRONT_SQUARE_OFFSET_MM},
    [IR_FR] = {H_E, H_W, H_N, FRONT_WALL_REF_MM - 0.5f * FRONT_SQUARE_OFFSET_MM},
    [IR_SL] = {H_N, H_S, H_E, SIDE_CENTER_L_MM},
    [IR_SR] = {H_N, H_S, H_W, SIDE_CENTER_R_MM},
};
static const char *const NAME[IR_COUNT] = {"FL", "FR", "SL", "SR"};
static const char HEADING[HEADINGS] = {'N', 'E', 'S', 'W'};

// Mean and standard deviation of every sensor's raw counts, shifted by the first sample to keep the floats' precision.
static uint8_t sample(float mean[IR_COUNT], float sd[IR_COUNT], uint16_t peak[IR_COUNT]){
    float base[IR_COUNT], sum[IR_COUNT] = {0.0f}, sq[IR_COUNT] = {0.0f};
    for(uint8_t s = 0; s < IR_COUNT; s++) base[s] = (float)ir_raw((ir_sensor_t)s);
    for(uint32_t i = 0; i < SAMPLES; i++){
        if(!motion_wait(1)) return 0;
        for(uint8_t s = 0; s < IR_COUNT; s++){
            const uint16_t r = ir_raw((ir_sensor_t)s);
            if(r > peak[s]) peak[s] = r;
            const float d = (float)r - base[s];
            sum[s] += d;
            sq[s] += d * d;
        }
    }
    for(uint8_t s = 0; s < IR_COUNT; s++){
        const float m = sum[s] / (float)SAMPLES;
        mean[s] = base[s] + m;
        sd[s] = control_sqrt(fmaxf(sq[s] / (float)SAMPLES - m * m, 0.0f));
    }
    return 1;
}

// The counts that, taken off both readings, make their mean read ref_mm (bisection: mm falls as raw grows).
static float offset_for(ir_sensor_t s, float a, float b, float ref_mm){
    float lo = -OFFSET_RANGE, hi = OFFSET_RANGE;
    for(uint8_t i = 0; i < 20; i++){
        const float o = 0.5f * (lo + hi);
        if(0.5f * (ir_mm_of(s, a - o) + ir_mm_of(s, b - o)) < ref_mm) lo = o;
        else hi = o;
    }
    return 0.5f * (lo + hi);
}

// One sensor's report line and verdict.
static light_verdict_t judge(ir_sensor_t s, float mean[HEADINGS][IR_COUNT], float sd[HEADINGS][IR_COUNT], uint16_t peak){
    const plan_t *p = &PLAN[s];
    const float a = mean[p->wall_a][s], b = mean[p->wall_b][s], open = mean[p->open][s];
    const float shift = 0.5f * (ir_mm_of(s, a) + ir_mm_of(s, b)) - p->ref_mm;     // < 0: walls read close
    const float o = offset_for(s, a, b, p->ref_mm), mid = 0.5f * (a + b);
    const float noise = fmaxf(sd[p->wall_a][s], sd[p->wall_b][s]) * 0.5f * fabsf(ir_mm_of(s, mid - 1.0f) - ir_mm_of(s, mid + 1.0f));
    const float open_mm = ir_mm_of(s, open), fixed_mm = ir_mm_of(s, open - o), open_min = WALL_DETECT_MM + LIGHT_OPEN_MARGIN_MM;
    const uint8_t walls = ir_mm_of(s, a) < WALL_DETECT_MM && ir_mm_of(s, b) < WALL_DETECT_MM;
    const char *why = NULL;
    light_verdict_t v = LIGHT_OK;
    if(!walls) why = "a wall missing: not in the start cell, facing north?";
    else if(peak >= SATURATED_RAW) why = "saturated";
    else if(noise > LIGHT_NOISE_MAX_MM) why = "noisy (flickering light?)";
    else if(fixed_mm < open_min) why = "open space reads as a wall even corrected";
    else if(fabsf(shift) >= LIGHT_OK_MM || open_mm < open_min) v = LIGHT_CORRECT;
    if(why) v = LIGHT_FAIL;
    char t[4][12];
    print("%s: walls %d/%d shift %smm (%d counts) noise %smm open %dmm (%dmm corrected) %s\n", NAME[s], (int)a, (int)b,
          format_fixed(t[0], sizeof(t[0]), shift, 1), (int)lroundf(o), format_fixed(t[1], sizeof(t[1]), noise, 1),
          (int)open_mm, (int)fixed_mm, v == LIGHT_OK ? "OK" : v == LIGHT_CORRECT ? "CORRECT" : "FAIL");
    if(why) print("  %s (walls at headings %c and %c)\n", why, HEADING[p->wall_a], HEADING[p->wall_b]);
    return v;
}

light_verdict_t light_check(void){
    float mean[HEADINGS][IR_COUNT], sd[HEADINGS][IR_COUNT];
    uint16_t peak[IR_COUNT] = {0};
    for(uint8_t h = 0; h < HEADINGS; h++){
        if(h && motion_turn(1) != MOVE_OK) return LIGHT_ABORTED;
        if(!motion_wait(STILL_MS) || !sample(mean[h], sd[h], peak)) return LIGHT_ABORTED;
    }
    if(motion_turn(1) != MOVE_OK) return LIGHT_ABORTED;    // facing north again
    light_verdict_t worst = LIGHT_OK;
    for(uint8_t s = 0; s < IR_COUNT; s++){
        const light_verdict_t v = judge((ir_sensor_t)s, mean, sd, peak[s]);
        if(v > worst) worst = v;
    }
    print("light: %s\n", worst == LIGHT_OK ? "OK" : worst == LIGHT_CORRECT
          ? "shifted: needs a correction (not applied yet, docs/lighting.md)" : "FAIL");
    return worst;
}
