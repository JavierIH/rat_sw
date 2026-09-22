#ifndef PARAMS_H
#define PARAMS_H

#include <stdint.h>

// Runtime-tunable parameters: changed live over Bluetooth (SPD, FAST, TURN,
// KP, KD, KE, LOG, TELEM) and persisted with SAVE. Defaults in robot_config.h.
typedef struct {
    float kp;               // steering PD, PWM per mm of lateral error
    float kd;               // steering PD, PWM per mm of error change per 10 ms
    float ke;               // heading hold without side walls, PWM per encoder tick (0 = off)
    int16_t search_speed;   // PWM of search moves and of the final approach of every straight
    int16_t fast_speed;     // PWM cruise of speed-run straights
    int16_t turn_speed;     // PWM of in-place turns
    uint8_t log_level;      // 0 = events, 1 = + decisions, 2 = + per-move details
    uint8_t telemetry;      // 1 = '@' lines for the live maze view (telemetry.h)
} params_t;

extern params_t params;

void params_reset(void);

// Fingerprint of the compiled-in parameter and goal defaults. Data saved in
// flash is only reused by firmware built with the same defaults, so
// reflashing with new defaults (or a new goal) never runs on stale settings.
uint32_t params_defaults_signature(void);

#endif // PARAMS_H
