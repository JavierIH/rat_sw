#ifndef PARAMS_H
#define PARAMS_H

#include <stdint.h>

// Runtime-tunable parameters: changed live over Bluetooth (SPD, FAST, ACCEL,
// TURN, TACCEL, TURNTICKS, KP, KI, LOG, TELEM) and persisted with SAVE.
// Defaults in robot_config.h. Speeds are physical units: the speed control
// (control.h) makes the wheels follow them whatever the battery.
typedef struct {
    float kp;               // centring: deg of heading per mm off-centre
    float ki;               // centring: deg per mm off-centre per m travelled (0 = off)
    int16_t search_speed;   // mm/s cruise of search moves, wall following and the speed run's return
    int16_t fast_speed;     // mm/s cruise of speed-run straights
    int16_t accel;          // mm/s^2 of every straight, speeding up and braking
    int16_t turn_speed;     // deg/s peak of in-place turns
    int16_t turn_accel;     // deg/s^2 of in-place turns
    int16_t turn_ticks;     // encoder half-difference of a real 90 deg turn (wheel track)
    uint8_t log_level;      // 0 = events, 1 = + decisions, 2 = + per-move details
    uint8_t telemetry;      // 1 = '@' lines for the live maze view (telemetry.h)
    uint8_t reserved[2];
} params_t;

// Accepted ranges, for the console and the stored copy.
#define SPEED_MIN       50      // mm/s
#define SPEED_MAX       2000
#define ACCEL_MIN       500     // mm/s^2
#define ACCEL_MAX       20000
#define TURN_SPEED_MIN  50      // deg/s
#define TURN_SPEED_MAX  2000
#define TURN_ACCEL_MIN  500     // deg/s^2
#define TURN_ACCEL_MAX  30000

extern params_t params;

void params_reset(void);

// Fingerprint of the compiled-in parameter and goal defaults. Data saved in
// flash is only reused by firmware built with the same defaults, so
// reflashing with new defaults (or a new goal) never runs on stale settings.
uint32_t params_defaults_signature(void);

#endif // PARAMS_H
