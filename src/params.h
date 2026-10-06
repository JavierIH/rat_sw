#ifndef PARAMS_H
#define PARAMS_H

#include <stdint.h>

// Runtime parameters, changed over Bluetooth and saved with SAVE (defaults in robot_config.h); physical units.
typedef struct {
    float kp;               // centring: deg/s of turn per mm off-centre
    float kd;               // centring: deg of turn per mm the error changes
    int16_t search_speed;   // mm/s cruise of search moves, wall following and the speed run's return
    int16_t fast_speed;     // mm/s cruise of speed-run straights
    int16_t accel;          // mm/s^2 of every straight, speeding up and braking
    int16_t turn_speed;     // deg/s peak of in-place turns
    int16_t turn_accel;     // deg/s^2 of in-place turns
    int16_t turn_ticks;     // encoder half-difference of a real 90 deg turn (wheel track)
    uint8_t log_level;      // 0 = events, 1 = + decisions, 2 = + per-move details
    uint8_t telemetry;      // 1 = '@' lines for the live maze view (telemetry.h)
    int16_t curve_speed;    // mm/s through the smooth curves of the speed run (path.h)
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

// Fingerprint of the parameter defaults: saved parameters are reused only by a firmware with the same defaults.
uint32_t params_defaults_signature(void);

#endif // PARAMS_H
