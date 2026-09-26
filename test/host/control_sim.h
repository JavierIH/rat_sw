#ifndef CONTROL_SIM_H
#define CONTROL_SIM_H

#include <stdint.h>
#include "path.h"

// The firmware's speed control (control.c, robot_config.h constants) driving
// a simulated robot in a corridor: first-order motors with friction and dead
// time, quantized encoders, noisy angled side IR. Used by the host tests and
// by `host_tests --control` to tune the gains away from the robot.

typedef struct {
    float gain_l, gain_r;       // real mm/s per PWM relative to the model (1 = as MOTOR_KV_*)
    float tau;                  // real motor time constant, s
    float friction, stiction;   // PWM lost to friction moving / needed to start (each wheel)
    float yaw_friction;         // PWM the chassis (skids) takes from a change of heading...
    float yaw_stiction;         // ...and needs to start turning at all (measured: heading stuck up to ~50-80)
    int dead_ms;                // delay between the PWM and the motor
    float ir_noise;             // side IR noise, mm (standard deviation)
    int ir_delay_ms;            // side IR delay (the robot's Sharp-type sensors: ~50 ms)...
    int ir_period_ms;           // ...a new value this often...
    float ir_step_mm;           // ...in steps of about this much
    float y0;                   // start: mm left of the centre line
    float yaw0;                 // start: deg to the right of the corridor
    float wall_error_mm;        // each wall of each cell off by up to this much (a real maze: ~3, up to 7)
    uint32_t seed;
} plant_t;

typedef struct {
    float travelled;            // true distance, mm
    float turned;               // true rotation, deg
    float y_end;                // mm left of the centre at the end
    float y_late;               // largest |y| over the second half of the move
    float yaw_end;              // deg to the right of the corridor at the end
    float bias_end;             // deg: the centring's idea of the encoder heading parallel to the walls...
    float bias_true;            // ...and the real one
    float fwd_err_max, rot_err_max;
    int crossings;              // sign changes of y (weaving)
    int pwm_max;                // largest |PWM| asked
    uint32_t ms;                // until the move settled
} sim_result_t;

// A whole path (path.h) without walls, so without centring: how closely the
// wheels follow the curves. Maze frame: start cell centre at the origin.
typedef struct {
    float end_err;              // mm from where the robot stops to where the reference ends
    float heading_err;          // deg, true heading at the end minus the reference's
    float cross_err_max;        // mm, largest distance from the robot to the reference's path (not to where the reference is)
    float fwd_err_max, rot_err_max;
    int pwm_max;
    uint32_t ms;                // until it settled
    float scale_min;            // lowest time scale of the reference (1: it never waited for the robot)
    uint32_t stall_ms;          // when the robot fell FWD_ERROR_MAX_MM behind (MOVE_STALLED), 0 if never
} path_result_t;

extern float sim_average;       // side IR averaging, ms (default STEER_AVERAGE_MS)
extern float sim_window;        // KI learning window, mm (default STEER_BIAS_WINDOW_MM)
extern float sim_curve;         // centring curvature limit, deg/mm (default STEER_CURVE_DEG_PER_MM)
extern float sim_observer;      // bias observer distance, mm (default STEER_OBSERVER_MM; 0: the old integral)
extern float sim_vref;          // KP falls as 1/speed above this, mm/s (default STEER_VREF_MM_S)

plant_t plant_nominal(void);
sim_result_t sim_straight(const plant_t *p, float mm, float speed, float accel, float kp, float ki);
sim_result_t sim_turn(const plant_t *p, float deg, float speed, float accel);
path_result_t sim_path(const plant_t *p, const run_path_t *path, const curve_t *curve, float v_straight,
                       float v_curve, float accel);

#endif // CONTROL_SIM_H
