#ifndef CONTROL_H
#define CONTROL_H

#include <stdint.h>

// Speed control, pure C (no HAL): motion profiles, the position loops that
// make the wheels follow them, and the wall centring. motion.c runs it every
// millisecond from SysTick with the real encoders, IR and motors; the host
// tests run the same code against a simulated robot.
//
// Units: mm, mm/s, mm/s^2 on the forward axis; degrees on the rotation axis,
// positive clockwise (a right turn); PWM duty at the output.
//
// Why position loops: at 9 ticks/mm one encoder tick in 1 ms is 111 mm/s, so
// a speed measured every millisecond is mostly quantization noise. Summing
// the reference and the encoder increments instead gives the error in
// position, exact to one tick (0.11 mm), and its change over a few ms gives a
// usable speed error for the damping term.

// ---- Motion profile ------------------------------------------------------------------

// Trapezoidal profile: speeds up at `rate` to `top`, cruises, and slows down
// at `rate` so that it arrives at `target` exactly at `final` speed. The
// target can move while running (a front wall seen by the IR); the braking
// then uses up to twice `rate`.
typedef struct {
    float pos;          // distance covered by the reference since the start
    float delta;        // pos change in the last step
    float speed;        // signed reference speed
    float accel;        // signed reference acceleration in the last step (feedforward)
    float target;       // end point, same units as pos
    float top;          // cruise speed (> 0)
    float final;        // speed at the target (>= 0)
    float rate;         // acceleration limit (> 0)
    int8_t dir;         // +1 / -1
    uint8_t active;     // still heading for the target
} profile_t;

void profile_reset(profile_t *p);   // at rest at 0
void profile_start(profile_t *p, float distance, float top, float final, float rate);
void profile_step(profile_t *p, float dt);
float profile_remaining(const profile_t *p);    // distance left, >= 0
// Continue from standstill at `pos` towards the same target (after a pause).
void profile_resume(profile_t *p, float pos);
// Highest speed `v` may reach at the end of a step of `dt` and still arrive
// `rem` ahead at `final`, braking at `rate` (the profiles' braking curve).
float profile_brake_speed(float v, float rem, float final, float rate, float dt);

// ---- Wheel controllers ---------------------------------------------------------------

typedef struct {
    float ticks_per_mm;     // encoder resolution
    float wheel_diff;       // left wheel travel per tick relative to the right one, minus 1 (diameter mismatch)
    float mm_per_deg;       // wheel travel per degree of in-place rotation (from TURNTICKS)
    float kv_l, kv_r;       // feedforward: PWM per mm/s of each wheel
    float tau;              // feedforward: motor time constant, s (PWM per mm/s^2 = kv * tau)
    float ks;               // feedforward: PWM that overcomes the friction while moving
    float fwd_kp, fwd_kd;   // forward loop: PWM per mm, PWM per mm/s
    float rot_kp, rot_kd;   // rotation loop: PWM per deg, PWM per deg/s
    float rot_ki;           // rotation loop: PWM per deg*s (motor imbalance)
    float rot_i_max;        // clamp of the integral, PWM
    float settle_ki_fwd;    // integrals while settling (profiles arrived): PWM per mm*s...
    float settle_ki_rot;    // ...and per deg*s, to beat the static friction
    float settle_i_max;     // clamp of those, PWM
    float pwm_limit;        // output clamp
} control_config_t;

#define CONTROL_D_WINDOW 4  // ms over which the error change (damping) is measured
#define CONTROL_STILL_STEPS 5   // steps without encoder motion = stalled (settling integrals)

typedef struct {
    float fwd_error;        // mm: reference minus measured
    float rot_error;        // deg: reference (profile + steering) minus measured
    float rot_integral;     // PWM
    float settle_fwd, settle_rot;   // settling integrals, PWM
    uint8_t still_fwd, still_rot;   // steps in a row without motion on each axis
    float steer_prev;       // heading offset of the last step, deg
    float fwd_hist[CONTROL_D_WINDOW];
    float rot_hist[CONTROL_D_WINDOW];
    uint8_t slot;
    uint8_t saturated;      // the last output hit pwm_limit
    int16_t pwm_l, pwm_r;   // last output
} control_t;

void control_reset(control_t *c);
// The robot is where it is: errors and damping history to zero, keeping the
// learned imbalance and the steering offset (after a pause).
void control_clear_errors(control_t *c);
// One control period. dl, dr: encoder ticks since the last call (forward
// positive); fwd, rot: profiles already stepped; steer: heading offset asked
// by the centring (deg, added to the rotation reference). Writes pwm_l/pwm_r.
void control_step(control_t *c, const control_config_t *k, const profile_t *fwd, const profile_t *rot,
                  float steer, int32_t dl, int32_t dr, float dt);

// ---- Wall centring ---------------------------------------------------------------------

typedef enum { STEER_WALL_NONE, STEER_WALL_RIGHT, STEER_WALL_LEFT, STEER_WALL_BOTH } steer_wall_t;

typedef struct {
    float kp;               // deg of heading per mm off-centre
    float ki;               // deg per mm off-centre per mm travelled (heading misalignment)...
    float bias_window_mm;   // ...learned only while closer to the centre than this
    float max_deg;          // clamp of the heading offset
    float curve_deg;        // max change of the heading offset per mm travelled (curvature)
    float slew_mm;          // max change of the lateral error per step (posts, wall edges)
    float track_mm;         // a side wall closer than this is a reference
    float center_l_mm;      // SL reading with the robot on the centre line
    float center_r_mm;      // SR reading with the robot on the centre line
    float error_max_mm;     // clamp of the lateral error
    uint8_t delay_steps;    // side IR delay in steps, averaging included (<= STEER_DELAY_MAX)
    uint8_t average_steps;  // side IR averaged over this many steps (<= STEER_AVERAGE_MAX)
} steer_config_t;

#define STEER_DELAY_MAX 84
#define STEER_AVERAGE_MAX 40

typedef struct {
    float lateral;          // filtered lateral error, mm (> 0: left of the centre line)
    float bias;             // integral part of the heading offset, deg
    float heading;          // heading offset asked for, deg (> 0: to the right)
    float drift;            // lateral motion over the last delay_steps (encoders), mm
    float drift_hist[STEER_DELAY_MAX];
    float reading;          // lateral error from the wall, slew-limited, mm
    float reading_sum;      // of the last average_steps readings
    float reading_hist[STEER_AVERAGE_MAX];
    uint8_t slot, reading_slot;
    uint8_t wall;           // steer_wall_t in use
    uint8_t valid;          // lateral holds a reading
} steer_t;

void steer_reset(steer_t *s);
// A new corridor (after a curve): forget the readings and the motion since,
// keep the heading offset and what the walls taught about the heading.
void steer_restart(steer_t *s);
// One control period: side readings (mm), distance travelled this step (mm,
// >= 0), heading now (deg since the move started, from the encoders) and
// how much of the centring to apply (1 cruising, fading to 0 at the end of a
// move so the robot stops parallel to the walls). Returns the heading offset
// for control_step().
float steer_step(steer_t *s, const steer_config_t *k, float sl_mm, float sr_mm, float ds_mm, float heading_deg,
                 float gain);

#endif // CONTROL_H
