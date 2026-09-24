#include "control.h"
#include <math.h>

static float clampf(float v, float lo, float hi){
    return v < lo ? lo : v > hi ? hi : v;
}

static float absf(float v){
    return v < 0.0f ? -v : v;
}

// ---- Motion profile ------------------------------------------------------------------

#define PROFILE_DONE_EPS 0.01f     // closer than this to the target = arrived

void profile_reset(profile_t *p){
    p->pos = p->delta = p->speed = p->accel = p->target = 0.0f;
    p->top = p->final = 0.0f;
    p->rate = 1.0f;
    p->dir = 1;
    p->active = 0;
}

void profile_start(profile_t *p, float distance, float top, float final, float rate){
    const float speed = p->speed;   // a moving profile continues from its current speed
    profile_reset(p);
    p->speed = speed;
    p->target = distance;
    p->dir = distance < 0.0f ? -1 : 1;
    p->top = absf(top);
    p->final = absf(final);
    p->rate = rate > 0.0f ? rate : 1.0f;
    p->active = absf(distance) > PROFILE_DONE_EPS;
}

float profile_remaining(const profile_t *p){
    float rem = (p->target - p->pos) * (float)p->dir;
    return rem > 0.0f ? rem : 0.0f;
}

void profile_resume(profile_t *p, float pos){
    p->pos = pos;
    p->speed = p->accel = p->delta = 0.0f;
    p->active = profile_remaining(p) > PROFILE_DONE_EPS;
}

void profile_step(profile_t *p, float dt){
    const float before = p->pos, speed_before = p->speed;
    if(p->active){
        const float dir = (float)p->dir;
        const float rem = (p->target - p->pos) * dir;
        const float v = p->speed * dir;
        if(rem <= PROFILE_DONE_EPS){
            p->pos = p->target;
            p->speed = p->final * dir;
            p->active = 0;
        }
        else{
            float next = v < p->top ? fminf(v + p->rate * dt, p->top) : fmaxf(v - p->rate * dt, p->top);
            // Fastest speed from which the target can still be reached at
            // `final` braking at `rate`, counted from where this step ends:
            // next^2 = final^2 + 2 rate (rem - (v + next) dt / 2). Solved for
            // `next`, it brakes at exactly `rate` every step. If the target
            // moved closer, brake up to twice as hard; never reverse.
            const float a_dt = p->rate * dt;
            const float disc = a_dt * a_dt + 4.0f * (p->final * p->final + 2.0f * p->rate * (rem - 0.5f * v * dt));
            next = fminf(next, disc > 0.0f ? 0.5f * (sqrtf(disc) - a_dt) : 0.0f);
            next = fmaxf(next, fmaxf(v - 2.0f * a_dt, 0.0f));
            float step = 0.5f * (v + next) * dt;
            if(step >= rem){
                p->pos = p->target;
                p->speed = p->final * dir;
                p->active = 0;
            }
            else{
                p->pos += step * dir;
                p->speed = next * dir;
            }
        }
    }
    else if(p->final == 0.0f){
        p->speed = 0.0f;
    }
    p->delta = p->pos - before;
    p->accel = (p->speed - speed_before) / dt;
}

// ---- Wheel controllers ---------------------------------------------------------------

void control_reset(control_t *c){
    c->fwd_error = c->rot_error = c->rot_integral = c->steer_prev = 0.0f;
    c->settle_fwd = c->settle_rot = 0.0f;
    c->still_fwd = c->still_rot = 0;
    for(uint8_t i = 0; i < CONTROL_D_WINDOW; i++) c->fwd_hist[i] = c->rot_hist[i] = 0.0f;
    c->slot = 0;
    c->saturated = 0;
    c->pwm_l = c->pwm_r = 0;
}

void control_clear_errors(control_t *c){
    c->fwd_error = c->rot_error = 0.0f;
    for(uint8_t i = 0; i < CONTROL_D_WINDOW; i++) c->fwd_hist[i] = c->rot_hist[i] = 0.0f;
}

// PWM that makes a wheel run at `v` while accelerating at `a` (motor model
// from CAL STEP). The friction term fades in over the first 10 mm/s so that
// it does not chatter around standstill.
static float feedforward(float v, float a, float kv, const control_config_t *k){
    return kv * (v + k->tau * a) + k->ks * clampf(v * 0.1f, -1.0f, 1.0f);
}

void control_step(control_t *c, const control_config_t *k, const profile_t *fwd, const profile_t *rot,
                  float steer, int32_t dl, int32_t dr, float dt){
    // Measured motion of this step, in the units of each axis.
    const float ml = (float)dl * (1.0f + 0.5f * k->wheel_diff), mr = (float)dr * (1.0f - 0.5f * k->wheel_diff);
    const float fwd_moved = 0.5f * (ml + mr) / k->ticks_per_mm;
    const float rot_moved = 0.5f * (ml - mr) / k->ticks_per_mm / k->mm_per_deg;
    c->fwd_error += fwd->delta - fwd_moved;
    c->rot_error += rot->delta + (steer - c->steer_prev) - rot_moved;
    c->steer_prev = steer;

    // Error change over the last CONTROL_D_WINDOW steps: a speed error with
    // a quarter of the quantization noise of a single step.
    const float fwd_rate = (c->fwd_error - c->fwd_hist[c->slot]) / (CONTROL_D_WINDOW * dt);
    const float rot_rate = (c->rot_error - c->rot_hist[c->slot]) / (CONTROL_D_WINDOW * dt);
    c->fwd_hist[c->slot] = c->fwd_error;
    c->rot_hist[c->slot] = c->rot_error;
    c->slot = (uint8_t)((c->slot + 1u) % CONTROL_D_WINDOW);

    // The integral (the motors' imbalance) only learns while the output has
    // room, or it would wind up against the limit.
    if(!c->saturated){
        c->rot_integral = clampf(c->rot_integral + k->rot_ki * c->rot_error * dt, -k->rot_i_max, k->rot_i_max);
    }
    // Arrived and stalled: the last mm / degree need a push the
    // proportional part does not give against static friction. It builds up
    // only while that axis is stuck, and drops the moment it moves: kept, it
    // came out all at once when the wheels broke free (a turn overshot 2.7
    // deg), and building it while they still coasted made a slow motor
    // overshoot too.
    const uint8_t arrived = !fwd->active && !rot->active;
    if(dl + dr != 0){
        c->still_fwd = 0;
        c->settle_fwd = 0.0f;
    }
    else if(c->still_fwd < 255u) c->still_fwd++;
    if(dl - dr != 0){
        c->still_rot = 0;
        c->settle_rot = 0.0f;
    }
    else if(c->still_rot < 255u) c->still_rot++;
    if(arrived && c->still_fwd >= CONTROL_STILL_STEPS){
        c->settle_fwd = clampf(c->settle_fwd + k->settle_ki_fwd * c->fwd_error * dt, -k->settle_i_max, k->settle_i_max);
    }
    if(arrived && c->still_rot >= CONTROL_STILL_STEPS){
        c->settle_rot = clampf(c->settle_rot + k->settle_ki_rot * c->rot_error * dt, -k->settle_i_max, k->settle_i_max);
    }
    const float u_fwd = k->fwd_kp * c->fwd_error + k->fwd_kd * fwd_rate + c->settle_fwd;
    const float u_rot = k->rot_kp * c->rot_error + k->rot_kd * rot_rate + c->rot_integral + c->settle_rot;

    // Reference motion of each wheel (right turn: left wheel forward).
    const float v_l = fwd->speed + rot->speed * k->mm_per_deg, v_r = fwd->speed - rot->speed * k->mm_per_deg;
    const float a_l = fwd->accel + rot->accel * k->mm_per_deg, a_r = fwd->accel - rot->accel * k->mm_per_deg;
    const float ff_l = feedforward(v_l, a_l, k->kv_l, k), ff_r = feedforward(v_r, a_r, k->kv_r, k);

    // Common and differential parts. Out of room, the rotation keeps its
    // share and the forward drive gives way: a robot that runs a bit slow
    // stays straight, one that loses its heading hits a wall.
    float common = 0.5f * (ff_l + ff_r) + u_fwd;
    float diff = clampf(0.5f * (ff_l - ff_r) + u_rot, -k->pwm_limit, k->pwm_limit);
    const float room = k->pwm_limit - absf(diff);
    c->saturated = absf(common) > room;
    common = clampf(common, -room, room);
    c->pwm_l = (int16_t)lroundf(common + diff);
    c->pwm_r = (int16_t)lroundf(common - diff);
}

// ---- Wall centring ---------------------------------------------------------------------

void steer_reset(steer_t *s){
    s->lateral = s->bias = s->heading = s->drift = 0.0f;
    for(uint8_t i = 0; i < STEER_DELAY_MAX; i++) s->drift_hist[i] = 0.0f;
    s->reading = s->reading_sum = 0.0f;
    for(uint8_t i = 0; i < STEER_AVERAGE_MAX; i++) s->reading_hist[i] = 0.0f;
    s->slot = s->reading_slot = 0;
    s->wall = STEER_WALL_NONE;
    s->valid = 0;
}

// The heading offset is proportional to the lateral error, which makes the
// correction a matter of distance, not time: the robot converges within the
// same ~distance at 200 mm/s and at 1000 mm/s, and the position loops keep
// the heading however unequal the motors are. The integral learns how far
// the encoder heading of this move is from the corridor (the error a turn
// leaves), which P alone would turn into a steady offset from the centre.
//
// The side IR report where the robot was IR_DELAY_MS ago (25 mm at 500
// mm/s): steering on that alone weaves at speed. The encoders know how far
// the robot moved sideways since (distance times heading to the corridor),
// and adding that brings the reading up to date (a Smith predictor).
float steer_step(steer_t *s, const steer_config_t *k, float sl_mm, float sr_mm, float ds_mm, float heading_deg,
                 float gain){
    const float to_corridor = (heading_deg - s->bias) * (3.14159265f / 180.0f);    // rad, > 0 heading right
    const float dy = -ds_mm * to_corridor;          // heading right: the robot moves right, lateral decreases
    const uint8_t n = k->delay_steps < STEER_DELAY_MAX ? k->delay_steps : STEER_DELAY_MAX;
    if(n){
        s->drift += dy - s->drift_hist[s->slot];
        s->drift_hist[s->slot] = dy;
        s->slot = (uint8_t)((s->slot + 1u) % n);
    }
    const float error_r = sr_mm - k->center_r_mm, error_l = k->center_l_mm - sl_mm;
    uint8_t right = sr_mm < k->track_mm, left = sl_mm < k->track_mm;
    // Both walls: their average (half the noise, and no bias from one
    // sensor), unless one reading is implausible (the angled beam catching a
    // post or a wall ahead) and the other agrees better with where the robot
    // can be.
    if(right && left){
        if(absf(error_r) > k->error_max_mm && absf(error_l) < absf(error_r)) right = 0;
        else if(absf(error_l) > k->error_max_mm && absf(error_r) < absf(error_l)) left = 0;
    }

    float want = s->bias;
    if(right || left){
        const float raw = right && left ? 0.5f * (error_r + error_l) : right ? error_r : error_l;
        const float error = clampf(raw, -k->error_max_mm, k->error_max_mm);
        s->wall = right && left ? STEER_WALL_BOTH : right ? STEER_WALL_RIGHT : STEER_WALL_LEFT;
        // A post or a wall edge makes the reading jump further in 1 ms than
        // the robot can move sideways: follow it at a limited rate, so a
        // short glitch barely moves the estimate and a real change is
        // tracked a few ms later. Then average over one sensor period: the
        // sensors change in ~2 mm steps every ~16 ms, and each step used to
        // kick the heading.
        const uint8_t avg_n = k->average_steps < 1 ? 1
                            : k->average_steps > STEER_AVERAGE_MAX ? STEER_AVERAGE_MAX : k->average_steps;
        if(!s->valid){
            s->reading = error;
            s->reading_sum = error * (float)avg_n;
            for(uint8_t i = 0; i < avg_n; i++) s->reading_hist[i] = error;
        }
        else{
            s->reading += clampf(error - s->reading, -k->slew_mm, k->slew_mm);
        }
        s->reading_sum += s->reading - s->reading_hist[s->reading_slot];
        s->reading_hist[s->reading_slot] = s->reading;
        s->reading_slot = (uint8_t)((s->reading_slot + 1u) % avg_n);
        s->valid = 1;
        s->lateral = s->reading_sum / (float)avg_n + s->drift;
        // Only near the centre: during a big correction the error is the
        // P part's business, and integrating it there made the robot
        // overshoot the centre line.
        // Nor while the heading offset is at its clamp (it would wind up).
        const float unclamped = gain * k->kp * s->lateral + s->bias;
        if(absf(s->lateral) < k->bias_window_mm && absf(unclamped) < k->max_deg){
            s->bias = clampf(s->bias + k->ki * s->lateral * ds_mm, -k->max_deg, k->max_deg);
        }
        want = clampf(gain * k->kp * s->lateral + s->bias, -k->max_deg, k->max_deg);
    }
    else{
        // No wall: hold the heading, corrected by what the walls taught.
        s->wall = STEER_WALL_NONE;
        s->valid = 0;
    }
    // Limited per mm travelled, not per second: a gentle curve at any speed,
    // and no pivoting at the start of a move when the robot barely moves.
    const float step = k->curve_deg * ds_mm;
    s->heading += clampf(want - s->heading, -step, step);
    return s->heading;
}
