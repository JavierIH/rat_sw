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

float control_sqrt(float x){
    if(!(x > 0.0f)) return 0.0f;
    // Exponent halved (~6 %), then three Newton steps to float precision.
    union { float f; uint32_t u; } g = {x};
    g.u = (g.u >> 1) + 0x1FC00000u;
    float r = g.f;
    for(uint8_t i = 0; i < 3; i++) r = 0.5f * (r + x / r);
    return r;
}

// Fastest end-of-step speed that still reaches `rem` at `final` braking at `rate`.
float profile_brake_speed(float v, float rem, float final, float rate, float dt){
    const float a_dt = rate * dt;
    const float disc = a_dt * a_dt + 4.0f * (final * final + 2.0f * rate * (rem - 0.5f * v * dt));
    return disc > 0.0f ? 0.5f * (control_sqrt(disc) - a_dt) : 0.0f;
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
            // If the target moved closer, brake up to twice as hard; never reverse.
            next = fminf(next, profile_brake_speed(v, rem, p->final, p->rate, dt));
            next = fmaxf(next, fmaxf(v - 2.0f * p->rate * dt, 0.0f));
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

// PWM for a wheel at `v` accelerating at `a` (CAL STEP model); friction fades in over 10 mm/s.
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

    // Error change over CONTROL_D_WINDOW steps: a speed error with 1/4 of a step's quantization noise.
    const float fwd_rate = (c->fwd_error - c->fwd_hist[c->slot]) / (CONTROL_D_WINDOW * dt);
    const float rot_rate = (c->rot_error - c->rot_hist[c->slot]) / (CONTROL_D_WINDOW * dt);
    c->fwd_hist[c->slot] = c->fwd_error;
    c->rot_hist[c->slot] = c->rot_error;
    c->slot = (uint8_t)((c->slot + 1u) % CONTROL_D_WINDOW);

    // The integral (motor imbalance) learns only while the output has room (no wind-up).
    if(!c->saturated){
        c->rot_integral = clampf(c->rot_integral + k->rot_ki * c->rot_error * dt, -k->rot_i_max, k->rot_i_max);
    }
    // Stuck short of the target: a push against static friction, built while stuck, dropped once it moves.
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

    // Out of PWM the rotation keeps its share and the forward drive gives way (slow beats crooked).
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
    s->heading = 0.0f;
    steer_restart(s);
}

void steer_restart(steer_t *s){
    s->error = s->sum = 0.0f;
    for(uint8_t i = 0; i < STEER_AVERAGE_MAX; i++) s->hist[i] = 0.0f;
    s->slot = 0;
    s->wall = STEER_WALL_NONE;
    s->valid = 0;
}

// Wall follower: the turn rate is a PD of the lateral error.
float steer_step(steer_t *s, const steer_config_t *k, float sl_mm, float sr_mm, float dt, float gain){
    const uint8_t right = sr_mm < k->track_mm, left = sl_mm < k->track_mm;
    const uint8_t walls = right && left ? STEER_WALL_BOTH : right ? STEER_WALL_RIGHT
                        : left ? STEER_WALL_LEFT : STEER_WALL_NONE;
    if(walls != s->wall) s->valid = 0;          // another reference: no derivative across the change
    s->wall = walls;
    if(walls == STEER_WALL_NONE) return s->heading;
    const float error_r = sr_mm - k->center_r_mm, error_l = k->center_l_mm - sl_mm;
    const float raw = clampf(walls == STEER_WALL_BOTH ? 0.5f * (error_r + error_l) : right ? error_r : error_l,
                             -k->error_max_mm, k->error_max_mm);
    // Averaged over the sensors' period (they step ~2 mm every ~16 ms).
    const uint8_t n = k->average_steps < 1 ? 1 : k->average_steps > STEER_AVERAGE_MAX ? STEER_AVERAGE_MAX
                    : k->average_steps;
    if(!s->valid){
        for(uint8_t i = 0; i < n; i++) s->hist[i] = raw;
        s->sum = raw * (float)n;
        s->error = raw;
        s->slot = 0;
        s->valid = 1;
    }
    s->sum += raw - s->hist[s->slot];
    s->hist[s->slot] = raw;
    s->slot = (uint8_t)((s->slot + 1u) % n);
    const float error = s->sum / (float)n;
    s->heading += gain * (k->kp * error * dt + k->kd * (error - s->error));    // > 0: to the right
    s->error = error;
    return s->heading;
}
