#include "path.h"
#include <math.h>
#include "robot_config.h"

#define HALF_PI         1.57079633f
#define PATH_NONE       1.0e9f      // curve_start when no curve is left
#define PATH_DONE_EPS   0.01f       // closer than this to the end = arrived
#define FOOTPRINT_STEPS 64          // Simpson intervals for the footprint (even)

// ---- Curve shape ------------------------------------------------------------------------

// sin and cos on [0, pi/2] by their series (error < 1e-6): libm's sinf and
// cosf would add 4 KB of range reduction to the firmware for this.
static void sin_cos(float x, float *s, float *c){
    const float x2 = x * x;
    *s = x * (1.0f - x2 * (1.0f / 6.0f) * (1.0f - x2 * (1.0f / 20.0f) * (1.0f - x2 * (1.0f / 42.0f)
         * (1.0f - x2 * (1.0f / 72.0f) * (1.0f - x2 * (1.0f / 110.0f))))));
    *c = 1.0f - x2 * 0.5f * (1.0f - x2 * (1.0f / 12.0f) * (1.0f - x2 * (1.0f / 30.0f)
         * (1.0f - x2 * (1.0f / 56.0f) * (1.0f - x2 * (1.0f / 90.0f)))));
}

float curve_progress(const curve_t *c, float u){
    if(u <= 0.0f) return 0.0f;
    if(u >= c->length) return 1.0f;
    float heading;      // rad
    if(u < c->ramp){
        heading = 0.5f * c->k_ramp * u * u;
    }
    else if(u < c->length - c->ramp){
        heading = c->k * (u - 0.5f * c->ramp);
    }
    else{
        const float left = c->length - u;
        heading = HALF_PI - 0.5f * c->k_ramp * left * left;
    }
    return heading * (1.0f / HALF_PI);
}

// d(progress)/du, 1/mm: the curvature, over the quarter turn.
static float curve_rate(const curve_t *c, float u){
    if(u <= 0.0f || u >= c->length) return 0.0f;
    float curvature;
    if(u < c->ramp) curvature = c->k_ramp * u;
    else if(u < c->length - c->ramp) curvature = c->k;
    else curvature = c->k_ramp * (c->length - u);
    return curvature * (1.0f / HALF_PI);
}

uint8_t curve_setup(curve_t *c, float radius, float ramp, float angle, float pre_adjust, float post_adjust,
                    float cell_mm){
    if(radius < 10.0f || ramp < 1.0f || ramp > radius * HALF_PI || angle < 45.0f || angle > 135.0f) return 0;
    c->radius = radius;
    c->ramp = ramp;
    c->angle = angle;
    c->k = 1.0f / radius;
    c->k_ramp = c->k / ramp;
    // Each clothoid turns ramp / (2 radius): the arc makes up the rest.
    c->length = radius * HALF_PI + ramp;
    // How far the curve takes the robot along the entry direction and
    // sideways (equal: the shape is symmetric), integrating the heading.
    const float h = c->length / FOOTPRINT_STEPS;
    float along = 0.0f, across = 0.0f;
    for(uint8_t i = 0; i <= FOOTPRINT_STEPS; i++){
        float sin_h, cos_h;
        sin_cos(curve_progress(c, (float)i * h) * HALF_PI, &sin_h, &cos_h);
        const float w = (i == 0 || i == FOOTPRINT_STEPS) ? 1.0f : (i & 1u) ? 4.0f : 2.0f;
        along += w * cos_h;
        across += w * sin_h;
    }
    along *= h / 3.0f;
    across *= h / 3.0f;
    c->footprint = 0.5f * (along + across);
    c->pre = 0.5f * cell_mm - along + pre_adjust;
    c->post = 0.5f * cell_mm - across + post_adjust;
    // Curves in consecutive cells must not overlap, and there must be room
    // to settle on the centre line before and after one.
    return c->pre + c->post >= 0.0f && c->pre >= -0.25f * cell_mm && c->post >= -0.25f * cell_mm;
}

// ---- Path following ------------------------------------------------------------------------

static int8_t turn_at(const run_path_t *p, uint8_t i){
    return p->turn ? p->turn[i] : 0;
}

// Scans from the straight's first cell for the next curve.
static void find_next(path_run_t *r){
    const run_path_t *p = &r->path;
    float edge = r->first_edge;
    uint8_t i = r->first;
    while(i < p->cells && turn_at(p, i) == 0){
        edge += r->cell_mm;
        i++;
    }
    r->next = i;
    r->dir = i < p->cells ? turn_at(p, i) : 0;
    r->curve_start = i < p->cells ? edge + r->curve.pre : PATH_NONE;
}

// `next` limited so that a point `rem` ahead can still be reached at `final`
// (profile_brake_speed()). While that point is far the limit is higher than
// `next` anyway: checked without the square root (~10 us in soft float,
// every millisecond in SysTick).
static float brake_cap(float next, float v0, float rem, float final, float rate, float dt){
    const float a_dt = rate * dt;
    if(next * (next + a_dt) <= final * final + 2.0f * rate * (rem - 0.5f * v0 * dt)) return next;
    return fminf(next, profile_brake_speed(v0, rem, final, rate, dt));
}

uint8_t path_start(path_run_t *r, const run_path_t *path, const curve_t *curve, float cell_mm,
                   float v_straight, float v_curve, float accel){
    if(!path->cells || turn_at(path, (uint8_t)(path->cells - 1u)) != 0) return 0;
    float length = cell_mm;     // half of the start cell and half of the last one
    for(uint8_t i = 0; i + 1u < path->cells; i++){
        const int8_t turn = turn_at(path, i);
        if(turn < -1 || turn > 1) return 0;
        length += turn ? curve->pre + curve->length + curve->post : cell_mm;
    }
    r->path = *path;
    r->curve = *curve;
    r->cell_mm = cell_mm;
    r->accel = accel;
    r->v_straight = v_straight;
    // After its last curve the path ends at the next cell centre: the
    // curve speed must leave room to brake there. Every other curve is
    // followed by a straight or another curve at the same speed.
    const float room = curve->post + 0.5f * cell_mm;
    r->v_curve = fminf(fminf(v_curve, v_straight), sqrtf(2.0f * accel * room));
    r->length = r->stop_at = length;
    r->s = r->v = 0.0f;
    r->heading = r->base = 0.0f;
    r->first = 0;
    r->first_edge = 0.5f * cell_mm;
    r->last_curve_end = 0.0f;
    r->curves = 0;
    r->hold = 0;
    r->done = 0;
    r->lag = r->w = 0.0f;
    r->scale = r->scale_min = 1.0f;
    find_next(r);
    return 1;
}

// 1 while the robot keeps up, down to PATH_SCALE_MIN as it falls behind.
// Never 0: a blocked robot must still fall FWD_ERROR_MAX_MM behind.
static float time_scale(float lag){
    const float x = (lag - PATH_LAG_FREE_MM) * (1.0f / PATH_LAG_SPAN_MM);
    return x <= 0.0f ? 1.0f : fmaxf(1.0f - x, PATH_SCALE_MIN);
}

void path_step(path_run_t *r, profile_t *fwd, profile_t *rot, float dt_real){
    const float s0 = r->s, v0 = r->v, h0 = r->heading;
    // The reference lives in its own time, which runs slower than the real
    // one while the robot lags: distance, speed and heading all follow it.
    const float scale = time_scale(r->lag);
    const float dt = scale * dt_real;
    r->scale = scale;
    if(scale < r->scale_min) r->scale_min = scale;
    if(!r->done){
        const float a_dt = r->accel * dt;
        // Speed limit where the reference is, and braking in time for the
        // next curve and for the end: the plain profile's scheme, so it
        // arrives at each exactly at its speed.
        const float cap = r->hold ? 0.0f : r->s >= r->curve_start ? r->v_curve : r->v_straight;
        float next = v0 < cap ? fminf(v0 + a_dt, cap) : fmaxf(v0 - a_dt, cap);
        if(r->s < r->curve_start) next = brake_cap(next, v0, r->curve_start - r->s, r->v_curve, r->accel, dt);
        const float rem = r->stop_at - r->s;
        next = brake_cap(next, v0, rem, 0.0f, r->accel, dt);
        // The end moved closer (a wall): brake up to twice as hard; never reverse.
        next = fmaxf(next, fmaxf(v0 - 2.0f * a_dt, 0.0f));
        const float step = 0.5f * (v0 + next) * dt;
        if(rem <= PATH_DONE_EPS || step >= rem){
            r->s = r->stop_at;
            r->v = 0.0f;
            r->done = 1;
        }
        else{
            r->s += step;
            r->v = next;
        }
        // Past the end of a curve: the next straight starts at its cell's exit edge.
        while(r->next < r->path.cells && r->s >= r->curve_start + r->curve.length){
            r->base += (float)r->dir * r->curve.angle;
            r->last_curve_end = r->curve_start + r->curve.length;
            r->first_edge = r->last_curve_end + r->curve.post;
            r->first = (uint8_t)(r->next + 1u);
            r->curves++;
            find_next(r);
        }
        r->heading = r->base;
        if(r->s > r->curve_start){
            r->heading += (float)r->dir * r->curve.angle * curve_progress(&r->curve, r->s - r->curve_start);
        }
    }
    // What the loops see, in real time. The accelerations leave out the
    // change of the time scale itself: it only happens while the motors are
    // out of PWM, and feeding it forward would just kick them.
    fwd->pos = r->s;
    fwd->delta = r->s - s0;
    fwd->speed = scale * r->v;
    fwd->accel = scale * (r->v - v0) / dt_real;
    fwd->target = r->stop_at;
    fwd->top = r->v_straight;
    fwd->final = 0.0f;
    fwd->rate = r->accel;
    fwd->dir = 1;
    fwd->active = !r->done;
    // Angular speed = curvature times forward speed.
    const float rate = r->s > r->curve_start
                     ? (float)r->dir * r->curve.angle * curve_rate(&r->curve, r->s - r->curve_start) : 0.0f;
    const float w = rate * r->v;
    rot->pos = rot->target = r->heading;
    rot->delta = r->heading - h0;
    rot->speed = scale * w;
    rot->accel = scale * (w - r->w) / dt_real;
    r->w = w;
    rot->top = rot->final = 0.0f;
    rot->rate = 1.0f;
    rot->dir = 1;
    rot->active = 0;
}

uint8_t path_on_straight(const path_run_t *r, float s){
    return s >= r->last_curve_end && s < r->curve_start;
}

float path_straight_end(const path_run_t *r){
    return r->next < r->path.cells ? r->curve_start - r->curve.pre + 0.5f * r->cell_mm : r->length;
}

uint8_t path_straight_centre(const path_run_t *r, float s, uint8_t nearest, uint8_t *entered, float *centre){
    // Cells with their centre on this straight: `first` up to the one before
    // the next curve (or the last cell), plus the start cell (-1) if the
    // straight starts there.
    const int16_t last = r->next < r->path.cells ? (int16_t)r->next - 1 : (int16_t)r->path.cells - 1;
    const int16_t lo = r->first == 0 ? -1 : (int16_t)r->first;
    if(last < lo) return 0;
    const float x = (s - r->first_edge - 0.5f * r->cell_mm) / r->cell_mm + (nearest ? 0.5f : 0.0f);
    int16_t i = (int16_t)x;
    if((float)i > x) i--;       // floor
    i = (int16_t)(i + (int16_t)r->first);
    if(i > last) i = last;
    if(i < lo){
        if(!nearest) return 0;
        i = lo;
    }
    *entered = (uint8_t)(i + 1);
    *centre = r->first_edge + 0.5f * r->cell_mm + (float)(i - (int16_t)r->first) * r->cell_mm;
    return 1;
}
