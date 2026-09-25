#include "control_sim.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "control.h"
#include "robot_config.h"

#define DEAD_MAX 32
#define SIDE_BEAM_MM_PER_DEG 0.33f  // the 15 deg side beam reads shorter as the robot yaws towards its wall
#define FADE_MM 40.0f               // as STEER_FADE_MM in motion.c

typedef struct {
    float gain, tau, friction, stiction;
    float v;                    // mm/s
    int16_t queue[DEAD_MAX];
    int dead, head;
} motor_t;

static uint32_t rng;
float sim_average = STEER_AVERAGE_MS;       // experiments: side IR averaging, ms
float sim_curve = STEER_CURVE_DEG_PER_MM;   // experiments: centring curvature limit
float sim_window = STEER_BIAS_WINDOW_MM;    // experiments: KI learning window

static float uniform(void){
    rng = rng * 1664525u + 1013904223u;
    return (float)(rng >> 8) / 16777216.0f;
}

static float gauss(float sd){
    float s = 0.0f;
    for(int i = 0; i < 12; i++) s += uniform();
    return (s - 6.0f) * sd;
}

static void motor_init(motor_t *m, float gain, const plant_t *p){
    memset(m, 0, sizeof(*m));
    m->gain = gain;
    m->tau = p->tau;
    m->friction = p->friction;
    m->stiction = p->stiction;
    m->dead = p->dead_ms < DEAD_MAX ? p->dead_ms : DEAD_MAX - 1;
}

// PWM left to accelerate the wheel after its friction; 0 while the PWM
// cannot beat the static friction of a wheel at rest.
static float motor_drive(motor_t *m, int16_t pwm){
    m->queue[m->head] = pwm;
    m->head = (m->head + 1) % (m->dead + 1);
    const float u = (float)m->queue[m->head];
    if(fabsf(m->v) < 1.0f){
        if(fabsf(u) <= m->stiction) return 0.0f;
        return u - copysignf(m->friction, u);
    }
    return u - copysignf(m->friction, m->v);
}

// Both wheels, first order each, coupled through the chassis: a change of
// heading also drags the skids, which hold the heading until the wheels
// push apart hard enough (the robot's stick-slip in yaw).
static void wheels_step(motor_t *l, motor_t *r, int16_t pwm_l, int16_t pwm_r, float yaw_friction,
                        float yaw_stiction, float dt){
    const float ul = motor_drive(l, pwm_l), ur = motor_drive(r, pwm_r);
    float al = (l->gain * ul - l->v) / l->tau, ar = (r->gain * ur - r->v) / r->tau;
    if(fabsf(l->v) < 1.0f && ul == 0.0f){ l->v = 0.0f; al = 0.0f; }
    if(fabsf(r->v) < 1.0f && ur == 0.0f){ r->v = 0.0f; ar = 0.0f; }
    const float vd = 0.5f * (l->v - r->v);
    const float ac = 0.5f * (al + ar);
    float ad = 0.5f * (al - ar);
    if(yaw_stiction > 0.0f || yaw_friction > 0.0f){
        const float gain = 0.5f * (l->gain + r->gain), tau = 0.5f * (l->tau + r->tau);
        if(fabsf(vd) < 1.0f && fabsf(0.5f * (ul - ur)) <= yaw_stiction){
            ad = -vd / dt;      // held: the wheels run together
        }
        else{
            const float sign = fabsf(vd) >= 1.0f ? copysignf(1.0f, vd) : copysignf(1.0f, ul - ur);
            ad -= gain * yaw_friction * sign / tau;
        }
    }
    l->v += (ac + ad) * dt;
    r->v += (ac - ad) * dt;
}

plant_t plant_nominal(void){
    plant_t p = {
        .gain_l = 1.0f, .gain_r = 1.0f, .tau = MOTOR_TAU_S, .friction = MOTOR_KS_PWM, .stiction = 85.0f,
        .yaw_friction = 15.0f, .yaw_stiction = 50.0f,
        .dead_ms = 0, .ir_noise = 1.0f, .ir_delay_ms = IR_DELAY_MS,
        .ir_period_ms = 16, .ir_step_mm = 2.0f, .y0 = 0.0f, .yaw0 = 0.0f, .seed = 1,
    };
    return p;
}

// The firmware's gains (robot_config.h), as motion.c sets them up.
static control_config_t firmware_control(void){
    const control_config_t k = {
        .ticks_per_mm = WHEEL_TICKS_PER_MM, .mm_per_deg = TICKS_PER_TURN / 90.0f / WHEEL_TICKS_PER_MM,
        .kv_l = MOTOR_KV_L, .kv_r = MOTOR_KV_R, .tau = MOTOR_TAU_S, .ks = MOTOR_KS_PWM,
        .fwd_kp = FWD_KP, .fwd_kd = FWD_KD, .rot_kp = ROT_KP, .rot_kd = ROT_KD,
        .rot_ki = ROT_KI, .rot_i_max = ROT_I_MAX, .pwm_limit = CONTROL_PWM_LIMIT,
        .settle_ki_fwd = SETTLE_KI_FWD, .settle_ki_rot = SETTLE_KI_ROT, .settle_i_max = SETTLE_I_MAX,
    };
    return k;
}

static sim_result_t run(const plant_t *p, float mm, float speed, float accel, float deg, float turn_speed,
                        float turn_accel, float kp, float ki){
    const float dt = CONTROL_DT_S;
    const control_config_t k = firmware_control();
    const float mm_per_deg = k.mm_per_deg;
    const steer_config_t sk = {
        .kp = kp, .ki = ki * 0.001f, .max_deg = STEER_MAX_DEG, .curve_deg = sim_curve,
        .slew_mm = STEER_SLEW_MM_PER_MS, .track_mm = SIDE_WALL_TRACK_MM, .center_l_mm = LANE_WIDTH_MM / 2.0f,
        .center_r_mm = LANE_WIDTH_MM / 2.0f,
        .error_max_mm = STEER_ERROR_MAX_MM, .bias_window_mm = sim_window, .delay_steps = (uint8_t)(IR_DELAY_MS + sim_average / 2),
        .average_steps = (uint8_t)sim_average,
    };
    profile_t fwd, rot;
    control_t c;
    steer_t s;
    motor_t ml, mr;
    profile_reset(&fwd);
    profile_reset(&rot);
    control_reset(&c);
    steer_reset(&s);
    // The model gives PWM = KV * v, so the true gain is 1 / KV (scaled).
    motor_init(&ml, p->gain_l / MOTOR_KV_L, p);
    motor_init(&mr, p->gain_r / MOTOR_KV_R, p);
    rng = p->seed;
    const uint8_t steering = mm != 0.0f;
    if(steering) profile_start(&fwd, mm, speed, 0.0f, accel);
    else profile_start(&rot, deg, turn_speed, 0.0f, turn_accel);

    sim_result_t r;
    memset(&r, 0, sizeof(r));
    float xl = 0.0f, xr = 0.0f;             // true wheel travel
    int32_t cl = 0, cr = 0;                 // encoder counts
    float y = p->y0, yaw = p->yaw0;         // mm left of centre, deg right of the corridor
    float turned = 0.0f;
    uint32_t done_at = 0, settled = 0;
    float held_l = LANE_WIDTH_MM / 2.0f, held_r = LANE_WIDTH_MM / 2.0f;
    static float y_hist[8000], yaw_hist[8000];
    uint32_t n = 0;
    for(uint32_t t = 1; t < 8000 && !settled; t++){
        profile_step(&fwd, dt);
        profile_step(&rot, dt);
        const int32_t nl = (int32_t)floorf(xl * WHEEL_TICKS_PER_MM), nr = (int32_t)floorf(xr * WHEEL_TICKS_PER_MM);
        const int32_t dl = nl - cl, dr = nr - cr;
        cl = nl;
        cr = nr;
        float heading = 0.0f;
        if(steering){
            // What the side sensors saw ir_delay_ms ago.
            const uint32_t back = n > (uint32_t)p->ir_delay_ms ? n - (uint32_t)p->ir_delay_ms : 0;
            const float ys = n ? y_hist[back] : y, yaws = n ? yaw_hist[back] : yaw;
            // Sample and hold every ir_period_ms, quantized, as the real sensors.
            if(!p->ir_period_ms || t % (uint32_t)p->ir_period_ms == 1u || t == 1u){
                const float step = p->ir_step_mm > 0.0f ? p->ir_step_mm : 0.001f;
                held_r = step * roundf((LANE_WIDTH_MM / 2.0f + ys - SIDE_BEAM_MM_PER_DEG * yaws + gauss(p->ir_noise)) / step);
                held_l = step * roundf((LANE_WIDTH_MM / 2.0f - ys + SIDE_BEAM_MM_PER_DEG * yaws + gauss(p->ir_noise)) / step);
            }
            const float sr = held_r, sl = held_l;
            const float remaining = fwd.target - (fwd.pos - c.fwd_error);
            const float fade = remaining < FADE_MM ? fmaxf(remaining, 0.0f) / FADE_MM : 1.0f;
            const float gain = fade * (fwd.speed > STEER_VREF_MM_S ? STEER_VREF_MM_S / fwd.speed : 1.0f);  // as motion.c
            const float rot_now = rot.pos + c.steer_prev - c.rot_error;
            heading = steer_step(&s, &sk, sl, sr, 0.5f * fabsf((float)(dl + dr)) / WHEEL_TICKS_PER_MM, rot_now, gain);
        }
        control_step(&c, &k, &fwd, &rot, heading, dl, dr, dt);
        if(abs(c.pwm_l) > r.pwm_max) r.pwm_max = abs(c.pwm_l);
        if(abs(c.pwm_r) > r.pwm_max) r.pwm_max = abs(c.pwm_r);
        wheels_step(&ml, &mr, c.pwm_l, c.pwm_r, p->yaw_friction, p->yaw_stiction, dt);
        const float vl = ml.v, vr = mr.v;
        xl += vl * dt;
        xr += vr * dt;
        const float v = 0.5f * (vl + vr), w = 0.5f * (vl - vr) / mm_per_deg;
        yaw += w * dt;
        turned += w * dt;
        y -= v * sinf(yaw * 3.14159265f / 180.0f) * dt;
        r.travelled += v * dt;
        if(fabsf(c.fwd_error) > r.fwd_err_max) r.fwd_err_max = fabsf(c.fwd_error);
        if(fabsf(c.rot_error) > r.rot_err_max) r.rot_err_max = fabsf(c.rot_error);
        yaw_hist[n] = yaw;
        y_hist[n++] = y;
        // As guard_settled() in motion.c.
        if(!fwd.active && !rot.active){
            if(!done_at) done_at = t;
            const uint8_t still = fabsf(vl) < 1.0f && fabsf(vr) < 1.0f;
            if((fabsf(c.fwd_error) < SETTLE_MM && fabsf(c.rot_error) < SETTLE_DEG && still)
               || t - done_at >= SETTLE_MAX_MS){
                settled = 1;
                r.ms = t;
            }
        }
    }
    r.turned = turned;
    r.y_end = y;
    r.yaw_end = yaw;
    for(uint32_t i = 1; i < n; i++){
        if((y_hist[i] < 0.0f) != (y_hist[i - 1] < 0.0f)) r.crossings++;
        if(i >= n / 2 && fabsf(y_hist[i]) > r.y_late) r.y_late = fabsf(y_hist[i]);
    }
    return r;
}

sim_result_t sim_straight(const plant_t *p, float mm, float speed, float accel, float kp, float ki){
    return run(p, mm, speed, accel, 0.0f, 0.0f, 1.0f, kp, ki);
}

sim_result_t sim_turn(const plant_t *p, float deg, float speed, float accel){
    return run(p, 0.0f, 0.0f, 1.0f, deg, speed, accel, 0.0f, 0.0f);
}

path_result_t sim_path(const plant_t *p, const run_path_t *path, const curve_t *curve, float v_straight,
                       float v_curve, float accel){
    const float dt = CONTROL_DT_S;
    const control_config_t k = firmware_control();
    const double rad = 3.14159265358979 / 180.0;
    path_run_t pr;
    profile_t fwd, rot;
    control_t c;
    motor_t ml, mr;
    path_result_t r;
    memset(&r, 0, sizeof(r));
    profile_reset(&fwd);
    profile_reset(&rot);
    control_reset(&c);
    motor_init(&ml, p->gain_l / MOTOR_KV_L, p);
    motor_init(&mr, p->gain_r / MOTOR_KV_R, p);
    if(!path_start(&pr, path, curve, CELL_MM, v_straight, v_curve, accel)) return r;
    float xl = 0.0f, xr = 0.0f;
    int32_t cl = 0, cr = 0;
    double x = 0.0, y = 0.0, yaw = p->yaw0;        // true pose: mm, deg (> 0 right of the start heading)
    double rx = 0.0, ry = 0.0;                      // the reference's point
    // The reference's path, one point per step (<= 1 mm apart), with its distance.
    enum { STEPS = 30000 };
    static double path_x[STEPS], path_y[STEPS];
    static float path_s[STEPS];
    uint32_t points = 0;
    double travelled = 0.0;
    uint32_t done_at = 0;
    for(uint32_t t = 1; t < STEPS; t++){
        const float ref_before = rot.pos;
        pr.lag = c.fwd_error;
        path_step(&pr, &fwd, &rot, dt);
        const double ref_mid = 0.5 * (ref_before + rot.pos) * 90.0 / curve->angle * rad;
        rx += fwd.delta * sin(ref_mid);
        ry += fwd.delta * cos(ref_mid);
        path_x[points] = rx;
        path_y[points] = ry;
        path_s[points++] = pr.s;
        const int32_t nl = (int32_t)floorf(xl * WHEEL_TICKS_PER_MM), nr = (int32_t)floorf(xr * WHEEL_TICKS_PER_MM);
        const int32_t dl = nl - cl, dr = nr - cr;
        cl = nl;
        cr = nr;
        control_step(&c, &k, &fwd, &rot, 0.0f, dl, dr, dt);
        if(abs(c.pwm_l) > r.pwm_max) r.pwm_max = abs(c.pwm_l);
        if(abs(c.pwm_r) > r.pwm_max) r.pwm_max = abs(c.pwm_r);
        wheels_step(&ml, &mr, c.pwm_l, c.pwm_r, p->yaw_friction, p->yaw_stiction, dt);
        xl += ml.v * dt;
        xr += mr.v * dt;
        const double v = 0.5 * (ml.v + mr.v), w = 0.5 * (ml.v - mr.v) / k.mm_per_deg;
        const double mid = (yaw + 0.5 * w * dt) * rad;
        x += v * sin(mid) * dt;
        y += v * cos(mid) * dt;
        yaw += w * dt;
        travelled += v * dt;
        // Distance to the reference's path near where the robot has got to
        // (behind the reference is not off the path).
        double nearest = 1e9;
        for(uint32_t i = points; i-- > 0 && path_s[i] > travelled - 40.0;){
            if(path_s[i] > travelled + 40.0) continue;
            nearest = fmin(nearest, hypot(x - path_x[i], y - path_y[i]));
        }
        if(nearest < 1e9 && nearest > r.cross_err_max) r.cross_err_max = (float)nearest;
        if(!r.stall_ms && c.fwd_error > FWD_ERROR_MAX_MM) r.stall_ms = t;
        if(fabsf(c.fwd_error) > r.fwd_err_max) r.fwd_err_max = fabsf(c.fwd_error);
        if(fabsf(c.rot_error) > r.rot_err_max) r.rot_err_max = fabsf(c.rot_error);
        if(!fwd.active){        // as guard_settled() in motion.c
            if(!done_at) done_at = t;
            const uint8_t still = fabsf(ml.v) < 1.0f && fabsf(mr.v) < 1.0f;
            if((fabsf(c.fwd_error) < SETTLE_MM && fabsf(c.rot_error) < SETTLE_DEG && still)
               || t - done_at >= SETTLE_MAX_MS){
                r.ms = t;
                break;
            }
        }
    }
    r.scale_min = pr.scale_min;
    r.end_err = (float)hypot(x - rx, y - ry);
    r.heading_err = (float)(yaw - pr.heading * 90.0 / curve->angle);
    return r;
}
