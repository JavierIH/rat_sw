#include "control_sim.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "control.h"
#include "robot_config.h"

#define DEAD_MAX 32
#define FADE_MM 40.0f               // as STEER_FADE_MM in motion.c
#define SIDE_BEAM_AHEAD_MM 20.0f    // where the angled side beams hit the walls, ahead of the robot's centre
#define SIDE_IR_DELAY_MS 8          // side IR delay: 4-8 ms on in-place turns (docs/faults/centring.md)

typedef struct {
    float gain, tau, friction, stiction;
    float v;                    // mm/s
    int16_t queue[DEAD_MAX];
    int dead, head;
} motor_t;

static uint32_t rng;

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

// PWM left to accelerate the wheel after its friction; 0 while it cannot beat the static friction.
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

// Both wheels, first order each, coupled through the chassis' stick-slip in yaw.
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
        .dead_ms = 0, .ir_noise = 1.0f, .ir_delay_ms = SIDE_IR_DELAY_MS,
        .ir_period_ms = 16, .ir_step_mm = 2.0f, .y0 = 0.0f, .yaw0 = 0.0f, .seed = 1,
        // Sensors ~40 mm ahead of the axle, beams 15 deg forward (fitted on the ring).
        .side_lever_mm = 55.0f,
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
                        float turn_accel, float kp, float kd){
    const float dt = CONTROL_DT_S;
    const control_config_t k = firmware_control();
    const float mm_per_deg = k.mm_per_deg;
    const steer_config_t sk = {
        .kp = kp, .kd = kd, .track_mm = SIDE_WALL_TRACK_MM, .center_l_mm = LANE_WIDTH_MM / 2.0f,
        .center_r_mm = LANE_WIDTH_MM / 2.0f,
        .error_max_mm = STEER_ERROR_MAX_MM, .average_steps = STEER_AVERAGE_MS,
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
    static float wall_error[64][2];
    for(int i = 0; i < 64 && p->wall_error_mm > 0.0f; i++){
        wall_error[i][0] = 2.0f * uniform() - 1.0f;
        wall_error[i][1] = 2.0f * uniform() - 1.0f;
    }
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
                // Walls a few mm off: each cell's pair, from where the beams hit it.
                const int cell = (int)floorf((r.travelled + SIDE_BEAM_AHEAD_MM) / CELL_MM + 0.5f) & 63;
                const float off_r = p->wall_error_mm * wall_error[cell][0], off_l = p->wall_error_mm * wall_error[cell][1];
                // Angled beams as geometry, 1:1 with the lateral position; at a big yaw one loses its wall.
                const float beam = 15.0f * (3.14159265f / 180.0f), a = yaws * (3.14159265f / 180.0f);
                const float d0 = LANE_WIDTH_MM / 2.0f * cosf(beam);      // sensor to wall, centred and square
                const float ahead = p->side_lever_mm - d0 * tanf(beam);   // sensors ahead of the axle
                const float yn = ys - ahead * sinf(a);
                const float to_r = d0 + yn + off_r, to_l = d0 - yn + off_l;
                const float along_r = cosf(beam - a) > 0.1f ? to_r * cosf(beam) / cosf(beam - a) : 999.0f;
                const float along_l = cosf(beam + a) > 0.1f ? to_l * cosf(beam) / cosf(beam + a) : 999.0f;
                held_r = step * roundf((LANE_WIDTH_MM / 2.0f + along_r - d0 + gauss(p->ir_noise)) / step);
                held_l = step * roundf((LANE_WIDTH_MM / 2.0f + along_l - d0 + gauss(p->ir_noise)) / step);
            }
            const float sr = held_r, sl = held_l;
            const float remaining = fwd.target - (fwd.pos - c.fwd_error);
            const float gain = remaining < FADE_MM ? fmaxf(remaining, 0.0f) / FADE_MM : 1.0f;  // as motion.c
            heading = steer_step(&s, &sk, sl, sr, dt, gain);
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
        if(fabsf(y) > r.y_max) r.y_max = fabsf(y);
        for(int i = 0; i < SIM_SAMPLES; i++){
            const float at = 90.0f * (float)(i + 1);
            if(r.travelled >= at && r.travelled - v * dt < at){
                r.y_at[i] = y;
                r.yaw_at[i] = yaw;
            }
        }
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
    r.rot_error_end = c.rot_error;
    r.y_end = y;
    r.yaw_end = yaw;
    for(uint32_t i = 1; i < n; i++){
        if((y_hist[i] < 0.0f) != (y_hist[i - 1] < 0.0f)) r.crossings++;
        if(i >= n / 2 && fabsf(y_hist[i]) > r.y_late) r.y_late = fabsf(y_hist[i]);
    }
    return r;
}

sim_result_t sim_straight(const plant_t *p, float mm, float speed, float accel, float kp, float kd){
    return run(p, mm, speed, accel, 0.0f, 0.0f, 1.0f, kp, kd);
}

sim_result_t sim_turn(const plant_t *p, float deg, float speed, float accel){
    return run(p, 0.0f, 0.0f, 1.0f, deg, speed, accel, 0.0f, 0.0f);
}

// The path's cells (start cell centre at the origin, +y north): their centres and the heading the robot
// crosses them with, 0-3 = NESW; curving ones are -1 (no side walls, the centring is blind there anyway).
typedef struct { int x, y; int8_t heading; } path_cell_t;

static uint8_t path_cells(const run_path_t *path, path_cell_t *out){
    static const int DX[4] = {0, 1, 0, -1}, DY[4] = {1, 0, -1, 0};
    int x = 0, y = 0, h = 0;
    out[0] = (path_cell_t){0, 0, 0};
    for(uint8_t i = 0; i < path->cells; i++){
        x += DX[h];
        y += DY[h];
        const int8_t turn = path->turn ? path->turn[i] : 0;
        out[i + 1] = (path_cell_t){x, y, (int8_t)(turn ? -1 : h)};
        h = (h + turn + 4) % 4;
    }
    return (uint8_t)(path->cells + 1);
}

static path_result_t path_run(const plant_t *p, const run_path_t *path, const curve_t *curve, float v_straight,
                              float v_curve, float accel, const steer_config_t *sk){
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
    double x = -(double)p->y0, y = 0.0, yaw = p->yaw0;  // true pose: mm, deg (> 0 right of the start heading)
    // Walls along the straights and the centring, as motion.c runs it (sk != NULL).
    static path_cell_t cells[PATH_MAX_CELLS + 1];
    const uint8_t n_cells = sk ? path_cells(path, cells) : 0;
    enum { HIST = 64 };
    static float lat_hist[HIST], yaw_hist[HIST], trail[HIST];
    static uint8_t wall_hist[HIST];
    steer_t st;
    steer_reset(&st);
    uint8_t blind = 0;
    float held_l = 250.0f, held_r = 250.0f;
    rng = p->seed;
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
        const double ref_mid = 0.5 * (ref_before + rot.pos) * 90.0 / pr.curve.angle * rad;
        rx += fwd.delta * sin(ref_mid);
        ry += fwd.delta * cos(ref_mid);
        path_x[points] = rx;
        path_y[points] = ry;
        path_s[points++] = pr.s;
        const int32_t nl = (int32_t)floorf(xl * WHEEL_TICKS_PER_MM), nr = (int32_t)floorf(xr * WHEEL_TICKS_PER_MM);
        const int32_t dl = nl - cl, dr = nr - cr;
        cl = nl;
        cr = nr;
        float heading = 0.0f;
        if(sk){
            // Where the robot is: its cell on the path, offset (left) and yaw against that cell's corridor.
            const int cx = (int)lround(x / CELL_MM), cy = (int)lround(y / CELL_MM);
            int8_t h = -1;
            for(uint8_t i = 0; i < n_cells; i++) if(cells[i].x == cx && cells[i].y == cy) h = cells[i].heading;
            const double ox = x - cx * CELL_MM, oy = y - cy * CELL_MM;
            const double lat = h == 0 ? -ox : h == 1 ? oy : h == 2 ? ox : -oy;
            const uint32_t now = t % HIST, then = (t + HIST - (uint32_t)p->ir_delay_ms) % HIST;
            lat_hist[now] = (float)lat;
            yaw_hist[now] = (float)(yaw - 90.0 * (h < 0 ? 0 : h));
            wall_hist[now] = h >= 0;
            trail[now] = fwd.pos - c.fwd_error;
            if(t % (uint32_t)p->ir_period_ms == 1u){
                if(t > (uint32_t)p->ir_delay_ms && wall_hist[then]){      // the angled beams, as in run()
                    const float beam = 15.0f * (float)rad, a = yaw_hist[then] * (float)rad;
                    const float d0 = LANE_WIDTH_MM / 2.0f * cosf(beam), ahead = p->side_lever_mm - d0 * tanf(beam);
                    const float yn = lat_hist[then] - ahead * sinf(a);
                    const float along_r = cosf(beam - a) > 0.1f ? (d0 + yn) * cosf(beam) / cosf(beam - a) : 999.0f;
                    const float along_l = cosf(beam + a) > 0.1f ? (d0 - yn) * cosf(beam) / cosf(beam + a) : 999.0f;
                    held_r = p->ir_step_mm * roundf((LANE_WIDTH_MM / 2.0f + along_r - d0 + gauss(p->ir_noise)) / p->ir_step_mm);
                    held_l = p->ir_step_mm * roundf((LANE_WIDTH_MM / 2.0f + along_l - d0 + gauss(p->ir_noise)) / p->ir_step_mm);
                }
                else held_l = held_r = 250.0f;
            }
            const float at = trail[now];
            const float fwd_at_ir = t > IR_DELAY_MS ? trail[(t + HIST - IR_DELAY_MS) % HIST] : 0.0f;
            if(!(pr.s < pr.curve_start && fwd_at_ir >= pr.last_curve_end)){
                blind = 1;
                heading = st.heading;
            }
            else{
                if(blind){
                    steer_restart(&st);
                    blind = 0;
                }
                const float remaining = fminf(pr.curve_start, pr.stop_at) - at;
                const float gain = remaining < FADE_MM ? fmaxf(remaining, 0.0f) / FADE_MM : 1.0f;
                heading = steer_step(&st, sk, held_l, held_r, dt, gain);
            }
        }
        control_step(&c, &k, &fwd, &rot, heading, dl, dr, dt);
        if(abs(c.pwm_l) > r.pwm_max) r.pwm_max = abs(c.pwm_l);
        if(abs(c.pwm_r) > r.pwm_max) r.pwm_max = abs(c.pwm_r);
        wheels_step(&ml, &mr, c.pwm_l, c.pwm_r, p->yaw_friction, p->yaw_stiction, dt);
        xl += ml.v * dt;
        xr += mr.v * dt;
        const double v = 0.5 * (ml.v + mr.v);
        // Turning while moving the wheels slip sideways: it turns less than the encoders say (~ v^2).
        const double vr = v / (double)CURVE_SLIP_VREF_MM_S;
        const double w = 0.5 * (ml.v - mr.v) / k.mm_per_deg * 90.0 / (90.0 + (double)p->curve_slip * vr * vr);
        const double mid = (yaw + 0.5 * w * dt) * rad;
        x += v * sin(mid) * dt;
        y += v * cos(mid) * dt;
        yaw += w * dt;
        travelled += v * dt;
        if(pr.curves && pr.next >= pr.path.cells && r.x_exit == 0.0f && r.y_exit == 0.0f){
            r.x_exit = (float)x;
            r.y_exit = (float)y;
        }
        // Distance to the reference's path near the robot (behind the reference is not off the path).
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
    r.x_end = (float)x;
    r.y_end = (float)y;
    r.heading_err = (float)(yaw - pr.heading * 90.0 / pr.curve.angle);
    return r;
}

path_result_t sim_path(const plant_t *p, const run_path_t *path, const curve_t *curve, float v_straight,
                       float v_curve, float accel){
    return path_run(p, path, curve, v_straight, v_curve, accel, NULL);
}

path_result_t sim_path_walls(const plant_t *p, const run_path_t *path, const curve_t *curve, float v_straight,
                             float v_curve, float accel, float kp, float kd){
    const steer_config_t sk = {
        .kp = kp, .kd = kd, .track_mm = SIDE_WALL_TRACK_MM, .center_l_mm = LANE_WIDTH_MM / 2.0f,
        .center_r_mm = LANE_WIDTH_MM / 2.0f, .error_max_mm = STEER_ERROR_MAX_MM, .average_steps = STEER_AVERAGE_MS,
    };
    return path_run(p, path, curve, v_straight, v_curve, accel, &sk);
}
