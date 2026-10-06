#include "sim.h"
#include <math.h>
#include <string.h>
#include "params.h"
#include "robot_config.h"

sim_stats_t sim_stats;
uint32_t sim_uart_waits;
uint8_t sim_x, sim_y;
heading_t sim_h;

static uint8_t truth[MAZE_SIZE][MAZE_SIZE];
static double noise;
static double side_doubt;
static uint8_t sides_fresh;     // a straight just ended: sides read on the way in (as motion.c)
static uint32_t rng_state = 1;
static uint32_t abort_after;
static uint32_t abort_after_cells;

uint32_t sim_rand(void){
    // xorshift32
    rng_state ^= rng_state << 13;
    rng_state ^= rng_state >> 17;
    rng_state ^= rng_state << 5;
    return rng_state;
}

static double rand_unit(void){
    return (double)(sim_rand() & 0xFFFFFFu) / (double)0x1000000u;
}

static uint8_t in_maze(int x, int y){
    return x >= 0 && y >= 0 && x < MAZE_SIZE && y < MAZE_SIZE;
}

void truth_set_wall(uint8_t x, uint8_t y, heading_t h, uint8_t on){
    int nx = x + heading_dx(h), ny = y + heading_dy(h);
    if(!in_maze(nx, ny)) return;     // border stays a wall
    if(on){
        truth[x][y] |= (uint8_t)(1u << h);
        truth[nx][ny] |= (uint8_t)(1u << heading_back(h));
    }
    else{
        truth[x][y] &= (uint8_t)~(1u << h);
        truth[nx][ny] &= (uint8_t)~(1u << heading_back(h));
    }
}

uint8_t truth_wall(uint8_t x, uint8_t y, heading_t h){
    return (truth[x][y] >> h) & 1u;
}

void truth_reset(uint8_t with_all_walls){
    memset(truth, with_all_walls ? 0x0F : 0x00, sizeof(truth));
    for(uint8_t i = 0; i < MAZE_SIZE; i++){
        truth[i][0] |= 1u << SOUTH;
        truth[i][MAZE_SIZE - 1] |= 1u << NORTH;
        truth[0][i] |= 1u << WEST;
        truth[MAZE_SIZE - 1][i] |= 1u << EAST;
    }
}

void truth_generate(uint32_t seed, uint16_t extra_openings){
    rng_state = seed ? seed : 1;
    truth_reset(1);
    static uint8_t seen[MAZE_SIZE][MAZE_SIZE];
    static uint8_t stack_x[MAZE_SIZE * MAZE_SIZE], stack_y[MAZE_SIZE * MAZE_SIZE];
    memset(seen, 0, sizeof(seen));
    int top = 0;
    stack_x[0] = 0;
    stack_y[0] = 0;
    seen[0][0] = 1;
    while(top >= 0){
        uint8_t x = stack_x[top], y = stack_y[top];
        heading_t options[4];
        int n = 0;
        for(int h = 0; h < 4; h++){
            int nx = x + heading_dx((heading_t)h), ny = y + heading_dy((heading_t)h);
            if(!in_maze(nx, ny) || seen[nx][ny]) continue;
            if(x == 0 && y == 0 && h == EAST) continue;     // start cell keeps its east wall
            options[n++] = (heading_t)h;
        }
        if(!n){
            top--;
            continue;
        }
        heading_t h = options[sim_rand() % (uint32_t)n];
        uint8_t nx = (uint8_t)(x + heading_dx(h)), ny = (uint8_t)(y + heading_dy(h));
        truth_set_wall(x, y, h, 0);
        seen[nx][ny] = 1;
        top++;
        stack_x[top] = nx;
        stack_y[top] = ny;
    }
    for(uint16_t i = 0; i < extra_openings; i++){
        uint8_t x = (uint8_t)(sim_rand() % MAZE_SIZE), y = (uint8_t)(sim_rand() % MAZE_SIZE);
        heading_t h = (heading_t)(sim_rand() % 4);
        if(x == 0 && y == 0 && h == EAST) continue;
        if(x == 1 && y == 0 && h == WEST) continue;
        truth_set_wall(x, y, h, 0);
    }
}

// Competition goal: the centre 2x2 open inside, closed around but for one entrance.
void truth_competition_goal(uint32_t seed){
    rng_state = seed ? seed : 1;
    const uint8_t g0 = MAZE_SIZE / 2 - 1, g1 = MAZE_SIZE / 2;
    for(uint8_t x = g0; x <= g1; x++){
        for(uint8_t y = g0; y <= g1; y++){
            for(uint8_t h = 0; h < 4; h++){
                const int nx = x + heading_dx((heading_t)h), ny = y + heading_dy((heading_t)h);
                const uint8_t inside = nx >= g0 && nx <= g1 && ny >= g0 && ny <= g1;
                truth_set_wall(x, y, (heading_t)h, !inside);
            }
        }
    }
    // One of the 8 outer walls opens: cell (x, y) of the block, side h.
    const uint8_t k = (uint8_t)(sim_rand() % 8u);
    const uint8_t ex = (uint8_t)(g0 + ((k >> 1) & 1u)), ey = (uint8_t)(g0 + ((k >> 2) & 1u));
    const heading_t eh = (heading_t)(k & 1u ? (ex == g0 ? WEST : EAST) : (ey == g0 ? SOUTH : NORTH));
    truth_set_wall(ex, ey, eh, 0);
    // Rejoin: open a wall between a reached and an unreached cell (never the goal's) until all are reached.
    static uint8_t reached[MAZE_SIZE][MAZE_SIZE];
    static uint8_t qx[MAZE_SIZE * MAZE_SIZE], qy[MAZE_SIZE * MAZE_SIZE];
    for(;;){
        memset(reached, 0, sizeof(reached));
        int head = 0, tail = 0;
        qx[tail] = 0;
        qy[tail++] = 0;
        reached[0][0] = 1;
        while(head < tail){
            const uint8_t x = qx[head], y = qy[head++];
            for(uint8_t h = 0; h < 4; h++){
                if(truth_wall(x, y, (heading_t)h)) continue;
                const uint8_t nx = (uint8_t)(x + heading_dx((heading_t)h)), ny = (uint8_t)(y + heading_dy((heading_t)h));
                if(reached[nx][ny]) continue;
                reached[nx][ny] = 1;
                qx[tail] = nx;
                qy[tail++] = ny;
            }
        }
        if(tail == MAZE_SIZE * MAZE_SIZE) return;
        uint8_t joined = 0;
        for(uint8_t x = 0; x < MAZE_SIZE && !joined; x++){
            for(uint8_t y = 0; y < MAZE_SIZE && !joined; y++){
                if(!reached[x][y] || (x >= g0 && x <= g1 && y >= g0 && y <= g1)) continue;
                for(uint8_t h = 0; h < 4 && !joined; h++){
                    const int nx = x + heading_dx((heading_t)h), ny = y + heading_dy((heading_t)h);
                    if(!in_maze(nx, ny) || reached[nx][ny]) continue;
                    if(nx >= g0 && nx <= g1 && ny >= g0 && ny <= g1) continue;
                    truth_set_wall(x, y, (heading_t)h, 0);
                    joined = 1;
                }
            }
        }
        if(!joined) return;     // cannot happen: the block has its entrance
    }
}

void truth_load_into_map(void){
    maze_init();
    for(uint8_t x = 0; x < MAZE_SIZE; x++){
        for(uint8_t y = 0; y < MAZE_SIZE; y++){
            for(uint8_t h = 0; h < 4; h++){
                if(truth_wall(x, y, (heading_t)h)) maze_mark_blocked(x, y, (heading_t)h);
                else maze_mark_crossed(x, y, (heading_t)h);
            }
            maze_mark_visited(x, y);
        }
    }
}

void sim_reset(double sensor_noise, uint32_t seed){
    memset(&sim_stats, 0, sizeof(sim_stats));
    sim_x = 0;
    sim_y = 0;
    sim_h = NORTH;
    noise = sensor_noise;
    rng_state = seed ? seed : 1;
    abort_after = 0;
    abort_after_cells = 0;
    side_doubt = 0.0;
    sides_fresh = 0;
}

void sim_side_doubt(double probability){
    side_doubt = probability;
}

void sim_abort_after(uint32_t actions){
    abort_after = actions;
}

void sim_abort_after_cells(uint32_t cells){
    abort_after_cells = cells;
}

// ---- Time model ----------------------------------------------------------------
// Seconds as the robot takes them (practice maze): profiles, 0.30 s a quarter turn, 0.08 s at every stop.
#define SIM_STOP_S          0.08
#define SIM_QUARTER_TURN_S  0.30

// From rest to rest over `mm`, cruising at `v` with the accel parameter.
static double drive_seconds(double mm, double v){
    const double a = params.accel;
    if(mm <= 0.0) return 0.0;
    if(v * v / a >= mm) return 2.0 * sqrt(mm / a);
    return mm / v + v / a;
}

// The virtual robot lives the moves' time for real (its wheels may turn: `speed` mm/s, or `turn` > 0 right).
static void elapse(double s, int16_t speed, int8_t turn){
    sim_stats.seconds += s;
#ifdef SIM_ON_ROBOT
    virtual_elapse(s, speed, turn);
#else
    (void)speed;
    (void)turn;
#endif
}

static void stopped(double drive_s, int16_t speed){
    elapse(drive_s, speed, 0);
    elapse(SIM_STOP_S, 0, 0);
    sim_stats.stops++;
}

// ---- motion.h ------------------------------------------------------------------

static uint8_t noisy(uint8_t v){
    return (noise > 0.0 && rand_unit() < noise) ? (uint8_t)!v : v;
}

move_result_t motion_sense_walls(wall_sense_t *out){
    sim_stats.senses++;
    out->front = noisy(truth_wall(sim_x, sim_y, sim_h));
    out->left = noisy(truth_wall(sim_x, sim_y, heading_left(sim_h)));
    out->right = noisy(truth_wall(sim_x, sim_y, heading_right(sim_h)));
    if(side_doubt > 0.0 && rand_unit() < side_doubt) out->left = SEEN_DOUBTFUL;
    if(side_doubt > 0.0 && rand_unit() < side_doubt) out->right = SEEN_DOUBTFUL;
    out->moving = sides_fresh;
    sides_fresh = 0;
    return MOVE_OK;
}

move_result_t motion_forward(uint8_t cells, int16_t cruise_speed){
    sides_fresh = 0;
    sim_stats.actions++;
    sim_stats.forward_moves++;
    for(uint8_t i = 0; i < cells; i++){
        if(truth_wall(sim_x, sim_y, sim_h)){
            stopped(drive_seconds(i * CELL_MM, cruise_speed), cruise_speed);
            if(i == 0){
                sim_stats.blocked++;    // the robot's emergency stop + back up
                return MOVE_BLOCKED;
            }
            sim_stats.crashes++;
            return MOVE_LOST;
        }
        sim_x = (uint8_t)(sim_x + heading_dx(sim_h));
        sim_y = (uint8_t)(sim_y + heading_dy(sim_h));
        sim_stats.forward_cells++;
    }
    stopped(drive_seconds(cells * CELL_MM, cruise_speed), cruise_speed);
    sides_fresh = cells > 0;
    return MOVE_OK;
}

// Side walls read on the way into a cell (three sensor periods): unanimous or doubtful.
static uint8_t side_on_the_way(uint8_t wall){
    const uint8_t seen = (uint8_t)(noisy(wall) + noisy(wall) + noisy(wall));
    return seen == 3 ? SEEN_PRESENT : seen == 0 ? SEEN_ABSENT : SEEN_DOUBTFUL;
}

// A search leg: straight on, each cell decided inside it with its walls in view.
move_result_t motion_explore(int16_t speed, next_cell_fn decide, void *ctx, uint8_t *entered){
    sides_fresh = 0;
    sim_stats.actions++;
    sim_stats.legs++;
    *entered = 0;
    double cells_s = 0.0;   // spent cell by cell on the way; the rest at the stop
    for(;;){
        // Leaving through an unexpected front wall (read open wrongly) stops it at the centre.
        if(truth_wall(sim_x, sim_y, sim_h)){
            stopped(drive_seconds(*entered * CELL_MM, speed) - cells_s, speed);
            sim_stats.wall_stops++;
            sides_fresh = *entered > 0;
            return MOVE_BLOCKED;
        }
        // Into the next cell: its walls and decision come there, as on the robot.
        const double cell_s = CELL_MM / (double)speed;
        elapse(cell_s, speed, 0);
        cells_s += cell_s;
        sim_x = (uint8_t)(sim_x + heading_dx(sim_h));
        sim_y = (uint8_t)(sim_y + heading_dy(sim_h));
        sim_stats.forward_cells++;
        wall_sense_t w;
        w.front = noisy(truth_wall(sim_x, sim_y, sim_h));
        w.left = side_on_the_way(truth_wall(sim_x, sim_y, heading_left(sim_h)));
        w.right = side_on_the_way(truth_wall(sim_x, sim_y, heading_right(sim_h)));
        w.moving = 1;
        const uint32_t pops0 = maze_plan_pops(), waits0 = sim_uart_waits;
        const next_move_t next = decide(&w, ctx);
        const uint32_t pops = maze_plan_pops() - pops0;
        sim_stats.decide_waits += sim_uart_waits - waits0;
        sim_stats.decides++;
        sim_stats.decide_pops += pops;
        if(pops > sim_stats.decide_pops_max) sim_stats.decide_pops_max = pops;
        const uint32_t bucket = pops / SIM_POPS_BUCKET_SIZE;
        sim_stats.decide_hist[bucket < SIM_POPS_BUCKETS ? bucket : SIM_POPS_BUCKETS - 1]++;
        (*entered)++;
        if(next == NEXT_STOP){
            stopped(drive_seconds(*entered * CELL_MM, speed) - cells_s, speed);
            sides_fresh = 1;
            return MOVE_OK;
        }
    }
}

move_result_t motion_run_path(const run_path_t *path, int16_t cruise_speed, int16_t curve_speed, uint8_t *entered){
    (void)cruise_speed;
    (void)curve_speed;
    sides_fresh = 0;
    sim_stats.actions++;
    sim_stats.paths++;
    *entered = 0;
    double mm = 0.0;
    for(uint8_t i = 0; i < path->cells; i++){
        if(truth_wall(sim_x, sim_y, sim_h)){
            // Seen from a straight it stops at the cell before; right after a curve it is seen too late.
            if(i == 0 || path->turn[i - 1] == 0){
                sim_stats.blocked++;
                return MOVE_BLOCKED;
            }
            sim_stats.crashes++;
            return MOVE_LOST;
        }
        sim_x = (uint8_t)(sim_x + heading_dx(sim_h));
        sim_y = (uint8_t)(sim_y + heading_dy(sim_h));
        sim_stats.forward_cells++;
        mm += path->turn[i] ? 149.0 : CELL_MM;
        if(path->turn[i]){
            sim_h = (heading_t)((sim_h + path->turn[i] + 4) & 3);
            sim_stats.curves++;
        }
        *entered = (uint8_t)(i + 1u);
    }
    stopped(drive_seconds(mm, cruise_speed), cruise_speed);
    sides_fresh = 1;
    return MOVE_OK;
}

move_result_t motion_turn(int8_t quarter_turns){
    sides_fresh = 0;
    sim_stats.actions++;
    sim_stats.quarter_turns += (uint32_t)(quarter_turns < 0 ? -quarter_turns : quarter_turns);
    elapse(SIM_QUARTER_TURN_S * (quarter_turns < 0 ? -quarter_turns : quarter_turns), 0,
           (int8_t)(quarter_turns > 0 ? 1 : -1));
    sim_h = (heading_t)((sim_h + quarter_turns + 4) & 3);
    return MOVE_OK;
}

void motion_align_front(void){}

#ifndef SIM_ON_ROBOT    // the virtual robot has the robot's: START, STOP, step mode, LEDs
uint8_t motion_checkpoint(void){
    return !(abort_after && sim_stats.actions >= abort_after)
           && !(abort_after_cells && sim_stats.forward_cells >= abort_after_cells);
}

void motion_indicate(indication_t what){
    (void)what;
}
#endif
