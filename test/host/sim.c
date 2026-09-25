#include "sim.h"
#include <string.h>

sim_stats_t sim_stats;
uint8_t sim_x, sim_y;
heading_t sim_h;

static uint8_t truth[MAZE_SIZE][MAZE_SIZE];
static double noise;
static double side_doubt;
static uint8_t sides_fresh;     // a straight just ended: sides read on the way in (as motion.c)
static uint32_t rng_state = 1;
static uint32_t abort_after;

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
    side_doubt = 0.0;
    sides_fresh = 0;
}

void sim_side_doubt(double probability){
    side_doubt = probability;
}

void sim_abort_after(uint32_t actions){
    abort_after = actions;
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
    (void)cruise_speed;
    sides_fresh = 0;
    sim_stats.actions++;
    sim_stats.forward_moves++;
    for(uint8_t i = 0; i < cells; i++){
        if(truth_wall(sim_x, sim_y, sim_h)){
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
    sides_fresh = cells > 0;
    return MOVE_OK;
}

move_result_t motion_run_path(const run_path_t *path, int16_t cruise_speed, int16_t curve_speed, uint8_t *entered){
    (void)cruise_speed;
    (void)curve_speed;
    sides_fresh = 0;
    sim_stats.actions++;
    sim_stats.paths++;
    *entered = 0;
    for(uint8_t i = 0; i < path->cells; i++){
        if(truth_wall(sim_x, sim_y, sim_h)){
            // Seen from a straight: the robot stops at the centre of the cell
            // before it. Right after a curve the front sensors see it too late.
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
        if(path->turn[i]){
            sim_h = (heading_t)((sim_h + path->turn[i] + 4) & 3);
            sim_stats.curves++;
        }
        *entered = (uint8_t)(i + 1u);
    }
    sides_fresh = 1;
    return MOVE_OK;
}

move_result_t motion_turn(int8_t quarter_turns){
    sides_fresh = 0;
    sim_stats.actions++;
    sim_stats.quarter_turns += (uint32_t)(quarter_turns < 0 ? -quarter_turns : quarter_turns);
    sim_h = (heading_t)((sim_h + quarter_turns + 4) & 3);
    return MOVE_OK;
}

void motion_align_front(void){}

uint8_t motion_checkpoint(void){
    return !(abort_after && sim_stats.actions >= abort_after);
}

void motion_indicate(indication_t what){
    (void)what;
}
