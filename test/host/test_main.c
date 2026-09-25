// Host tests for the robot's decision logic. Exit code != 0 on any failure.
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "control.h"
#include "control_sim.h"
#include "crc32.h"
#include "flash_store.h"
#include "maze.h"
#include "params.h"
#include "path.h"
#include "robot_config.h"
#include "search.h"
#include "sim.h"
#include "storage.h"
#include "telemetry.h"

extern int host_verbose;
extern unsigned char fake_flash[FLASH_STORE_SIZE];
extern int fake_flash_writes;
void fake_flash_wipe(void);

static int checks, failures;

#define CHECK(cond) do { \
    checks++; \
    if(!(cond)){ failures++; printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); } \
} while(0)

#define CHECK_EQ(a, b) do { \
    long long va_ = (long long)(a), vb_ = (long long)(b); \
    checks++; \
    if(va_ != vb_){ failures++; printf("FAIL %s:%d: %s == %s (%lld != %lld)\n", __FILE__, __LINE__, #a, #b, va_, vb_); } \
} while(0)

static const plan_costs_t FAST = {FAST_COST_CELL, FAST_COST_TURN};
static const plan_costs_t SEARCH = {SEARCH_COST_CELL, SEARCH_COST_TURN};
static uint16_t cost[MAZE_STATES];

// ---- Independent reference planner: plain O(V^2) Dijkstra ----------------------

static uint16_t ref[MAZE_STATES];

static int ref_index(int x, int y, int h){
    return (y * MAZE_SIZE + x) * 4 + h;
}

static int ref_passable(int x, int y, int h, plan_mode_t mode){
    int8_t e = maze_evidence((uint8_t)x, (uint8_t)y, (heading_t)h);
    int nx = x + (h == 1) - (h == 3), ny = y + (h == 0) - (h == 2);
    if(nx < 0 || ny < 0 || nx >= MAZE_SIZE || ny >= MAZE_SIZE) return 0;
    return mode == PLAN_VERIFIED ? e <= -2 : e <= 0;
}

// backwards = 1: cost to reach `targets`; 0: cost from the state (sx, sy, sh).
static void reference_plan(int backwards, const cellset_t *targets, int sx, int sy, int sh,
                           plan_mode_t mode, plan_costs_t k){
    static unsigned char done[MAZE_STATES];
    memset(done, 0, sizeof(done));
    for(int s = 0; s < MAZE_STATES; s++) ref[s] = PLAN_INF;
    if(backwards){
        for(int y = 0; y < MAZE_SIZE; y++)
            for(int x = 0; x < MAZE_SIZE; x++)
                if((targets->rows[y] >> x) & 1u)
                    for(int h = 0; h < 4; h++) ref[ref_index(x, y, h)] = 0;
    }
    else{
        ref[ref_index(sx, sy, sh)] = 0;
    }
    for(;;){
        int best = -1;
        for(int s = 0; s < MAZE_STATES; s++){
            if(!done[s] && ref[s] != PLAN_INF && (best < 0 || ref[s] < ref[best])) best = s;
        }
        if(best < 0) break;
        done[best] = 1;
        int h = best % 4, x = (best / 4) % MAZE_SIZE, y = (best / 4) / MAZE_SIZE;
        int dx = (h == 1) - (h == 3), dy = (h == 0) - (h == 2);
        int n[3], w[3], count = 0;
        if(backwards){
            if(ref_passable(x - dx, y - dy, h, mode) && x - dx >= 0 && y - dy >= 0 && x - dx < MAZE_SIZE && y - dy < MAZE_SIZE){
                n[count] = ref_index(x - dx, y - dy, h); w[count++] = k.cell;
            }
        }
        else if(ref_passable(x, y, h, mode)){
            n[count] = ref_index(x + dx, y + dy, h); w[count++] = k.cell;
        }
        n[count] = ref_index(x, y, (h + 1) % 4); w[count++] = k.turn;
        n[count] = ref_index(x, y, (h + 3) % 4); w[count++] = k.turn;
        for(int i = 0; i < count; i++){
            if(!done[n[i]] && ref[best] + w[i] < ref[n[i]]) ref[n[i]] = (uint16_t)(ref[best] + w[i]);
        }
    }
}

// Optimal speed-run cost start->goal in the TRUE maze (map restored after).
static uint16_t true_optimum(void){
    maze_snapshot_t saved;
    cellset_t goal;
    maze_export(&saved);
    truth_load_into_map();
    maze_goal_cells(&goal);
    reference_plan(1, &goal, 0, 0, 0, PLAN_VERIFIED, FAST);
    uint16_t best = ref[ref_index(START_X, START_Y, NORTH)];
    CHECK(maze_import(&saved));
    return best;
}

// Every belief held by the map must match the true maze.
static int map_matches_truth(void){
    for(uint8_t x = 0; x < MAZE_SIZE; x++){
        for(uint8_t y = 0; y < MAZE_SIZE; y++){
            for(uint8_t h = 0; h < 4; h++){
                wall_state_t w = maze_wall(x, y, (heading_t)h);
                if(w == WALL_UNKNOWN) continue;
                if((w == WALL_PRESENT) != truth_wall(x, y, (heading_t)h)) return 0;
            }
        }
    }
    return 1;
}

static void randomize_evidence(uint32_t seed){
    srand(seed);
    maze_init();
    for(uint8_t x = 0; x < MAZE_SIZE; x++){
        for(uint8_t y = 0; y < MAZE_SIZE; y++){
            for(uint8_t h = 0; h < 2; h++){     // north and east cover every wall once
                int v = rand() % 7 - 3;         // -3..3
                heading_t d = h ? EAST : NORTH;
                if(v == 3 || v == 2) maze_mark_blocked(x, y, d);
                else if(v == -3) maze_mark_crossed(x, y, d);
                else for(int i = 0; i < (v < 0 ? -v : v); i++) maze_observe(x, y, d, v > 0);
                if(rand() % 3 == 0) maze_mark_visited(x, y);
            }
        }
    }
}

// ---- Unit tests -------------------------------------------------------------------

static void test_crc32(void){
    CHECK_EQ(crc32_update(0, "123456789", 9), 0xCBF43926u);
    uint32_t c = crc32_update(0, "1234", 4);
    CHECK_EQ(crc32_update(c, "56789", 5), 0xCBF43926u);
    CHECK_EQ(crc32_update(0, "", 0), 0);
}

static void test_evidence(void){
    maze_init();
    CHECK_EQ(maze_wall(3, 3, NORTH), WALL_UNKNOWN);
    maze_observe(3, 3, NORTH, 1);
    CHECK_EQ(maze_wall(3, 3, NORTH), WALL_PRESENT);     // one sighting counts at once
    CHECK_EQ(maze_wall(3, 4, SOUTH), WALL_PRESENT);     // same wall seen from the neighbour
    maze_observe(3, 4, SOUTH, 0);
    CHECK_EQ(maze_wall(3, 3, NORTH), WALL_UNKNOWN);     // one contradiction: doubtful
    maze_observe(3, 3, NORTH, 0);
    CHECK_EQ(maze_wall(3, 3, NORTH), WALL_ABSENT);

    for(int i = 0; i < 10; i++) maze_observe(5, 5, EAST, 1);
    CHECK_EQ(maze_evidence(5, 5, EAST), 3);             // saturates
    for(int i = 0; i < 3; i++) maze_observe(6, 5, WEST, 0);
    CHECK_EQ(maze_wall(5, 5, EAST), WALL_UNKNOWN);      // a firm wall needs as many contradictions

    CHECK_EQ(maze_wall(0, 0, WEST), WALL_PRESENT);      // border
    CHECK_EQ(maze_wall(0, 0, SOUTH), WALL_PRESENT);
    CHECK_EQ(maze_wall(15, 15, NORTH), WALL_PRESENT);
    CHECK_EQ(maze_wall(15, 15, EAST), WALL_PRESENT);
    maze_observe(0, 0, WEST, 0);
    maze_mark_crossed(0, 0, WEST);
    CHECK_EQ(maze_wall(0, 0, WEST), WALL_PRESENT);      // the border never opens

    maze_mark_crossed(2, 2, EAST);
    CHECK_EQ(maze_evidence(3, 2, WEST), -3);
    maze_mark_blocked(2, 2, EAST);
    CHECK_EQ(maze_evidence(2, 2, EAST), 2);
    maze_mark_blocked(2, 2, EAST);
    CHECK_EQ(maze_evidence(2, 2, EAST), 3);

    maze_init();
    maze_observe(1, 1, NORTH, 1);
    maze_observe(2, 2, NORTH, 1);
    maze_observe(2, 2, NORTH, 1);
    maze_observe(4, 4, NORTH, 0);
    CHECK_EQ(maze_forget_walls(1), 1);
    CHECK_EQ(maze_wall(1, 1, NORTH), WALL_UNKNOWN);
    CHECK_EQ(maze_wall(2, 2, NORTH), WALL_PRESENT);
    CHECK_EQ(maze_forget_walls(INT8_MAX), 1);
    CHECK_EQ(maze_wall(4, 4, NORTH), WALL_ABSENT);      // open passages are kept

    maze_init();
    maze_mark_visited(3, 4);
    maze_mark_visited(3, 4);
    maze_mark_visited(15, 15);
    CHECK_EQ(maze_visited_count(), 2);
    CHECK(maze_is_visited(3, 4));
    CHECK(!maze_is_visited(4, 3));

    CHECK(!maze_set_goal(3, 3, 2, 2));
    CHECK(!maze_set_goal(0, 0, 16, 0));
    CHECK(maze_set_goal(7, 7, 8, 8));
    CHECK(maze_is_goal(8, 7));
    CHECK(!maze_is_goal(9, 7));
}

static void test_side_doubt(void){
    const float close = FRONT_WALL_REF_MM - SIDE_CLOSE_DOUBT_MM;
    wall_sense_t w;
    // Square to a front wall at the right distance: sides trusted.
    w = (wall_sense_t){SEEN_PRESENT, SEEN_PRESENT, SEEN_PRESENT, 0};
    motion_doubt_sides(&w, 1, 1, 94, 94, 0, SIDE_YAW_DOUBT_MM, close);
    CHECK(w.left == SEEN_PRESENT && w.right == SEEN_PRESENT);
    // Rotated left (FL farther): the right beam swings forward.
    w = (wall_sense_t){SEEN_PRESENT, SEEN_PRESENT, SEEN_PRESENT, 0};
    motion_doubt_sides(&w, 1, 1, 110, 80, 0, SIDE_YAW_DOUBT_MM, close);
    CHECK(w.left == SEEN_PRESENT && w.right == SEEN_DOUBTFUL);
    // Rotated right.
    w = (wall_sense_t){SEEN_PRESENT, SEEN_PRESENT, SEEN_PRESENT, 0};
    motion_doubt_sides(&w, 1, 1, 80, 110, 0, SIDE_YAW_DOUBT_MM, close);
    CHECK(w.left == SEEN_DOUBTFUL && w.right == SEEN_PRESENT);
    // The square offset is taken into account.
    w = (wall_sense_t){SEEN_PRESENT, SEEN_PRESENT, SEEN_PRESENT, 0};
    motion_doubt_sides(&w, 1, 1, 80, 110, -30, SIDE_YAW_DOUBT_MM, close);
    CHECK(w.left == SEEN_PRESENT && w.right == SEEN_PRESENT);
    // Stopped too close to the front wall: both sides doubtful.
    w = (wall_sense_t){SEEN_PRESENT, SEEN_PRESENT, SEEN_PRESENT, 0};
    motion_doubt_sides(&w, 1, 1, 50, 52, 0, SIDE_YAW_DOUBT_MM, close);
    CHECK(w.left == SEEN_DOUBTFUL && w.right == SEEN_DOUBTFUL);
    // Only the right front sensor sees something: only the right side doubted.
    w = (wall_sense_t){SEEN_ABSENT, SEEN_PRESENT, SEEN_PRESENT, 0};
    motion_doubt_sides(&w, 0, 1, 300, 90, 0, SIDE_YAW_DOUBT_MM, close);
    CHECK(w.left == SEEN_PRESENT && w.right == SEEN_DOUBTFUL);
    // Nothing in front: nothing to go on, sides trusted.
    w = (wall_sense_t){SEEN_ABSENT, SEEN_PRESENT, SEEN_PRESENT, 0};
    motion_doubt_sides(&w, 0, 0, 300, 300, 0, SIDE_YAW_DOUBT_MM, close);
    CHECK(w.left == SEEN_PRESENT && w.right == SEEN_PRESENT);
    // "No wall" readings are never doubted: that failure only adds walls.
    w = (wall_sense_t){SEEN_PRESENT, SEEN_ABSENT, SEEN_ABSENT, 0};
    motion_doubt_sides(&w, 1, 1, 50, 52, 0, SIDE_YAW_DOUBT_MM, close);
    CHECK(w.left == SEEN_ABSENT && w.right == SEEN_ABSENT);
}

static curve_t default_curve(void){
    curve_t c;
    CHECK(curve_setup(&c, CURVE_RADIUS_MM, CURVE_RAMP_MM, CURVE_ANGLE_DEG, CURVE_PRE_ADJUST_MM,
                      CURVE_POST_ADJUST_MM, CELL_MM));
    return c;
}

// Seconds the reference of a speed run takes along a route (the simulated
// robot tracks it within a few ms).
static float route_seconds(const int8_t *turns, uint8_t cells, float v_fast, float v_curve){
    const curve_t c = default_curve();
    const run_path_t path = {turns, cells};
    path_run_t r;
    profile_t f, o;
    profile_reset(&f);
    profile_reset(&o);
    if(!path_start(&r, &path, &c, CELL_MM, v_fast, v_curve, PARAM_ACCEL)) return 0.0f;
    uint32_t steps = 0;
    while(!r.done && steps < 1000000u){
        path_step(&r, &f, &o, CONTROL_DT_S);
        steps++;
    }
    return (float)steps * CONTROL_DT_S;
}

static void test_planner_basics(void){
    cellset_t goal;
    maze_init();
    maze_set_goal(7, 7, 8, 8);
    maze_goal_cells(&goal);

    maze_plan_to(&goal, PLAN_OPTIMISTIC, FAST, cost);
    CHECK_EQ(cost[maze_state(0, 0, NORTH)], 14 * FAST_COST_CELL + FAST_COST_TURN);  // 7 up, turn, 7 right
    CHECK_EQ(cost[maze_state(8, 7, WEST)], 0);
    CHECK_EQ(maze_best_action(cost, 0, 0, NORTH, PLAN_OPTIMISTIC, FAST), ACT_FORWARD);
    CHECK_EQ(maze_best_action(cost, 0, 0, SOUTH, PLAN_OPTIMISTIC, FAST), ACT_TURN_LEFT);    // east first: one turn
    CHECK_EQ(maze_best_action(cost, 7, 7, NORTH, PLAN_OPTIMISTIC, FAST), ACT_NONE);         // already there

    maze_plan_to(&goal, PLAN_VERIFIED, FAST, cost);
    CHECK_EQ(cost[maze_state(0, 0, NORTH)], PLAN_INF);  // nothing verified yet
    CHECK_EQ(maze_best_action(cost, 0, 0, NORTH, PLAN_VERIFIED, FAST), ACT_NONE);

    // Dead end facing north: turning around must be one action.
    maze_init();
    maze_mark_blocked(0, 1, NORTH);
    maze_mark_blocked(0, 1, EAST);
    maze_plan_to(&goal, PLAN_OPTIMISTIC, FAST, cost);
    CHECK_EQ(maze_best_action(cost, 0, 1, NORTH, PLAN_OPTIMISTIC, FAST), ACT_TURN_AROUND);

    // Fully known empty maze: routes.
    truth_reset(0);
    truth_load_into_map();
    int8_t turn, turns[PATH_MAX_CELLS];
    uint8_t cells;
    maze_set_goal(0, 5, 0, 5);
    maze_goal_cells(&goal);
    maze_plan_to(&goal, PLAN_VERIFIED, FAST, cost);
    CHECK(maze_route(cost, 0, 0, NORTH, PLAN_VERIFIED, FAST, &turn, turns, PATH_MAX_CELLS, &cells));
    CHECK_EQ(turn, 0);
    CHECK_EQ(cells, 5);
    CHECK(maze_route(cost, 0, 0, SOUTH, PLAN_VERIFIED, FAST, &turn, turns, PATH_MAX_CELLS, &cells));
    CHECK_EQ(turn, 2);
    CHECK_EQ(cells, 5);
    CHECK(!maze_route(cost, 0, 5, EAST, PLAN_VERIFIED, FAST, &turn, turns, PATH_MAX_CELLS, &cells));
    maze_set_goal(15, 0, 15, 0);
    maze_goal_cells(&goal);
    maze_plan_to(&goal, PLAN_VERIFIED, FAST, cost);
    CHECK(maze_route(cost, 0, 0, NORTH, PLAN_VERIFIED, FAST, &turn, turns, PATH_MAX_CELLS, &cells));
    CHECK_EQ(turn, 1);
    CHECK_EQ(cells, 15);
    // Around a corner without stopping: north 5, curve right in the 5th cell, east 3.
    maze_set_goal(3, 5, 3, 5);
    maze_goal_cells(&goal);
    maze_plan_to(&goal, PLAN_VERIFIED, FAST, cost);
    CHECK(maze_route(cost, 0, 0, NORTH, PLAN_VERIFIED, FAST, &turn, turns, PATH_MAX_CELLS, &cells));
    CHECK_EQ(turn, 0);
    CHECK_EQ(cells, 8);
    for(uint8_t i = 0; i < cells; i++) CHECK_EQ(turns[i], i == 4 ? 1 : 0);
    // A route longer than the buffer stops at the centre of its last cell.
    CHECK(maze_route(cost, 0, 0, NORTH, PLAN_VERIFIED, FAST, &turn, turns, 5, &cells));
    CHECK_EQ(cells, 5);
    CHECK_EQ(turns[4], 0);

    // Optimal in time: to reach (3,3), a staircase of 6 cells and 5 turns
    // against a detour of 8 cells and 2 turns. With smooth curves the
    // staircase is faster (curves at 400 mm/s all the way, against
    // accelerating to FAST and braking twice), and so is its cost.
    truth_reset(1);
    truth_set_wall(0, 0, NORTH, 0);     // staircase N E N E N E
    truth_set_wall(0, 1, EAST, 0);
    truth_set_wall(1, 1, NORTH, 0);
    truth_set_wall(1, 2, EAST, 0);
    truth_set_wall(2, 2, NORTH, 0);
    truth_set_wall(2, 3, EAST, 0);
    truth_set_wall(0, 1, NORTH, 0);     // detour: north to (0,4)...
    truth_set_wall(0, 2, NORTH, 0);
    truth_set_wall(0, 3, NORTH, 0);
    truth_set_wall(0, 4, EAST, 0);      // ...east to (3,4)...
    truth_set_wall(1, 4, EAST, 0);
    truth_set_wall(2, 4, EAST, 0);
    truth_set_wall(3, 4, SOUTH, 0);     // ...and down into (3,3)
    truth_load_into_map();
    maze_set_goal(3, 3, 3, 3);
    maze_goal_cells(&goal);
    maze_plan_to(&goal, PLAN_VERIFIED, FAST, cost);
    CHECK_EQ(cost[maze_state(0, 0, NORTH)], 6 * FAST_COST_CELL + 5 * FAST_COST_TURN);
    CHECK(maze_route(cost, 0, 0, NORTH, PLAN_VERIFIED, FAST, &turn, turns, PATH_MAX_CELLS, &cells));
    CHECK_EQ(turn, 0);
    CHECK_EQ(cells, 6);
    static const int8_t staircase[6] = {1, -1, 1, -1, 1, 0}, detour[8] = {0, 0, 0, 1, 0, 0, 1, 0};
    for(uint8_t i = 0; i < cells; i++) CHECK_EQ(turns[i], staircase[i]);
    CHECK(route_seconds(staircase, 6, PARAM_FAST_SPEED, PARAM_CURVE_SPEED) < route_seconds(detour, 8, PARAM_FAST_SPEED,
                                                                                            PARAM_CURVE_SPEED));
}

// SPFA (maze.c) against the independent Dijkstra on random evidence maps, for
// every state, both directions and both modes; then follow the best actions
// and check they realise exactly the planned cost.
static void test_planner_against_reference(void){
    const plan_costs_t cost_models[2] = {FAST, SEARCH};
    int mismatches = 0, walk_errors = 0;
    for(uint32_t seed = 1; seed <= 60; seed++){
        randomize_evidence(seed);
        cellset_t targets;
        cellset_clear(&targets);
        for(int i = 0; i < 1 + (int)(seed % 4); i++) cellset_add(&targets, (uint8_t)(rand() % 16), (uint8_t)(rand() % 16));
        for(int mode = 0; mode < 2; mode++){
            plan_costs_t k = cost_models[seed % 2];
            maze_plan_to(&targets, (plan_mode_t)mode, k, cost);
            reference_plan(1, &targets, 0, 0, 0, (plan_mode_t)mode, k);
            for(int s = 0; s < MAZE_STATES; s++) mismatches += cost[s] != ref[s];

            for(int s = 0; s < MAZE_STATES; s += 7){
                if(cost[s] == PLAN_INF) continue;
                uint8_t x = (uint8_t)((s / 4) % MAZE_SIZE), y = (uint8_t)((s / 4) / MAZE_SIZE);
                heading_t h = (heading_t)(s % 4);
                uint32_t spent = 0;
                for(int step = 0; step < MAZE_STATES; step++){
                    action_t a = maze_best_action(cost, x, y, h, (plan_mode_t)mode, k);
                    if(a == ACT_NONE) break;
                    if(a == ACT_FORWARD){ x = (uint8_t)(x + heading_dx(h)); y = (uint8_t)(y + heading_dy(h)); spent += k.cell; }
                    else if(a == ACT_TURN_LEFT){ h = heading_left(h); spent += k.turn; }
                    else if(a == ACT_TURN_RIGHT){ h = heading_right(h); spent += k.turn; }
                    else{ h = heading_back(h); spent += 2u * k.turn; }
                }
                walk_errors += !cellset_has(&targets, x, y) || spent != cost[s];
                // The same path as a route driven in one go: same cells, same cost.
                int8_t turn, turns[PATH_MAX_CELLS];
                uint8_t cells;
                x = (uint8_t)((s / 4) % MAZE_SIZE);
                y = (uint8_t)((s / 4) / MAZE_SIZE);
                h = (heading_t)(s % 4);
                if(!maze_route(cost, x, y, h, (plan_mode_t)mode, k, &turn, turns, PATH_MAX_CELLS, &cells)){
                    walk_errors += cost[s] != 0;
                    continue;
                }
                spent = (uint32_t)(turn < 0 ? -turn : turn) * k.turn;
                h = (heading_t)((h + turn + 4) & 3);
                for(uint8_t i = 0; i < cells; i++){
                    x = (uint8_t)(x + heading_dx(h));
                    y = (uint8_t)(y + heading_dy(h));
                    h = (heading_t)((h + turns[i] + 4) & 3);
                    spent += k.cell + (turns[i] ? k.turn : 0u);
                }
                walk_errors += !cellset_has(&targets, x, y) || spent != cost[s] || !cells || turns[cells - 1];
            }

            uint8_t sx = (uint8_t)(seed % 16), sy = (uint8_t)((seed * 7) % 16);
            maze_plan_from(sx, sy, (heading_t)(seed % 4), (plan_mode_t)mode, k, cost);
            reference_plan(0, NULL, sx, sy, (int)(seed % 4), (plan_mode_t)mode, k);
            for(int s = 0; s < MAZE_STATES; s++) mismatches += cost[s] != ref[s];
        }
    }
    CHECK_EQ(mismatches, 0);
    CHECK_EQ(walk_errors, 0);
}

static void test_storage(void){
    fake_flash_wipe();
    params_reset();
    CHECK_EQ(storage_load(), STORAGE_EMPTY);

    maze_init();
    maze_observe(4, 4, EAST, 1);
    maze_mark_crossed(0, 0, NORTH);
    maze_mark_visited(4, 4);
    maze_set_goal(3, 2, 3, 2);
    params.search_speed = 123;
    params.turn_ticks = 415;
    params.accel = 4321;
    params.ki = 0.75f;
    CHECK(storage_save());

    maze_init();
    maze_set_goal(7, 7, 8, 8);
    params_reset();
    CHECK_EQ(storage_load(), STORAGE_LOADED);
    CHECK_EQ(maze_wall(4, 4, EAST), WALL_PRESENT);
    CHECK_EQ(maze_evidence(0, 0, NORTH), -3);
    CHECK(maze_is_visited(4, 4));
    CHECK(maze_is_goal(3, 2));
    CHECK(!maze_is_goal(7, 7));
    CHECK_EQ(params.search_speed, 123);
    CHECK_EQ(params.turn_ticks, 415);
    CHECK_EQ(params.accel, 4321);
    CHECK(params.ki == 0.75f);

    // A corrupt record is rejected and leaves RAM untouched.
    maze_init();
    fake_flash[40] ^= 0x55;
    CHECK_EQ(storage_load(), STORAGE_CORRUPT);
    CHECK_EQ(maze_wall(4, 4, EAST), WALL_UNKNOWN);
    fake_flash[40] ^= 0x55;
    CHECK_EQ(storage_load(), STORAGE_LOADED);

    // Saved by firmware with other defaults: ignored even with a valid CRC.
    uint16_t size;
    memcpy(&size, fake_flash + 6, sizeof(size));
    CHECK(size > 8 && size <= FLASH_STORE_SIZE);
    fake_flash[8] ^= 0xFF;          // signature
    uint32_t crc = crc32_update(0, fake_flash, (size_t)size - 4u);
    memcpy(fake_flash + size - 4, &crc, sizeof(crc));
    maze_init();
    params_reset();
    CHECK_EQ(storage_load(), STORAGE_STALE);
    CHECK_EQ(maze_wall(4, 4, EAST), WALL_UNKNOWN);
    CHECK_EQ(params.search_speed, PARAM_SEARCH_SPEED);
    CHECK(params_defaults_signature() == params_defaults_signature());

    // Older record layout (another STORE_VERSION): stale, not corrupt.
    fake_flash[8] ^= 0xFF;
    fake_flash[4] ^= 0x03;          // version
    crc = crc32_update(0, fake_flash, (size_t)size - 4u);
    memcpy(fake_flash + size - 4, &crc, sizeof(crc));
    CHECK_EQ(storage_load(), STORAGE_STALE);
    fake_flash[4] ^= 0x03;
    crc = crc32_update(0, fake_flash, (size_t)size - 4u);
    memcpy(fake_flash + size - 4, &crc, sizeof(crc));
    CHECK_EQ(storage_load(), STORAGE_LOADED);
    params_reset();
    maze_set_goal(GOAL_X0, GOAL_Y0, GOAL_X1, GOAL_Y1);
}

// ---- Simulation --------------------------------------------------------------------

typedef struct {
    int runs, search_ok, fast_ok, optimal, consistent, back_home;
    long blocked, crashes, search_actions, search_cells, search_senses, fast_cells, fast_curves, fast_turns;
} summary_t;

static void print_summary(const char *name, const summary_t *s){
    printf("  %-34s runs %3d | search ok %3d, optimal %3d, map ok %3d | fast ok %3d, home %3d | "
           "blocked %ld crashes %ld | avg search %ld actions %ld cells | avg fast %ld cells %ld curves %ld turns\n",
           name, s->runs, s->search_ok, s->optimal, s->consistent, s->fast_ok, s->back_home,
           s->blocked, s->crashes, s->search_actions / s->runs, s->search_cells / s->runs,
           s->fast_cells / (s->runs ? s->runs : 1), s->fast_curves / (s->runs ? s->runs : 1),
           s->fast_turns / (s->runs ? s->runs : 1));
}

// Full competition cycle on one maze: search, then speed run + return.
static void run_cycle(summary_t *s, double noise, uint32_t seed, double doubt){
    maze_init();
    params_reset();
    fake_flash_wipe();
    sim_reset(noise, seed);
    sim_side_doubt(doubt);
    search_set_home();
    s->runs++;

    run_result_t r = search_explore();
    int home = sim_x == START_X && sim_y == START_Y && sim_h == NORTH;
    if(r == RUN_OK && home && search_ready()) s->search_ok++;
    s->optimal += search_fast_path_cost() == true_optimum();
    s->consistent += map_matches_truth();
    s->blocked += sim_stats.blocked;
    s->crashes += sim_stats.crashes;
    s->search_actions += sim_stats.actions;
    s->search_cells += sim_stats.forward_cells;
    s->search_senses += sim_stats.senses;
    if(r != RUN_OK || !home) return;

    CHECK_EQ(storage_load(), STORAGE_LOADED);   // the search saved its map
    sim_reset(noise, seed ^ 0x9E3779B9u);
    sim_side_doubt(doubt);
    r = search_fast_run();
    if(r == RUN_OK) s->fast_ok++;
    s->back_home += sim_x == START_X && sim_y == START_Y && sim_h == NORTH && search_ready();
    s->blocked += sim_stats.blocked;
    s->crashes += sim_stats.crashes;
    s->fast_cells += sim_stats.forward_cells;
    s->fast_curves += sim_stats.curves;
    s->fast_turns += sim_stats.quarter_turns;
    // The speed run and its return drive whole routes: at most one in-place
    // turn per leg, at the start of each (and more only when a route fails).
    CHECK(sim_stats.paths > 0);
}

static void test_competition_mazes(void){
    printf("simulation (16x16, goal (7,7)-(8,8)):\n");
    maze_set_goal(7, 7, 8, 8);
    const struct { const char *name; uint16_t openings; double noise, doubt; } suites[] = {
        {"perfect mazes", 0, 0.0, 0.0},
        {"mazes with loops", 40, 0.0, 0.0},
        {"open mazes (many loops)", 150, 0.0, 0.0},
        {"loops + 20% doubtful sides", 40, 0.0, 0.2},
        {"loops + 1% sensor noise", 40, 0.01, 0.0},
        {"loops + 3% sensor noise", 40, 0.03, 0.0},
    };
    for(size_t i = 0; i < sizeof(suites) / sizeof(suites[0]); i++){
        summary_t s = {0};
        for(uint32_t m = 1; m <= 100; m++){
            truth_generate(m * 2654435761u + (uint32_t)i, suites[i].openings);
            run_cycle(&s, suites[i].noise, m + 1000u * (uint32_t)i, suites[i].doubt);
        }
        print_summary(suites[i].name, &s);
        if(suites[i].doubt > 0.0){
            // Doubtful readings only withhold information: never a wrong
            // belief, never a crash, always done.
            CHECK_EQ(s.search_ok, s.runs);
            CHECK_EQ(s.consistent, s.runs);
            CHECK_EQ(s.fast_ok, s.runs);
            CHECK_EQ(s.crashes, 0);
        }
        else if(suites[i].noise == 0.0){
            // Perfect sensing: everything must work every time.
            CHECK_EQ(s.search_ok, s.runs);
            CHECK_EQ(s.consistent, s.runs);
            CHECK_EQ(s.optimal, s.runs);
            CHECK_EQ(s.fast_ok, s.runs);
            CHECK_EQ(s.back_home, s.runs);
            CHECK_EQ(s.blocked, 0);
            CHECK_EQ(s.crashes, 0);
        }
        else{
            // Noisy sensing: the robot must still finish and never lose itself.
            CHECK(s.search_ok >= s.runs * 95 / 100);
            CHECK(s.fast_ok >= s.search_ok * 95 / 100);
            CHECK_EQ(s.crashes, 0);
        }
    }
    maze_set_goal(GOAL_X0, GOAL_Y0, GOAL_X1, GOAL_Y1);
}

// The 4x3 practice maze inside the unexplored 16x16 grid.
static void test_practice_maze(void){
    printf("simulation (4x3 practice maze, goal (3,2)):\n");
    maze_set_goal(3, 2, 3, 2);
    summary_t s = {0};
    for(uint32_t m = 0; m < 60; m++){
        truth_reset(1);
        // Carve a random spanning tree inside x<4, y<3 (DFS), plus the
        // layout of the old flood test (only the bottom row joins the halves).
        if(m == 0){
            for(uint8_t x = 0; x < 4; x++)
                for(uint8_t y = 0; y < 3; y++){
                    if(x < 3) truth_set_wall(x, y, EAST, 0);
                    if(y < 2) truth_set_wall(x, y, NORTH, 0);
                }
            truth_set_wall(1, 1, EAST, 1);
            truth_set_wall(1, 2, EAST, 1);
            truth_set_wall(0, 0, EAST, 1);
        }
        else{
            uint8_t seen[4][3] = {{0}};
            uint8_t sx[12], sy[12];
            int top = 0;
            sx[0] = 0; sy[0] = 0; seen[0][0] = 1;
            srand(m);
            while(top >= 0){
                uint8_t x = sx[top], y = sy[top];
                heading_t opts[4];
                int n = 0;
                for(int h = 0; h < 4; h++){
                    int nx = x + heading_dx((heading_t)h), ny = y + heading_dy((heading_t)h);
                    if(nx < 0 || ny < 0 || nx >= 4 || ny >= 3 || seen[nx][ny]) continue;
                    if(x == 0 && y == 0 && h == EAST) continue;
                    opts[n++] = (heading_t)h;
                }
                if(!n){ top--; continue; }
                heading_t h = opts[rand() % n];
                truth_set_wall(x, y, h, 0);
                x = (uint8_t)(x + heading_dx(h));
                y = (uint8_t)(y + heading_dy(h));
                seen[x][y] = 1;
                top++;
                sx[top] = x; sy[top] = y;
            }
            if(m % 2){  // some loops
                truth_set_wall((uint8_t)(1 + rand() % 2), (uint8_t)(rand() % 2), NORTH, 0);
            }
        }
        run_cycle(&s, 0.0, m + 77u, 0.0);
    }
    print_summary("practice mazes", &s);
    CHECK_EQ(s.search_ok, s.runs);
    CHECK_EQ(s.optimal, s.runs);
    CHECK_EQ(s.consistent, s.runs);
    CHECK_EQ(s.fast_ok, s.runs);
    CHECK_EQ(s.crashes + s.blocked, 0);
    maze_set_goal(GOAL_X0, GOAL_Y0, GOAL_X1, GOAL_Y1);
}

static void test_wall_followers(void){
    maze_set_goal(7, 7, 8, 8);
    int ok = 0, runs = 0;
    for(uint32_t m = 1; m <= 40; m++){
        truth_generate(m * 40503u, 0);     // perfect maze: wall following reaches every cell
        for(uint8_t left = 0; left < 2; left++){
            maze_init();
            sim_reset(0.0, m);
            runs++;
            run_result_t r = search_wall_follow(left);
            ok += r == RUN_OK && maze_is_goal(sim_x, sim_y) && sim_stats.crashes == 0 && sim_stats.blocked == 0;
        }
    }
    printf("wall followers: %d/%d reached the goal\n", ok, runs);
    CHECK_EQ(ok, runs);
    maze_set_goal(GOAL_X0, GOAL_Y0, GOAL_X1, GOAL_Y1);
}

static void test_run_control(void){
    maze_set_goal(7, 7, 8, 8);
    truth_generate(12345u, 30);

    // No verified path: the speed run refuses without moving.
    maze_init();
    sim_reset(0.0, 1);
    search_set_home();
    CHECK_EQ(search_fast_run(), RUN_FAILED);
    CHECK_EQ(sim_stats.actions, 0);
    CHECK(search_ready());

    // STOP in the middle of a search.
    maze_init();
    fake_flash_wipe();
    sim_reset(0.0, 1);
    sim_abort_after(15);
    CHECK_EQ(search_explore(), RUN_ABORTED);
    CHECK(!search_ready());
    CHECK_EQ(storage_load(), STORAGE_EMPTY);    // nothing half-done was saved

    // A phantom wall sealing off the goal is repaired, not fatal.
    maze_init();
    sim_reset(0.0, 2);
    for(uint8_t x = 6; x <= 9; x++){
        maze_observe(x, 6, NORTH, 1);
        maze_observe(x, 9, SOUTH, 1);
    }
    for(uint8_t y = 7; y <= 8; y++){
        maze_observe(6, y, EAST, 1);
        maze_observe(9, y, WEST, 1);
    }
    for(uint8_t x = 7; x <= 8; x++){
        maze_observe(x, 7, SOUTH, 1);
        maze_observe(x, 8, NORTH, 1);
        maze_mark_blocked(x, 7, SOUTH);
        maze_mark_blocked(x, 8, NORTH);
    }
    for(uint8_t y = 7; y <= 8; y++){
        maze_mark_blocked(7, y, WEST);
        maze_mark_blocked(8, y, EAST);
    }
    CHECK_EQ(search_explore(), RUN_OK);
    CHECK(map_matches_truth());
    CHECK_EQ(sim_stats.crashes, 0);
    maze_set_goal(GOAL_X0, GOAL_Y0, GOAL_X1, GOAL_Y1);
}

// A wall appears on a straight of the verified route after the search (the
// map was wrong): the speed run stops at the cell before it, notes it, and
// drives on around it.
static void test_fast_run_surprise_wall(void){
    maze_set_goal(7, 7, 8, 8);
    cellset_t goal;
    maze_goal_cells(&goal);
    int runs = 0, ok = 0;
    for(uint32_t m = 1; m <= 30; m++){
        truth_generate(m * 7919u, 60);
        maze_init();
        params_reset();
        fake_flash_wipe();
        sim_reset(0.0, m);
        search_set_home();
        if(search_explore() != RUN_OK) continue;
        maze_plan_to(&goal, PLAN_VERIFIED, FAST, cost);
        int8_t turn, turns[PATH_MAX_CELLS];
        uint8_t cells, x = START_X, y = START_Y;
        maze_route(cost, x, y, NORTH, PLAN_VERIFIED, FAST, &turn, turns, PATH_MAX_CELLS, &cells);
        heading_t h = (heading_t)((NORTH + turn + 4) & 3);
        // The passage out of the second straight cell in a row.
        uint8_t placed = 0;
        for(uint8_t i = 0; i < cells && !placed; i++){
            if(i >= 2 && !turns[i - 1] && !turns[i - 2]){
                truth_set_wall(x, y, h, 1);
                placed = 1;
                break;
            }
            x = (uint8_t)(x + heading_dx(h));
            y = (uint8_t)(y + heading_dy(h));
            h = (heading_t)((h + turns[i] + 4) & 3);
        }
        if(!placed || true_optimum() == PLAN_INF) continue;
        runs++;
        sim_reset(0.0, m + 1u);
        run_result_t r = search_fast_run();
        const int good = r == RUN_OK && sim_stats.blocked == 1 && sim_stats.crashes == 0
                      && maze_wall(x, y, h) == WALL_PRESENT && sim_x == START_X && sim_y == START_Y && search_ready();
        ok += good;
        if(!good) printf("  surprise wall, maze %u: result %d blocked %u crashes %u\n", m, r, sim_stats.blocked,
                         sim_stats.crashes);
    }
    printf("speed runs with a wall the map had as open: %d/%d finished\n", ok, runs);
    CHECK(runs >= 10);
    CHECK_EQ(ok, runs);
    maze_set_goal(GOAL_X0, GOAL_Y0, GOAL_X1, GOAL_Y1);
}

// --demo: one full cycle on the old flood-test practice layout, with the
// firmware's own log and map drawing.
static void demo(void){
    host_verbose = 1;
    maze_set_goal(3, 2, 3, 2);
    truth_reset(1);
    for(uint8_t x = 0; x < 4; x++){
        for(uint8_t y = 0; y < 3; y++){
            if(x < 3) truth_set_wall(x, y, EAST, 0);
            if(y < 2) truth_set_wall(x, y, NORTH, 0);
        }
    }
    truth_set_wall(1, 1, EAST, 1);
    truth_set_wall(1, 2, EAST, 1);
    truth_set_wall(0, 0, EAST, 1);
    maze_init();
    params.log_level = 1;
    sim_reset(0.0, 1);
    search_set_home();
    search_explore();
    search_print_map();
    sim_reset(0.0, 2);
    search_fast_run();
    search_print_map();
}

// --transcript <seed> <openings> [practice]: everything the robot would send
// over Bluetooth during a search and a speed run on one random maze, telemetry
// included, as tools/test_robot_monitor.py consumes it. Lines starting with
// '#' are markers for the test, not firmware output.
// Random 4x3 practice maze (spanning tree, start cell closed to the east)
// plus `openings` random extra passages.
static void practice_truth(uint32_t seed, uint16_t openings){
    uint8_t seen[4][3] = {{0}};
    uint8_t sx[12], sy[12];
    int top = 0;
    truth_reset(1);
    srand(seed);
    sx[0] = 0;
    sy[0] = 0;
    seen[0][0] = 1;
    while(top >= 0){
        uint8_t x = sx[top], y = sy[top];
        heading_t options[4];
        int n = 0;
        for(int h = 0; h < 4; h++){
            int nx = x + heading_dx((heading_t)h), ny = y + heading_dy((heading_t)h);
            if(nx < 0 || ny < 0 || nx >= 4 || ny >= 3 || seen[nx][ny]) continue;
            if(x == 0 && y == 0 && h == EAST) continue;
            options[n++] = (heading_t)h;
        }
        if(!n){
            top--;
            continue;
        }
        heading_t h = options[rand() % n];
        truth_set_wall(x, y, h, 0);
        x = (uint8_t)(x + heading_dx(h));
        y = (uint8_t)(y + heading_dy(h));
        seen[x][y] = 1;
        top++;
        sx[top] = x;
        sy[top] = y;
    }
    for(uint16_t i = 0; i < openings; i++){
        uint8_t x = (uint8_t)(rand() % 4), y = (uint8_t)(rand() % 3);
        if(x < 3 && !(x == 0 && y == 0) && rand() % 2) truth_set_wall(x, y, EAST, 0);
        else if(y < 2) truth_set_wall(x, y, NORTH, 0);
    }
}

// Walls the robot "believes" around the goal before starting: forces the
// map repair path (telemetry must resend the map).
static void phantom_walls(void){
    uint8_t g[4];
    maze_get_goal(g);
    for(uint8_t x = g[0]; x <= g[2]; x++){
        maze_mark_blocked(x, g[1], SOUTH);
        maze_mark_blocked(x, g[3], NORTH);
    }
    for(uint8_t y = g[1]; y <= g[3]; y++){
        maze_mark_blocked(g[0], y, WEST);
        maze_mark_blocked(g[2], y, EAST);
    }
}

static void transcript(uint32_t seed, uint16_t openings, int practice, int phantom){
    host_verbose = 1;
    if(practice){
        maze_set_goal(3, 2, 3, 2);
        practice_truth(seed, openings);
    }
    else{
        maze_set_goal(7, 7, 8, 8);
        truth_generate(seed, openings);
    }
    maze_init();
    if(phantom) phantom_walls();
    params_reset();
    params.log_level = 1;
    fake_flash_wipe();
    sim_reset(0.0, seed);
    search_set_home();
    telemetry_sync(1, TM_COUNTDOWN, 0, 0, NORTH);
    run_result_t r = search_explore();
    telemetry_activity(TM_IDLE);
    printf("#RESULT search %d\n#CHECK\n", (int)r);
    telemetry_sync(1, TM_IDLE, sim_x, sim_y, sim_h);
    if(r != RUN_OK) return;
    sim_reset(0.0, seed + 1);
    telemetry_sync(2, TM_COUNTDOWN, 0, 0, NORTH);
    r = search_fast_run();
    telemetry_activity(TM_IDLE);
    printf("#RESULT fast %d\n#CHECK\n", (int)r);
    telemetry_sync(2, TM_IDLE, sim_x, sim_y, sim_h);
}

// ---- Speed control ------------------------------------------------------------------

static void test_profile(void){
    profile_t p;
    profile_reset(&p);
    // Triangle (never reaches top), then trapezoid: exact arrival at rest.
    const float cases[][3] = {{50.0f, 1000.0f, 3000.0f}, {540.0f, 700.0f, 3000.0f}, {-90.0f, 500.0f, 5000.0f}};
    for(size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); i++){
        profile_reset(&p);
        profile_start(&p, cases[i][0], cases[i][1], 0.0f, cases[i][2]);
        float vmax = 0.0f, amax = 0.0f, alast = 0.0f;
        int steps = 0;
        while(p.active && steps < 10000){
            profile_step(&p, 0.001f);
            vmax = fmaxf(vmax, fabsf(p.speed));
            if(p.active) amax = fmaxf(amax, fabsf(p.accel));
            else alast = fabsf(p.accel);
            steps++;
        }
        CHECK(!p.active);
        CHECK(p.pos == cases[i][0]);
        CHECK(p.speed == 0.0f);
        CHECK(vmax <= cases[i][1] + 0.01f);
        CHECK(amax <= cases[i][2] * 1.01f);
        // The last step drops the few mm/s left to zero at once.
        CHECK(alast <= cases[i][2] * 3.0f);
        // No slower than the ideal time plus a few ms.
        const float d = fabsf(cases[i][0]), v = cases[i][1], a = cases[i][2];
        const float ideal = d > v * v / a ? d / v + v / a : 2.0f * sqrtf(d / a);
        CHECK(steps < (int)(ideal * 1000.0f) + 10);
    }

    // The target moves closer mid-way (front wall): never reverses, stops there.
    profile_reset(&p);
    profile_start(&p, 180.0f, 500.0f, 0.0f, 3000.0f);
    float last = 0.0f;
    uint8_t forward_only = 1;
    for(int i = 0; i < 5000 && p.active; i++){
        if(i == 300) p.target = p.pos + 30.0f;
        profile_step(&p, 0.001f);
        forward_only &= p.pos >= last;
        last = p.pos;
    }
    CHECK(forward_only);
    CHECK(!p.active);
    CHECK(p.pos == p.target);

    // Resume after a pause: from standstill at the given position.
    profile_reset(&p);
    profile_start(&p, 100.0f, 300.0f, 0.0f, 2000.0f);
    for(int i = 0; i < 200; i++) profile_step(&p, 0.001f);
    profile_resume(&p, 60.0f);
    CHECK(p.speed == 0.0f);
    CHECK(p.active);
    for(int i = 0; i < 5000 && p.active; i++) profile_step(&p, 0.001f);
    CHECK(p.pos == 100.0f);
}

static void test_steering_filter(void){
    const steer_config_t k = {
        .kp = 1.0f, .ki = 0.0f, .max_deg = STEER_MAX_DEG, .curve_deg = 2.0f, .slew_mm = STEER_SLEW_MM_PER_MS,
        .track_mm = SIDE_WALL_TRACK_MM, .center_l_mm = LANE_WIDTH_MM / 2.0f,
        .center_r_mm = LANE_WIDTH_MM / 2.0f, .error_max_mm = STEER_ERROR_MAX_MM,
        .bias_window_mm = STEER_BIAS_WINDOW_MM, .delay_steps = 0, .average_steps = 1,
    };
    steer_t s;
    steer_reset(&s);
    for(int i = 0; i < 50; i++) steer_step(&s, &k, 84.0f, 84.0f, 0.5f, 0.0f, 1.0f);
    CHECK(fabsf(s.heading) < 0.01f);
    CHECK_EQ(s.wall, STEER_WALL_BOTH);
    // One wall only: that one.
    for(int i = 0; i < 50; i++) steer_step(&s, &k, 250.0f, 84.0f, 0.5f, 0.0f, 1.0f);
    CHECK_EQ(s.wall, STEER_WALL_RIGHT);
    CHECK(fabsf(s.heading) < 0.01f);
    for(int i = 0; i < 50; i++) steer_step(&s, &k, 84.0f, 84.0f, 0.5f, 0.0f, 1.0f);
    // A post caught by the right beam for 2 ms: the estimate barely moves.
    steer_step(&s, &k, 84.0f, 50.0f, 0.5f, 0.0f, 1.0f);
    steer_step(&s, &k, 84.0f, 50.0f, 0.5f, 0.0f, 1.0f);
    CHECK(fabsf(s.lateral) <= 2.0f * STEER_SLEW_MM_PER_MS + 0.001f);
    // Robot 10 mm left of centre: heads right, KP deg per mm.
    for(int i = 0; i < 100; i++) steer_step(&s, &k, 74.0f, 94.0f, 0.5f, 0.0f, 1.0f);
    CHECK(fabsf(s.heading - 10.0f * k.kp) < 0.01f || fabsf(s.heading - STEER_MAX_DEG) < 0.01f);
    CHECK(s.heading > 0.0f);
    // Fading out at the end of a move: back to straight.
    for(int i = 0; i < 100; i++) steer_step(&s, &k, 74.0f, 94.0f, 0.5f, 0.0f, 0.0f);
    CHECK(fabsf(s.heading) < 0.01f);
    // No walls: hold the heading.
    for(int i = 0; i < 100; i++) steer_step(&s, &k, 250.0f, 250.0f, 0.5f, 0.0f, 1.0f);
    CHECK_EQ(s.wall, STEER_WALL_NONE);
    CHECK(fabsf(s.heading) < 0.01f);
}

static void test_speed_control(void){
    // Nominal robot, 15 mm off-centre: exact distance, centred within the
    // first half, no weaving, at search and speed-run speeds.
    const float speeds[] = {300.0f, 500.0f, 800.0f};
    for(size_t i = 0; i < sizeof(speeds) / sizeof(speeds[0]); i++){
        plant_t p = plant_nominal();
        p.y0 = 8.0f;
        sim_result_t r = sim_straight(&p, 540.0f, speeds[i], 3000.0f, PARAM_KP, PARAM_KI);
        CHECK(fabsf(r.travelled - 540.0f) < 1.0f);
        CHECK(r.fwd_err_max < 5.0f);
        CHECK(r.rot_err_max < 6.0f);
        // The side IR resolve ~2 mm; above STEER_VREF_MM_S the centring
        // works over a longer distance (it weaved on the robot otherwise).
        CHECK(r.y_late < (speeds[i] > STEER_VREF_MM_S ? 6.0f : 3.0f));
        p.y0 = 15.0f;               // a bad start: centred within the move (more slowly
        r = sim_straight(&p, 540.0f, speeds[i], 3000.0f, PARAM_KP, PARAM_KI);     // above STEER_VREF_MM_S)
        CHECK(fabsf(r.y_end) < (speeds[i] > STEER_VREF_MM_S ? 6.0f : 3.0f));
        CHECK(r.crossings <= 4);   // also counts +-0.5 mm wobbles at the centre
        CHECK(fabsf(r.yaw_end) < 2.0f);
        CHECK(r.ms < 3000);
    }
    // Model off by 20 % either way, slower motors, more friction and dead time.
    const plant_t base = plant_nominal();
    plant_t variants[5];
    for(int i = 0; i < 5; i++) variants[i] = base;
    variants[0].gain_l = variants[0].gain_r = 0.8f;
    variants[1].gain_l = variants[1].gain_r = 1.2f;
    variants[2].tau = 0.08f;
    variants[3].friction = 40.0f;
    variants[3].stiction = 160.0f;
    variants[4].dead_ms = 6;    // measured: none
    for(int i = 0; i < 5; i++){
        variants[i].y0 = 10.0f;
        sim_result_t r = sim_straight(&variants[i], 540.0f, 600.0f, 3000.0f, PARAM_KP, PARAM_KI);
        CHECK(fabsf(r.travelled - 540.0f) < 1.5f);
        CHECK(r.y_late < 5.0f);
        CHECK(r.fwd_err_max < 10.0f);
    }
    // Unequal motors (the right one 8 % stronger than modelled): still straight.
    plant_t uneven = base;
    uneven.gain_r = 1.08f;
    sim_result_t r = sim_straight(&uneven, 900.0f, 700.0f, 3000.0f, PARAM_KP, PARAM_KI);
    CHECK(r.y_late < 3.0f);     // 2.0 even with equal motors: IR steps and yaw friction
    // A turn left the robot 5 deg off the corridor: the integral finds it.
    plant_t yawed = base;
    yawed.yaw0 = 5.0f;
    r = sim_straight(&yawed, 540.0f, 500.0f, 3000.0f, PARAM_KP, PARAM_KI);
    CHECK(fabsf(r.y_end) < 2.5f);
    // Out of PWM (flat battery at full speed): rotation keeps priority.
    plant_t flat = base;
    flat.gain_l = flat.gain_r = 0.7f;
    flat.y0 = 5.0f;
    r = sim_straight(&flat, 900.0f, 1000.0f, 3000.0f, PARAM_KP, PARAM_KI);
    CHECK(r.y_late < 6.0f);
    // Turns: exact angle (encoder), quick.
    const float angles[] = {90.0f, -90.0f, 180.0f};
    for(size_t i = 0; i < sizeof(angles) / sizeof(angles[0]); i++){
        r = sim_turn(&base, angles[i], PARAM_TURN_SPEED, PARAM_TURN_ACCEL);
        CHECK(fabsf(r.turned - angles[i]) < 0.5f);
        CHECK(fabsf(r.travelled) < 1.0f);
        CHECK(r.ms < (fabsf(angles[i]) > 90.0f ? 600u : 450u));
    }
}

// ---- Smooth paths --------------------------------------------------------------------

typedef struct {
    double x, y;            // mm, start cell centre = origin, +y = the start heading
    double heading;         // deg, > 0 right
    float v_max, v_curve_max, a_max, alpha_max;
    int steps;
    uint8_t done;
} ref_walk_t;

// Follows the reference of a path the way a perfect robot would: every
// step moves `delta` along the reference heading.
static ref_walk_t walk_reference(path_run_t *r, int max_steps){
    ref_walk_t w = {0};
    profile_t fwd, rot;
    profile_reset(&fwd);
    profile_reset(&rot);
    const double deg = 3.14159265358979 / 180.0;
    for(w.steps = 0; w.steps < max_steps && !r->done; w.steps++){
        const double before = rot.pos;
        path_step(r, &fwd, &rot, CONTROL_DT_S);
        // Real heading: the encoder angle scaled back to a real quarter turn per curve.
        const double mid = 0.5 * (before + rot.pos) * 90.0 / r->curve.angle * deg;
        w.x += fwd.delta * sin(mid);
        w.y += fwd.delta * cos(mid);
        w.v_max = fmaxf(w.v_max, fwd.speed);
        if(rot.speed != 0.0f) w.v_curve_max = fmaxf(w.v_curve_max, fwd.speed);
        if(!r->done) w.a_max = fmaxf(w.a_max, fabsf(fwd.accel));     // the last step drops the last mm/s at once
        w.alpha_max = fmaxf(w.alpha_max, fabsf(rot.accel));
    }
    w.heading = rot.pos * 90.0 / r->curve.angle;
    w.done = r->done;
    return w;
}

static void test_curve_shape(void){
    curve_t c = default_curve();
    // Clothoid-arc-clothoid: R pi/2 + ramp long; symmetric, so the same
    // footprint along both axes (Fresnel integrals: 85.47 mm).
    CHECK(fabsf(c.length - (CURVE_RADIUS_MM * 1.5707963f + CURVE_RAMP_MM)) < 0.01f);
    CHECK(fabsf(c.footprint - 85.47f) < 0.05f);
    CHECK(fabsf(c.pre - (CELL_MM / 2.0f - c.footprint)) < 0.05f);
    CHECK(fabsf(c.pre - c.post) < 0.01f);
    CHECK(curve_progress(&c, 0.0f) == 0.0f && curve_progress(&c, c.length) == 1.0f);
    CHECK(fabsf(curve_progress(&c, 0.5f * c.length) - 0.5f) < 1e-5f);
    // Nearly a pure arc: the ramps push it out by half their length, so
    // radius 90 would overhang the cell; 89 with 1 mm ramps fills it.
    CHECK(!curve_setup(&c, 90.0f, 1.0f, 90.0f, 0.0f, 0.0f, CELL_MM));
    CHECK(curve_setup(&c, 89.0f, 1.0f, 90.0f, 0.0f, 0.0f, CELL_MM));
    CHECK(fabsf(c.footprint - 89.5f) < 0.05f);
    // Adjustments move the start and the exit edge.
    CHECK(curve_setup(&c, CURVE_RADIUS_MM, CURVE_RAMP_MM, 90.0f, 3.0f, -2.0f, CELL_MM));
    CHECK(fabsf(c.pre - (CELL_MM / 2.0f - 85.47f + 3.0f)) < 0.05f);
    CHECK(fabsf(c.post - (CELL_MM / 2.0f - 85.47f - 2.0f)) < 0.05f);
    // Shapes that do not exist or do not fit: curves in consecutive cells would overlap.
    CHECK(!curve_setup(&c, 70.0f, 120.0f, 90.0f, 0.0f, 0.0f, CELL_MM));     // ramps longer than the turn
    CHECK(!curve_setup(&c, 80.0f, 60.0f, 90.0f, 0.0f, 0.0f, CELL_MM));      // footprint 111 mm
    CHECK(!curve_setup(&c, CURVE_RADIUS_MM, CURVE_RAMP_MM, 90.0f, -5.0f, -5.0f, CELL_MM));
    CHECK(!curve_setup(&c, CURVE_RADIUS_MM, 0.0f, 90.0f, 0.0f, 0.0f, CELL_MM));
}

static void test_path_reference(void){
    const curve_t c = default_curve();
    static const struct {
        const char *name;
        uint8_t cells;
        int8_t turn[8];
        double x, y, heading;       // where it must end: a cell centre
    } paths[] = {
        {"one cell", 1, {0}, 0, 180, 0},
        {"five cells", 5, {0}, 0, 900, 0},
        {"corner right", 2, {1, 0}, 180, 180, 90},
        {"corner left after 3", 5, {0, 0, -1, 0, 0}, -360, 540, -90},
        {"curve in the first cell", 2, {-1, 0}, -180, 180, -90},
        {"staircase", 5, {1, -1, 1, -1, 0}, 360, 540, 0},
        {"u-turn over two cells", 3, {1, 1, 0}, 180, 0, 180},
    };
    for(size_t i = 0; i < sizeof(paths) / sizeof(paths[0]); i++){
        const float speeds[][2] = {{700.0f, 400.0f}, {1500.0f, 700.0f}, {300.0f, 300.0f}};
        for(size_t k = 0; k < 3; k++){
            const run_path_t path = {paths[i].turn, paths[i].cells};
            path_run_t r;
            CHECK(path_start(&r, &path, &c, CELL_MM, speeds[k][0], speeds[k][1], 3000.0f));
            ref_walk_t w = walk_reference(&r, 20000);
            const int ok = w.done && fabs(w.x - paths[i].x) < 0.3 && fabs(w.y - paths[i].y) < 0.3
                        && fabs(w.heading - paths[i].heading) < 0.01;
            if(!ok){
                printf("  path %s at %.0f/%.0f: end (%.2f, %.2f) %.2f deg, done %d\n", paths[i].name,
                       (double)speeds[k][0], (double)speeds[k][1], w.x, w.y, w.heading, w.done);
            }
            CHECK(ok);
            CHECK(w.v_max <= speeds[k][0] + 0.01f);
            CHECK(w.v_curve_max <= r.v_curve + 0.01f);
            CHECK(w.a_max <= 3000.0f * 1.01f);
            // Angular acceleration: at most the ramps' v^2 / (R ramp), no step.
            const float alpha = r.v_curve * r.v_curve / (CURVE_RADIUS_MM * CURVE_RAMP_MM) * 57.29578f;
            CHECK(w.alpha_max <= alpha * 1.05f);
        }
    }
    // The curve speed leaves room to stop at the next cell centre after the
    // last curve: at 3000 mm/s^2, sqrt(2 a (post + 90)) = 753 mm/s.
    const int8_t corner[2] = {1, 0};
    const run_path_t path = {corner, 2};
    path_run_t r;
    CHECK(path_start(&r, &path, &c, CELL_MM, 2000.0f, 2000.0f, 3000.0f));
    CHECK(fabsf(r.v_curve - sqrtf(2.0f * 3000.0f * (c.post + 90.0f))) < 0.1f);
    // Malformed paths are refused.
    const int8_t bad_last[2] = {0, 1}, bad_turn[2] = {2, 0};
    CHECK(!path_start(&r, &(run_path_t){bad_last, 2}, &c, CELL_MM, 700.0f, 400.0f, 3000.0f));
    CHECK(!path_start(&r, &(run_path_t){bad_turn, 2}, &c, CELL_MM, 700.0f, 400.0f, 3000.0f));
    CHECK(!path_start(&r, &(run_path_t){corner, 0}, &c, CELL_MM, 700.0f, 400.0f, 3000.0f));
    // A calibrated angle: the encoders turn 88 per real 90, the shape stays.
    curve_t c88;
    CHECK(curve_setup(&c88, CURVE_RADIUS_MM, CURVE_RAMP_MM, 88.0f, 0.0f, 0.0f, CELL_MM));
    CHECK(path_start(&r, &path, &c88, CELL_MM, 700.0f, 400.0f, 3000.0f));
    ref_walk_t w = walk_reference(&r, 20000);
    CHECK(fabs(w.x - 180.0) < 0.3 && fabs(w.y - 180.0) < 0.3);
    CHECK(fabsf(r.heading - 88.0f) < 0.001f);
}

static void test_path_control(void){
    const curve_t c = default_curve();
    profile_t fwd, rot;
    profile_reset(&fwd);
    profile_reset(&rot);
    // PAUSE mid-curve: brakes to a stop on the path, then carries on to the same end.
    const int8_t corner[3] = {0, 1, 0};
    const run_path_t path = {corner, 3};
    path_run_t r;
    CHECK(path_start(&r, &path, &c, CELL_MM, 700.0f, 400.0f, 3000.0f));
    while(r.s < r.curve_start + 0.5f * c.length) path_step(&r, &fwd, &rot, CONTROL_DT_S);
    r.hold = 1;
    for(int i = 0; i < 300; i++) path_step(&r, &fwd, &rot, CONTROL_DT_S);
    CHECK(r.v == 0.0f && !r.done);
    CHECK(r.s < r.curve_start + c.length);     // stopped inside the curve
    const float held = r.s;
    for(int i = 0; i < 100; i++) path_step(&r, &fwd, &rot, CONTROL_DT_S);
    CHECK(r.s == held && rot.speed == 0.0f);
    r.hold = 0;
    for(int i = 0; i < 5000 && !r.done; i++) path_step(&r, &fwd, &rot, CONTROL_DT_S);
    CHECK(r.done && r.s == r.length && fabsf(r.heading - 90.0f) < 0.001f);

    // Cell centres on the straight, as the supervision asks for them.
    const run_path_t straight = {NULL, 5};
    uint8_t entered;
    float centre;
    CHECK(path_start(&r, &straight, &c, CELL_MM, 700.0f, 400.0f, 3000.0f));
    CHECK(path_straight_centre(&r, 100.0f, 0, &entered, &centre) && entered == 0 && centre == 0.0f);
    CHECK(path_straight_centre(&r, 100.0f, 1, &entered, &centre) && entered == 1 && centre == 180.0f);
    CHECK(path_straight_centre(&r, 700.0f, 1, &entered, &centre) && entered == 4 && centre == 720.0f);
    CHECK(path_straight_centre(&r, 5000.0f, 1, &entered, &centre) && entered == 5 && centre == 900.0f);
    CHECK(path_on_straight(&r, 10.0f) && fabsf(path_straight_end(&r) - 900.0f) < 0.001f);
    // Stopping short (a wall where the map had none): ends exactly there.
    for(int i = 0; i < 150; i++) path_step(&r, &fwd, &rot, CONTROL_DT_S);
    CHECK(path_straight_centre(&r, r.s + 200.0f, 1, &entered, &centre));
    r.stop_at = centre;
    for(int i = 0; i < 5000 && !r.done; i++) path_step(&r, &fwd, &rot, CONTROL_DT_S);
    CHECK(r.done && r.s == centre && r.v == 0.0f);

    // After a curve: the straight starts at the curve cell's exit edge.
    const int8_t late[4] = {0, -1, 0, 0};
    const run_path_t after = {late, 4};
    CHECK(path_start(&r, &after, &c, CELL_MM, 700.0f, 400.0f, 3000.0f));
    CHECK(path_straight_centre(&r, 0.0f, 0, &entered, &centre) && entered == 0 && centre == 0.0f);
    CHECK(fabsf(path_straight_end(&r) - (1.5f * CELL_MM + 0.5f * CELL_MM)) < 0.001f);   // the curve cell's centre
    while(r.curves == 0) path_step(&r, &fwd, &rot, CONTROL_DT_S);
    CHECK(!path_on_straight(&r, r.last_curve_end - 1.0f) && path_on_straight(&r, r.last_curve_end));
    CHECK(fabsf(r.first_edge - (r.last_curve_end + c.post)) < 0.001f && r.first == 2);
    CHECK(path_straight_centre(&r, r.s + 300.0f, 1, &entered, &centre) && entered == 4);
    CHECK(!path_straight_centre(&r, r.first_edge, 0, &entered, &centre));   // no centre passed yet
}

// The simulated robot (motors, encoders, the chassis' yaw stick-slip)
// through corners, staircases and u-turns at the default speeds: it must end
// on the cell centre, square, and never stray from the path. There are no
// walls here, so nothing centres the robot: over a long tour the small
// heading errors of each curve add up (the maze's walls take them out).
static void test_path_tracking(void){
    const curve_t c = default_curve();
    static const int8_t corner[3] = {0, 1, 0}, stairs[7] = {0, 1, -1, 1, -1, 0, 0}, u_turn[4] = {0, 1, 1, 0};
    static const int8_t tour[14] = {0, 0, 1, 0, -1, 1, 0, 0, 0, -1, -1, 0, 1, 0};
    const run_path_t paths[] = {{corner, 3}, {stairs, 7}, {u_turn, 4}, {tour, 14}};
    const plant_t base = plant_nominal();
    plant_t plants[7];
    for(int i = 0; i < 7; i++) plants[i] = base;
    plants[1].gain_l = plants[1].gain_r = 0.8f;     // the model off by 20 % either way
    plants[2].gain_l = plants[2].gain_r = 1.2f;
    plants[3].tau = 0.08f;
    plants[4].friction = 40.0f;
    plants[4].stiction = 160.0f;
    plants[5].dead_ms = 6;
    plants[6].gain_r = 1.08f;                       // unequal motors
    for(size_t i = 0; i < sizeof(paths) / sizeof(paths[0]); i++){
        for(int k = 0; k < 7; k++){
            const path_result_t r = sim_path(&plants[k], &paths[i], &c, PARAM_FAST_SPEED, PARAM_CURVE_SPEED, PARAM_ACCEL);
            // Nominal robot: within 2 mm (the tour: 3.5). With the model 20 %
            // off, each curve lags ~2 deg and leaves 3-5 mm at CURVE 400;
            // at 480 a motor 50 % slower than measured leaves 7 mm after a
            // staircase.
            const uint8_t long_tour = i == 3;
            const float tol = k == 0 ? (long_tour ? 3.5f : 2.0f) : (long_tour ? 16.0f : 8.0f);
            const int ok = r.ms > 0 && r.end_err < tol && r.cross_err_max < tol
                        && fabsf(r.heading_err) < (k == 0 ? 0.5f : 1.0f) && r.fwd_err_max < 6.0f && r.rot_err_max < 3.0f;
            if(!ok){
                printf("  path %zu plant %d: end %.2f mm %+.2f deg, off the path %.2f mm, errors %.2f mm %.2f deg, %u ms\n",
                       i, k, (double)r.end_err, (double)r.heading_err, (double)r.cross_err_max,
                       (double)r.fwd_err_max, (double)r.rot_err_max, r.ms);
            }
            CHECK(ok);
        }
        // Curves at the most the motors are asked for (the firmware caps
        // them at ~480 mm/s), between straights slow enough not to run out
        // of PWM themselves: on the path, and never out of PWM.
        const path_result_t r = sim_path(&base, &paths[i], &c, 700.0f, 480.0f, PARAM_ACCEL);
        CHECK(r.end_err < 2.0f && r.cross_err_max < 3.5f && r.pwm_max < CONTROL_PWM_LIMIT);
    }
}

// Motors that cannot keep up (a low battery: 70-80 % of the model) at 900
// / 480 mm/s: the reference slows down instead of running away, so the
// robot never falls far behind it and the curves start where they should.
// Without that, at 80 % the robot fell 24 mm behind and ended a 14-cell tour
// 47 mm off. A blocked robot must still be caught.
static void test_path_governor(void){
    const curve_t c = default_curve();
    static const int8_t corner[3] = {0, 1, 0}, stairs[7] = {0, 1, -1, 1, -1, 0, 0}, u_turn[4] = {0, 1, 1, 0};
    static const int8_t tour[14] = {0, 0, 1, 0, -1, 1, 0, 0, 0, -1, -1, 0, 1, 0};
    const run_path_t paths[] = {{corner, 3}, {stairs, 7}, {u_turn, 4}, {tour, 14}, {NULL, 6}};
    const plant_t base = plant_nominal();
    for(size_t i = 0; i < sizeof(paths) / sizeof(paths[0]); i++){
        const uint8_t long_tour = i == 3;
        // A robot that keeps up never makes the reference wait.
        path_result_t r = sim_path(&base, &paths[i], &c, 900.0f, 480.0f, PARAM_ACCEL);
        CHECK(r.scale_min == 1.0f);
        const float gains[] = {0.8f, 0.7f};
        for(size_t g = 0; g < 2; g++){
            plant_t weak = base;
            weak.gain_l = weak.gain_r = gains[g];
            r = sim_path(&weak, &paths[i], &c, 900.0f, 480.0f, PARAM_ACCEL);
            const float tol = long_tour ? 18.0f : 9.0f;
            const int ok = r.ms > 0 && r.fwd_err_max < 6.0f && r.scale_min < 0.9f && r.end_err < tol
                        && r.cross_err_max < tol && r.stall_ms == 0;
            if(!ok){
                printf("  governor path %zu motors x%.1f: behind %.2f mm, pace %.2f, end %.2f mm, off the path %.2f mm\n",
                       i, (double)gains[g], (double)r.fwd_err_max, (double)r.scale_min, (double)r.end_err,
                       (double)r.cross_err_max);
            }
            CHECK(ok);
        }
    }
    plant_t blocked = base;
    blocked.gain_l = blocked.gain_r = 0.001f;
    const path_result_t r = sim_path(&blocked, &paths[4], &c, 900.0f, 480.0f, PARAM_ACCEL);
    CHECK(r.stall_ms > 0 && r.stall_ms < 500);
}

// host_tests --control: the numbers behind test_speed_control(), for tuning.
static void control_report(void){
    printf("recta 540 mm, 15 mm descentrado (KP %.2f KI %.2f):\n", (double)PARAM_KP, (double)PARAM_KI);
    const float speeds[] = {300.0f, 500.0f, 800.0f, 1000.0f};
    for(size_t i = 0; i < sizeof(speeds) / sizeof(speeds[0]); i++){
        plant_t p = plant_nominal();
        p.y0 = 15.0f;
        sim_result_t r = sim_straight(&p, 540.0f, speeds[i], 3000.0f, PARAM_KP, PARAM_KI);
        printf("  %4.0f mm/s: %4u ms, recorrido %.2f mm, error max %.2f mm / %.2f deg, y 2a mitad %.2f mm,"
               " final %+.2f mm %+.2f deg, cruces %d, PWM max %d\n", (double)speeds[i], r.ms, (double)r.travelled,
               (double)r.fwd_err_max, (double)r.rot_err_max, (double)r.y_late, (double)r.y_end, (double)r.yaw_end,
               r.crossings, r.pwm_max);
    }
    const float angles[] = {90.0f, 180.0f};
    for(size_t i = 0; i < 2; i++){
        plant_t p = plant_nominal();
        sim_result_t r = sim_turn(&p, angles[i], PARAM_TURN_SPEED, PARAM_TURN_ACCEL);
        printf("giro %.0f: %u ms, girado %.2f deg, error max %.2f deg, desplazamiento %.2f mm\n", (double)angles[i],
               r.ms, (double)r.turned, (double)r.rot_err_max, (double)r.travelled);
    }
    const curve_t c = default_curve();
    printf("curvas (radio %.0f, rampas %.0f: %.1f mm, recta antes/despues %.1f mm), FAST %d:\n",
           (double)c.radius, (double)c.ramp, (double)c.length, (double)c.pre, PARAM_FAST_SPEED);
    static const int8_t corner[3] = {0, 1, 0}, stairs[7] = {0, 1, -1, 1, -1, 0, 0}, u_turn[4] = {0, 1, 1, 0};
    const struct { const char *name; run_path_t path; } paths[] = {
        {"esquina", {corner, 3}}, {"escalera", {stairs, 7}}, {"media vuelta", {u_turn, 4}},
    };
    const float curve_speeds[] = {300.0f, 400.0f, 500.0f, 600.0f, 700.0f};
    for(size_t i = 0; i < sizeof(paths) / sizeof(paths[0]); i++){
        for(size_t j = 0; j < sizeof(curve_speeds) / sizeof(curve_speeds[0]); j++){
            const plant_t p = plant_nominal();
            const path_result_t r = sim_path(&p, &paths[i].path, &c, PARAM_FAST_SPEED, curve_speeds[j], PARAM_ACCEL);
            printf("  %-12s %3.0f mm/s: %4u ms, final a %.2f mm y %+.2f deg, fuera de la ruta %.2f mm,"
                   " error max %.2f mm / %.2f deg, PWM max %d\n", paths[i].name, (double)curve_speeds[j], r.ms,
                   (double)r.end_err, (double)r.heading_err, (double)r.cross_err_max, (double)r.fwd_err_max,
                   (double)r.rot_err_max, r.pwm_max);
        }
    }
}

// host_tests --costs [fast curve]: speed-run time of the planner's route
// start -> goal for several (cell, turn) cost pairs, on fully known random
// mazes, to choose FAST_COST_CELL / FAST_COST_TURN.
static void costs_report(float v_fast, float v_curve){
    static const plan_costs_t pairs[] = {
        {2, 4}, {2, 3}, {2, 2}, {2, 1}, {3, 4}, {3, 2}, {3, 1}, {4, 3}, {4, 1}, {5, 2}, {5, 3},
    };
    const size_t n_pairs = sizeof(pairs) / sizeof(pairs[0]);
    const uint16_t openings[] = {0, 40, 150};
    int8_t turns[PATH_MAX_CELLS];
    printf("carrera rapida FAST %.0f, curvas %.0f mm/s, ACCEL %d: segundos medios salida->meta (celdas, curvas)\n",
           (double)v_fast, (double)v_curve, PARAM_ACCEL);
    maze_set_goal(7, 7, 8, 8);
    cellset_t goal;
    maze_goal_cells(&goal);
    for(size_t o = 0; o < sizeof(openings) / sizeof(openings[0]); o++){
        double seconds[16] = {0}, cells_sum[16] = {0}, curves_sum[16] = {0};
        int best_count[16] = {0};
        const int mazes = 200;
        for(int m = 1; m <= mazes; m++){
            truth_generate((uint32_t)m * 2654435761u + o, openings[o]);
            truth_load_into_map();
            float t[16], best = 1e9f;
            for(size_t k = 0; k < n_pairs; k++){
                int8_t turn;
                uint8_t cells = 0;
                maze_plan_to(&goal, PLAN_VERIFIED, pairs[k], cost);
                maze_route(cost, START_X, START_Y, NORTH, PLAN_VERIFIED, pairs[k], &turn, turns, PATH_MAX_CELLS, &cells);
                t[k] = route_seconds(turns, cells, v_fast, v_curve) + (turn ? 0.3f * (float)(turn < 0 ? -turn : turn) : 0.0f);
                seconds[k] += t[k];
                cells_sum[k] += cells;
                for(uint8_t i = 0; i < cells; i++) curves_sum[k] += turns[i] != 0;
                best = fminf(best, t[k]);
            }
            for(size_t k = 0; k < n_pairs; k++) best_count[k] += t[k] <= best + 0.001f;
        }
        printf("  %3u aberturas extra:\n", openings[o]);
        for(size_t k = 0; k < n_pairs; k++){
            printf("    celda %u giro %u: %.3f s (%.1f celdas, %.1f curvas), el mejor en %d de %d\n", pairs[k].cell,
                   pairs[k].turn, seconds[k] / mazes, cells_sum[k] / mazes, curves_sum[k] / mazes, best_count[k], mazes);
        }
    }
    maze_set_goal(GOAL_X0, GOAL_Y0, GOAL_X1, GOAL_Y1);
}

int main(int argc, char **argv){
    host_verbose = argc > 1 && strcmp(argv[1], "-v") == 0;
    fake_flash_wipe();
    if(argc > 1 && strcmp(argv[1], "--demo") == 0){
        demo();
        return 0;
    }
    if(argc > 1 && strcmp(argv[1], "--control") == 0){
        control_report();
        return 0;
    }
    if(argc > 1 && strcmp(argv[1], "--costs") == 0){
        costs_report(argc > 3 ? (float)atof(argv[2]) : PARAM_FAST_SPEED, argc > 3 ? (float)atof(argv[3]) : PARAM_CURVE_SPEED);
        return 0;
    }
    if(argc > 3 && strcmp(argv[1], "--transcript") == 0){
        int practice = 0, phantom = 0;
        for(int i = 4; i < argc; i++){
            practice |= strcmp(argv[i], "practice") == 0;
            phantom |= strcmp(argv[i], "phantom") == 0;
        }
        transcript((uint32_t)strtoul(argv[2], NULL, 10), (uint16_t)strtoul(argv[3], NULL, 10), practice, phantom);
        return 0;
    }

    test_crc32();
    test_evidence();
    test_side_doubt();
    test_planner_basics();
    test_planner_against_reference();
    test_storage();
    test_run_control();
    test_fast_run_surprise_wall();
    test_wall_followers();
    test_practice_maze();
    test_competition_mazes();
    test_profile();
    test_steering_filter();
    test_speed_control();
    test_curve_shape();
    test_path_reference();
    test_path_control();
    test_path_tracking();
    test_path_governor();

    printf("%d checks, %d failures\n", checks, failures);
    return failures ? 1 : 0;
}
