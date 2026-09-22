// Host tests for the robot's decision logic. Exit code != 0 on any failure.
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "crc32.h"
#include "flash_store.h"
#include "maze.h"
#include "params.h"
#include "robot_config.h"
#include "search.h"
#include "sim.h"
#include "storage.h"

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

    // Fully known empty maze: segments.
    truth_reset(0);
    truth_load_into_map();
    int8_t turn;
    uint8_t cells;
    maze_set_goal(0, 5, 0, 5);
    maze_goal_cells(&goal);
    maze_plan_to(&goal, PLAN_VERIFIED, FAST, cost);
    CHECK(maze_first_segment(cost, 0, 0, NORTH, PLAN_VERIFIED, FAST, &turn, &cells));
    CHECK_EQ(turn, 0);
    CHECK_EQ(cells, 5);
    CHECK(maze_first_segment(cost, 0, 0, SOUTH, PLAN_VERIFIED, FAST, &turn, &cells));
    CHECK_EQ(turn, 2);
    CHECK_EQ(cells, 5);
    CHECK(!maze_first_segment(cost, 0, 5, EAST, PLAN_VERIFIED, FAST, &turn, &cells));
    maze_set_goal(15, 0, 15, 0);
    maze_goal_cells(&goal);
    maze_plan_to(&goal, PLAN_VERIFIED, FAST, cost);
    CHECK(maze_first_segment(cost, 0, 0, NORTH, PLAN_VERIFIED, FAST, &turn, &cells));
    CHECK_EQ(turn, 1);
    CHECK_EQ(cells, 15);

    // Fewer turns beat fewer cells: to reach (3,3), a staircase of 6 cells and
    // 5 turns loses to a detour of 8 cells and 2 turns.
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
    CHECK_EQ(cost[maze_state(0, 0, NORTH)], 8 * FAST_COST_CELL + 2 * FAST_COST_TURN);
    CHECK(maze_first_segment(cost, 0, 0, NORTH, PLAN_VERIFIED, FAST, &turn, &cells));
    CHECK_EQ(turn, 0);
    CHECK_EQ(cells, 4);     // straight up the detour, not into the staircase
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
    params.kd = 12.5f;
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
    CHECK(params.kd == 12.5f);

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
    maze_set_goal(GOAL_X0, GOAL_Y0, GOAL_X1, GOAL_Y1);
}

// ---- Simulation --------------------------------------------------------------------

typedef struct {
    int runs, search_ok, fast_ok, optimal, consistent, back_home;
    long blocked, crashes, search_actions, search_cells, search_senses, fast_cells, fast_turns;
} summary_t;

static void print_summary(const char *name, const summary_t *s){
    printf("  %-34s runs %3d | search ok %3d, optimal %3d, map ok %3d | fast ok %3d, home %3d | "
           "blocked %ld crashes %ld | avg search %ld actions %ld cells | avg fast %ld cells %ld turns\n",
           name, s->runs, s->search_ok, s->optimal, s->consistent, s->fast_ok, s->back_home,
           s->blocked, s->crashes, s->search_actions / s->runs, s->search_cells / s->runs,
           s->fast_cells / (s->runs ? s->runs : 1), s->fast_turns / (s->runs ? s->runs : 1));
}

// Full competition cycle on one maze: search, then speed run + return.
static void run_cycle(summary_t *s, double noise, uint32_t seed){
    maze_init();
    params_reset();
    fake_flash_wipe();
    sim_reset(noise, seed);
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
    r = search_fast_run();
    if(r == RUN_OK) s->fast_ok++;
    s->back_home += sim_x == START_X && sim_y == START_Y && sim_h == NORTH && search_ready();
    s->blocked += sim_stats.blocked;
    s->crashes += sim_stats.crashes;
    s->fast_cells += sim_stats.forward_cells;
    s->fast_turns += sim_stats.quarter_turns;
}

static void test_competition_mazes(void){
    printf("simulation (16x16, goal (7,7)-(8,8)):\n");
    maze_set_goal(7, 7, 8, 8);
    const struct { const char *name; uint16_t openings; double noise; } suites[] = {
        {"perfect mazes", 0, 0.0},
        {"mazes with loops", 40, 0.0},
        {"open mazes (many loops)", 150, 0.0},
        {"loops + 1% sensor noise", 40, 0.01},
        {"loops + 3% sensor noise", 40, 0.03},
    };
    for(size_t i = 0; i < sizeof(suites) / sizeof(suites[0]); i++){
        summary_t s = {0};
        for(uint32_t m = 1; m <= 100; m++){
            truth_generate(m * 2654435761u + (uint32_t)i, suites[i].openings);
            run_cycle(&s, suites[i].noise, m + 1000u * (uint32_t)i);
        }
        print_summary(suites[i].name, &s);
        if(suites[i].noise == 0.0){
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
        run_cycle(&s, 0.0, m + 77u);
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

int main(int argc, char **argv){
    host_verbose = argc > 1 && strcmp(argv[1], "-v") == 0;
    fake_flash_wipe();
    if(argc > 1 && strcmp(argv[1], "--demo") == 0){
        demo();
        return 0;
    }

    test_crc32();
    test_evidence();
    test_planner_basics();
    test_planner_against_reference();
    test_storage();
    test_run_control();
    test_wall_followers();
    test_practice_maze();
    test_competition_mazes();

    printf("%d checks, %d failures\n", checks, failures);
    return failures ? 1 : 0;
}
