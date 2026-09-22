#ifndef MAZE_H
#define MAZE_H

#include <stdint.h>

// Maze map + path planner. Pure logic (no HAL): also built and tested on the
// PC by test/host.

#define MAZE_SIZE    16
#define MAZE_STATES  (MAZE_SIZE * MAZE_SIZE * 4)    // planner states: (cell, heading)
#define PLAN_INF     0xFFFFu

// Absolute direction; also the robot heading. N = +y, E = +x.
typedef enum { NORTH = 0, EAST = 1, SOUTH = 2, WEST = 3 } heading_t;

static inline heading_t heading_left(heading_t h)  { return (heading_t)((h + 3) & 3); }
static inline heading_t heading_right(heading_t h) { return (heading_t)((h + 1) & 3); }
static inline heading_t heading_back(heading_t h)  { return (heading_t)((h + 2) & 3); }
static inline int8_t heading_dx(heading_t h) { return (int8_t)(h == EAST ? 1 : h == WEST ? -1 : 0); }
static inline int8_t heading_dy(heading_t h) { return (int8_t)(h == NORTH ? 1 : h == SOUTH ? -1 : 0); }

static inline uint16_t maze_state(uint8_t x, uint8_t y, heading_t h){
    return (uint16_t)((((uint16_t)y * MAZE_SIZE + x) << 2) | (uint16_t)h);
}

// Set of cells: bit x of rows[y].
typedef struct { uint16_t rows[MAZE_SIZE]; } cellset_t;

static inline void cellset_clear(cellset_t *s){
    for(uint8_t y = 0; y < MAZE_SIZE; y++) s->rows[y] = 0;
}
static inline void cellset_add(cellset_t *s, uint8_t x, uint8_t y){
    s->rows[y] = (uint16_t)(s->rows[y] | (1u << x));
}
static inline uint8_t cellset_has(const cellset_t *s, uint8_t x, uint8_t y){
    return (uint8_t)((s->rows[y] >> x) & 1u);
}

// ---- Walls ---------------------------------------------------------------------
// Every interior wall carries signed evidence in [-3, +3], shared by the two
// cells it separates: each sighting moves it one step towards "present" (+)
// or "absent" (-). > 0 is a wall, <= 0 is passable for exploration, and
// <= -2 (two consistent sightings, or physically crossed) is trusted by speed
// runs. So one reading takes effect immediately, yet a single wrong reading
// never outweighs repeated consistent ones. The outer border is always a wall.
typedef enum { WALL_UNKNOWN, WALL_PRESENT, WALL_ABSENT } wall_state_t;

void maze_init(void);   // forget all interior walls and visited cells (goal is kept)
void maze_observe(uint8_t x, uint8_t y, heading_t dir, uint8_t present);
void maze_mark_crossed(uint8_t x, uint8_t y, heading_t dir);   // drove through it: surely open
void maze_mark_blocked(uint8_t x, uint8_t y, heading_t dir);   // hit an obstacle there: surely closed
wall_state_t maze_wall(uint8_t x, uint8_t y, heading_t dir);
int8_t maze_evidence(uint8_t x, uint8_t y, heading_t dir);     // 3 for the border
// Forgets walls with evidence in [1, max_evidence]. Returns how many.
uint16_t maze_forget_walls(int8_t max_evidence);

void maze_mark_visited(uint8_t x, uint8_t y);   // all four walls of the cell have been seen
uint8_t maze_is_visited(uint8_t x, uint8_t y);
uint16_t maze_visited_count(void);

// ---- Goal ------------------------------------------------------------------------
uint8_t maze_set_goal(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1);   // 0 if invalid
void maze_get_goal(uint8_t goal[4]);
uint8_t maze_is_goal(uint8_t x, uint8_t y);
void maze_goal_cells(cellset_t *out);

// ---- Planner -------------------------------------------------------------------------
// Shortest paths over (cell, heading) states: driving one cell forward costs
// `cell`, turning 90 degrees in place costs `turn` (180 = two turns). So paths
// are optimal in *time*, not just in cells: long straights beat staircases.
typedef struct { uint8_t cell, turn; } plan_costs_t;

typedef enum {
    PLAN_OPTIMISTIC,    // unknown walls are open: exploration
    PLAN_VERIFIED,      // only passages seen open twice or crossed: speed runs
} plan_mode_t;

typedef enum { ACT_NONE, ACT_FORWARD, ACT_TURN_LEFT, ACT_TURN_RIGHT, ACT_TURN_AROUND } action_t;

// Cost to reach any cell of `targets` (arriving with any heading), for every
// state. `cost` must hold MAZE_STATES entries; PLAN_INF = unreachable.
void maze_plan_to(const cellset_t *targets, plan_mode_t mode, plan_costs_t costs, uint16_t *cost);
// Cost from the state (x, y, h) to every state.
void maze_plan_from(uint8_t x, uint8_t y, heading_t h, plan_mode_t mode, plan_costs_t costs, uint16_t *cost);

// Best next action at (x, y, h) following a maze_plan_to() result computed
// with the same mode and costs. ACT_NONE when at a target or unreachable.
action_t maze_best_action(const uint16_t *cost, uint8_t x, uint8_t y, heading_t h,
                          plan_mode_t mode, plan_costs_t costs);

// First leg of the optimal path from (x, y, h): an in-place turn (quarter
// turns: 0, -1 = left, +1 = right, 2 = around) followed by `cells` straight
// cells. Returns 0 when there is nothing to do (at a target or unreachable).
uint8_t maze_first_segment(const uint16_t *cost, uint8_t x, uint8_t y, heading_t h,
                           plan_mode_t mode, plan_costs_t costs, int8_t *turn, uint8_t *cells);

// ---- Persistence -----------------------------------------------------------------------
typedef struct {
    int8_t north[MAZE_SIZE][MAZE_SIZE];     // [x][y]: evidence of the wall north of (x, y)
    int8_t east[MAZE_SIZE][MAZE_SIZE];      // [x][y]: evidence of the wall east of (x, y)
    uint16_t visited[MAZE_SIZE];            // bit x of row y
    uint8_t goal[4];                        // x0, y0, x1, y1
} maze_snapshot_t;

void maze_export(maze_snapshot_t *out);
uint8_t maze_import(const maze_snapshot_t *in);   // 0 (and map untouched) if invalid

#endif // MAZE_H
