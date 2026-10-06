#ifndef MAZE_H
#define MAZE_H

#include <stdint.h>

// Maze map + path planner: pure logic, host-tested.

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
// Signed evidence per wall in [-3, 3]: > 0 wall, <= 0 passable to explore, <= -2 trusted by speed runs; the border is a wall.
typedef enum { WALL_UNKNOWN, WALL_PRESENT, WALL_ABSENT } wall_state_t;

void maze_init(void);   // forget all interior walls and visited cells (goal is kept)
void maze_observe(uint8_t x, uint8_t y, heading_t dir, uint8_t present);
void maze_mark_crossed(uint8_t x, uint8_t y, heading_t dir);   // drove through it: surely open
void maze_mark_blocked(uint8_t x, uint8_t y, heading_t dir);   // hit an obstacle there: surely closed
wall_state_t maze_wall(uint8_t x, uint8_t y, heading_t dir);
int8_t maze_evidence(uint8_t x, uint8_t y, heading_t dir);     // 3 for the border
// Forgets walls with evidence in [1, max_evidence] except cell (keep_x, keep_y)'s (MAZE_SIZE: none); returns how many.
uint16_t maze_forget_walls(int8_t max_evidence, uint8_t keep_x, uint8_t keep_y);

void maze_mark_visited(uint8_t x, uint8_t y);   // all four walls of the cell have been seen
uint8_t maze_is_visited(uint8_t x, uint8_t y);
uint16_t maze_visited_count(void);

// ---- Goal ------------------------------------------------------------------------
uint8_t maze_set_goal(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1);   // 0 if invalid
void maze_get_goal(uint8_t goal[4]);
// ERASE / mode 3: forget the map and go back to the build's default goal.
void maze_erase(void);
uint8_t maze_is_goal(uint8_t x, uint8_t y);
void maze_goal_cells(cellset_t *out);

// ---- Planner -------------------------------------------------------------------------
// Shortest paths over (cell, heading): a cell costs `cell`, a quarter turn `turn`, so paths are optimal in time.
typedef struct { uint8_t cell, turn; } plan_costs_t;

typedef enum {
    PLAN_OPTIMISTIC,    // unknown walls are open: exploration
    PLAN_VERIFIED,      // only passages seen open twice or crossed: speed runs
} plan_mode_t;

typedef enum { ACT_NONE, ACT_FORWARD, ACT_TURN_LEFT, ACT_TURN_RIGHT, ACT_TURN_AROUND } action_t;

// Cost to any of `targets` for every state (MAZE_STATES entries); PLAN_INF = unreachable.
void maze_plan_to(const cellset_t *targets, plan_mode_t mode, plan_costs_t costs, uint16_t *cost);
// Cost from the state (x, y, h) to every state.
void maze_plan_from(uint8_t x, uint8_t y, heading_t h, plan_mode_t mode, plan_costs_t costs, uint16_t *cost);
// States popped by every plan so far (the planner's work; wraps).
uint32_t maze_plan_pops(void);

// Best next action from a maze_plan_to() result with the same mode and costs; ACT_NONE at a target or unreachable.
action_t maze_best_action(const uint16_t *cost, uint8_t x, uint8_t y, heading_t h,
                          plan_mode_t mode, plan_costs_t costs);

// The optimal path as a speed run drives it: a turn in place first, then the cells with the turn made in each; 0 if none.
uint8_t maze_route(const uint16_t *cost, uint8_t x, uint8_t y, heading_t h, plan_mode_t mode, plan_costs_t costs,
                   int8_t *turn, int8_t *turns, uint8_t max, uint8_t *cells);

// ---- Persistence -----------------------------------------------------------------------
// Packed for storage.c: each wall's evidence + 3 in a nibble.
typedef struct {
    uint8_t walls[MAZE_SIZE][MAZE_SIZE];    // [x][y]: the wall north of (x, y) in the low nibble, east in the high one
    uint16_t visited[MAZE_SIZE];            // bit x of row y
    uint8_t goal[4];                        // x0, y0, x1, y1
} maze_snapshot_t;

void maze_export(maze_snapshot_t *out);
uint8_t maze_import(const maze_snapshot_t *in);   // 0 (and map untouched) if invalid

#endif // MAZE_H
