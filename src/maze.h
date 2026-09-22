#ifndef MAZE_H
#define MAZE_H

#include <stdint.h>

#define MAZE_SIZE 16

// Robot heading / absolute wall direction (shared: no diagonals, so a heading
// IS an absolute direction). Enum order matches wall bitmask bit position.
typedef enum {UP_DIR, RIGHT_DIR, DOWN_DIR, LEFT_DIR} robot_heading_t;

#define WALL_BIT(dir) (1 << (dir))

extern uint8_t maze_walls[MAZE_SIZE][MAZE_SIZE];
extern uint8_t maze_visited[MAZE_SIZE][MAZE_SIZE];
extern uint16_t maze_flood[MAZE_SIZE][MAZE_SIZE];

void maze_init(void);
void maze_set_wall(int8_t x, int8_t y, robot_heading_t dir);
void maze_observe_wall(int8_t x, int8_t y, robot_heading_t dir, uint8_t present);
uint8_t maze_has_wall(int8_t x, int8_t y, robot_heading_t dir);
robot_heading_t maze_left_of(robot_heading_t heading);
robot_heading_t maze_right_of(robot_heading_t heading);

// Goal defaults (in maze_init) to the real 16x16 competition center 2x2 block.
// Call maze_set_goal() afterwards to point at a smaller practice maze's goal.
void maze_set_goal(int8_t x_min, int8_t y_min, int8_t x_max, int8_t y_max);
uint8_t maze_is_goal(int8_t x, int8_t y);

// Flood fill (modified/classic): BFS distance-to-goal per cell, respecting
// known walls (unexplored walls are implicitly "open" since maze_walls
// defaults to 0). Recompute any time new walls are learned.
void maze_compute_flood(void);

// Direction to move from (x,y) towards the goal, preferring to keep the
// current heading on ties (fewer turns = simpler/more elegant path).
robot_heading_t maze_next_move(int8_t x, int8_t y, robot_heading_t current_heading);

#endif // MAZE_H
