#ifndef SEARCH_H
#define SEARCH_H

#include <stdint.h>
#include "maze.h"

// Run strategies. Pure logic on top of motion.h and maze.h (tested on the PC
// by test/host). Every run starts at the start cell facing north.

typedef enum { RUN_OK, RUN_ABORTED, RUN_FAILED } run_result_t;

// Explores to the goal, keeps exploring cells that could still shorten the
// speed-run path, returns to the start (exploring on the way), faces north
// and saves the map.
run_result_t search_explore(void);
// Speed run over verified passages with merged straights, then back to the
// start and save. Refuses without moving if no verified path exists yet.
run_result_t search_fast_run(void);
// Left- or right-hand wall follower until the goal.
run_result_t search_wall_follow(uint8_t left_hand);

// 1 while the robot is known to be at the start cell facing north.
uint8_t search_ready(void);
void search_set_home(void);     // the robot was placed at the start facing north
void search_set_lost(void);     // moved by something else: no longer at the start
void search_pose(uint8_t *x, uint8_t *y, heading_t *h);

// Planner-based reports: robot stopped only (they reuse the run's buffers).
uint16_t search_fast_path_cost(void);   // verified start->goal cost, PLAN_INF if none
void search_print_map(void);

#endif // SEARCH_H
