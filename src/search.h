#ifndef SEARCH_H
#define SEARCH_H

#include <stdint.h>
#include "maze.h"

// Run strategies: pure logic on motion.h and maze.h (host-tested). Every run starts at the start facing north.

typedef enum { RUN_OK, RUN_ABORTED, RUN_FAILED } run_result_t;

// Explores to the goal, then the cells that could still shorten the speed run, returns exploring, faces north, saves.
run_result_t search_explore(void);
// Speed run over verified passages, back to the start and save; refused if no verified path. Without curves it turns in place.
run_result_t search_fast_run(uint8_t curves);
// How the search moves forward (CONT OFF/ON, until reset): stopping in every cell, or straights decided cell by cell (default).
typedef enum { SEARCH_STOP_EACH, SEARCH_STRAIGHTS } search_mode_t;
void search_set_mode(search_mode_t mode);
search_mode_t search_mode(void);

// Left- or right-hand wall follower: past the goal it goes on until STOP, so it ends RUN_ABORTED.
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
