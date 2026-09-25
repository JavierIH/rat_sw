#ifndef SIM_H
#define SIM_H

#include <stdint.h>
#include "maze.h"
#include "motion.h"

// Simulated robot in a known "true" maze, implementing motion.h.

typedef struct {
    uint32_t senses, quarter_turns, forward_moves, forward_cells;
    uint32_t paths, curves;     // motion_run_path() calls and the curves driven in them
    uint32_t blocked;       // drove at a wall on the first cell: robot backed up (safety net)
    uint32_t crashes;       // drove through a wall past the first cell: position lost
    uint32_t actions;
    uint32_t legs, stops;   // motion_explore() calls; moves that ended at rest (any kind)
    uint32_t wall_stops;    // search legs that stopped at a front wall seen on the way (expected)
    double seconds;         // estimated robot time of every move and stop (see sim.c)
} sim_stats_t;

extern sim_stats_t sim_stats;
extern uint8_t sim_x, sim_y;
extern heading_t sim_h;

// True maze: bit h of truth[x][y] = wall on side h.
void truth_reset(uint8_t with_all_walls);   // border always walls
void truth_set_wall(uint8_t x, uint8_t y, heading_t h, uint8_t on);
uint8_t truth_wall(uint8_t x, uint8_t y, heading_t h);
// Random perfect maze (DFS) with `extra_openings` walls knocked out to create
// loops. The start cell (0,0) keeps its east wall, as in competition mazes.
void truth_generate(uint32_t seed, uint16_t extra_openings);
// Loads the full truth into maze.c (for computing the true optimum).
void truth_load_into_map(void);

void sim_reset(double sensor_noise, uint32_t seed);   // robot at (0,0) facing north
void sim_abort_after(uint32_t actions);               // 0 = never
void sim_side_doubt(double probability);               // side readings reported doubtful
uint32_t sim_rand(void);

#endif // SIM_H
