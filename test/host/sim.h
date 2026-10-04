#ifndef SIM_H
#define SIM_H

#include <stdint.h>
#include "maze.h"
#include "motion.h"

// Simulated robot in a known "true" maze, implementing motion.h. Also built
// into the virtual robot firmware (src/test_virtual.c, SIM_ON_ROBOT), which
// replaces motion_checkpoint() and motion_indicate() with the robot's and
// takes each move's time for real in virtual_elapse() (turning its wheels).

#define SIM_POPS_BUCKETS      16
#define SIM_POPS_BUCKET_SIZE  1024u

typedef struct {
    uint32_t senses, quarter_turns, forward_moves, forward_cells;
    uint32_t paths, curves;     // motion_run_path() calls and the curves driven in them
    uint32_t blocked;       // drove at a wall on the first cell: robot backed up (safety net)
    uint32_t crashes;       // drove through a wall past the first cell: position lost
    uint32_t actions;
    uint32_t legs, stops;   // motion_explore() calls; moves that ended at rest (any kind)
    uint32_t wall_stops;    // search legs that stopped at a front wall seen on the way (expected)
    uint32_t decides, decide_pops, decide_pops_max;     // search legs' decisions on the way: planner work
    uint32_t decide_waits;  // ... and their waits for the UART (must stay 0: the robot is moving)
    uint32_t decide_hist[SIM_POPS_BUCKETS];             // ... how many took pops / SIM_POPS_BUCKET_SIZE
    double seconds;         // estimated robot time of every move and stop (see sim.c)
} sim_stats_t;

extern sim_stats_t sim_stats;
extern uint32_t sim_uart_waits;     // uart_wait_space() calls (host_io.c counts them)
extern uint8_t sim_x, sim_y;
extern heading_t sim_h;

// True maze: bit h of truth[x][y] = wall on side h.
void truth_reset(uint8_t with_all_walls);   // border always walls
void truth_set_wall(uint8_t x, uint8_t y, heading_t h, uint8_t on);
uint8_t truth_wall(uint8_t x, uint8_t y, heading_t h);
// Random perfect maze (DFS) with `extra_openings` walls knocked out to create
// loops. The start cell (0,0) keeps its east wall, as in competition mazes.
void truth_generate(uint32_t seed, uint16_t extra_openings);
// The centre 2x2 as in competition: open inside, one entrance (after truth_generate).
void truth_competition_goal(uint32_t seed);
// Loads the full truth into maze.c (for computing the true optimum).
void truth_load_into_map(void);

void sim_reset(double sensor_noise, uint32_t seed);   // robot at (0,0) facing north
void sim_abort_after(uint32_t actions);               // 0 = never
void sim_abort_after_cells(uint32_t cells);           // STOP at the first stop past that many cells
void sim_side_doubt(double probability);               // side readings reported doubtful
uint32_t sim_rand(void);
#ifdef SIM_ON_ROBOT
void virtual_elapse(double seconds, int16_t speed, int8_t turn);
#endif

#endif // SIM_H
