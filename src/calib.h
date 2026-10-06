#ifndef CALIB_H
#define CALIB_H

#include <stdint.h>
#include "robot_config.h"

// Calibration experiments (CAL): SysTick samples encoders, PWM and raw IR during a known motion; dumped as '@D' lines.

typedef enum {
    CAL_NOISE,      // robot still: sensor noise (no motion)
    CAL_STRAIGHT,   // a normal N-cell move: distance, stop, steering
    CAL_TURN,       // N quarter turns: overshoot and settling
    CAL_CURVE,      // a cell, a smooth curve, a cell (speed run): tracking and where it ends
    CAL_STEP,       // open-loop PWM step then coast: motor model, braking
    CAL_IR,         // back away from a front wall: IR curve vs distance
    CAL_DUMP,       // send the last recording again
    CAL_RUN,        // arm: record the next continuous move of a run (speed run to the goal, search leg)
} cal_test_t;

#if DEV_TOOLS
void calib_tick_1ms(void);              // from SysTick
uint8_t calib_moves(cal_test_t test);   // 1 if the robot will move
// Runs `test` with its (validated) arguments and dumps the samples.
void calib_run(cal_test_t test, int32_t a, int32_t b);
// CAL RUN: motion.c marks a continuous move's start and end; main.c dumps it after the run.
void calib_path_start(void);
void calib_path_end(const char *result);
void calib_run_finished(void);
#else
static inline void calib_tick_1ms(void){}
static inline void calib_path_start(void){}
static inline void calib_path_end(const char *result){ (void)result; }
static inline void calib_run_finished(void){}
#endif

#endif // CALIB_H
