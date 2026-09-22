#ifndef CALIB_H
#define CALIB_H

#include <stdint.h>

// Calibration experiments (CAL command). A known motion runs while SysTick
// samples encoders, requested PWM and raw IR at a fixed period; then the
// samples are dumped as '@D' lines together with every relevant constant.
// tools/robot_monitor.py saves them as CSV in tools/calib_data/ and
// tools/calib_analyze.py summarises them.

typedef enum {
    CAL_NOISE,      // robot still: sensor noise (no motion)
    CAL_STRAIGHT,   // a normal N-cell move: distance, stop, steering
    CAL_TURN,       // N quarter turns: overshoot and settling
    CAL_STEP,       // open-loop PWM step then coast: motor model, braking
    CAL_IR,         // back away from a front wall: IR curve vs distance
    CAL_DUMP,       // send the last recording again
} cal_test_t;

void calib_tick_1ms(void);              // from SysTick
uint8_t calib_moves(cal_test_t test);   // 1 if the robot will move
// Runs `test` with its (validated) arguments and dumps the samples.
void calib_run(cal_test_t test, int32_t a, int32_t b);

#endif // CALIB_H
