#ifndef MOTION_H
#define MOTION_H

#include <stdint.h>
#include "path.h"

// Robot actions for search.c, implemented by motion.c and by the host simulator (no HAL types here).

typedef enum {
    MOVE_OK = 0,
    MOVE_BLOCKED,   // obstacle right in front early in the move; backed up to the start point
    MOVE_ABORTED,   // STOP command or START button
    MOVE_TIMEOUT,
    MOVE_STALLED,   // wheels commanded but not turning
    MOVE_SLIPPED,   // left/right travel mismatch too large to trust the move
    MOVE_LOST,      // obstacle mid-move: position unknown
} move_result_t;

// A doubtful side reading (SIDE_YAW_DOUBT_MM: a likely phantom wall) is never recorded.
typedef enum { SEEN_ABSENT = 0, SEEN_PRESENT = 1, SEEN_DOUBTFUL = 2 } sighting_t;
typedef struct {
    uint8_t front, left, right;     // sighting_t (front never doubtful)
    uint8_t moving;                 // 1: the sides were read on the way in, during the straight that ended here
} wall_sense_t;

typedef enum { IND_GOAL, IND_DONE, IND_FAIL } indication_t;

// Drives `cells` cells straight at `cruise_speed` mm/s, stopping at the end or at the calibrated distance from a wall ahead.
move_result_t motion_forward(uint8_t cells, int16_t cruise_speed);
// Turns in place: -1 = 90 deg left, +1 = 90 deg right, 2 = 180 deg.
move_result_t motion_turn(int8_t quarter_turns);
// Drives the whole path without stopping (straights at `cruise_speed`, curves at `curve_speed`); `entered`: cells reached.
move_result_t motion_run_path(const run_path_t *path, int16_t cruise_speed, int16_t curve_speed, uint8_t *entered);
// ---- Search legs ----------------------------------------------------------------------
// From rest at a cell centre, straight on; `decide` is called inside each cell, its three walls in view. No curves.
typedef enum { NEXT_STRAIGHT, NEXT_STOP } next_move_t;
typedef next_move_t (*next_cell_fn)(const wall_sense_t *walls, void *ctx);
// `entered`: cells decided and reached; MOVE_BLOCKED: stopped at cell `entered`'s centre facing a wall the map had open.
move_result_t motion_explore(int16_t speed, next_cell_fn decide, void *ctx, uint8_t *entered);

// Majority-voted walls: the front stopped, the sides from the straight that just ended (`moving`).
move_result_t motion_sense_walls(wall_sense_t *out);
// Side readings exposed to phantom walls, from the front sensors' votes and readings (mm).
static inline void motion_doubt_sides(wall_sense_t *w, uint8_t fl_seen, uint8_t fr_seen,
                                      float fl_mm, float fr_mm, float square_offset,
                                      float yaw_doubt, float close_mm){
    uint8_t doubt_left = 0, doubt_right = 0;
    if(fl_seen && fr_seen){
        float skew = fl_mm - fr_mm - square_offset;     // > 0: rotated left, the right beam swings forward
        uint8_t too_close = (fl_mm + fr_mm) / 2.0f < close_mm;
        doubt_right = too_close || skew > yaw_doubt;
        doubt_left = too_close || skew < -yaw_doubt;
    }
    else{
        // Something ahead on one side only (yaw or a post): that side's beam may hit it too.
        doubt_left = fl_seen;
        doubt_right = fr_seen;
    }
    if(doubt_left && w->left == SEEN_PRESENT) w->left = SEEN_DOUBTFUL;
    if(doubt_right && w->right == SEEN_PRESENT) w->right = SEEN_DOUBTFUL;
}
// If a wall is in front, nudges to the calibrated distance from it.
void motion_align_front(void);
// Between actions: commands, pause and step mode. 0 when the run must stop.
uint8_t motion_checkpoint(void);
void motion_indicate(indication_t what);

static inline const char *move_result_name(move_result_t r){
    switch(r){
        case MOVE_OK:      return "OK";
        case MOVE_BLOCKED: return "BLOCKED";
        case MOVE_ABORTED: return "ABORTED";
        case MOVE_TIMEOUT: return "TIMEOUT";
        case MOVE_STALLED: return "STALLED";
        case MOVE_SLIPPED: return "SLIPPED";
        case MOVE_LOST:    return "LOST";
    }
    return "?";
}

// ---- Robot only (not used by search.c) ---------------------------------------
void motion_tick_1ms(void);             // from SysTick: speed control and centring
// Where the profiles are now and which move they belong to (SysTick: CAL recordings).
void motion_reference(float *fwd_mm, float *rot_deg, uint8_t *move_id);
void motion_tune_list(void);                        // TUNE: control constants that can change live
void motion_curve_info(uint8_t line);               // CAL dumps: the curve geometry and trims, '@D INFO' lines 0-2
void motion_tune_set(const char *name, float value);
void motion_stop(void);
uint8_t motion_wait(uint32_t ms);       // keeps polling inputs; 0 if aborted meanwhile
// Straight `mm` at `speed` mm/s (negative = backwards), without centring.
move_result_t motion_drive_straight(int16_t speed, int32_t mm);
void motion_request_abort(void);
void motion_clear_abort(void);
uint8_t motion_abort_requested(void);
void motion_set_paused(uint8_t paused);
void motion_set_step_mode(uint8_t on);
uint8_t motion_step_mode(void);
// The search legs' decisions on the way: worst planner time and pops, and how many came late.
void motion_leg_timing_reset(void);
float motion_ticks_per_mm(void);        // WHEEL_TICKS_PER_MM, or TUNE TICKS_MM
void motion_leg_timing_report(void);

#endif // MOTION_H
