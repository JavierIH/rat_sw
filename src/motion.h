#ifndef MOTION_H
#define MOTION_H

#include <stdint.h>

// Robot actions used by the strategies in search.c. Implemented by motion.c
// on the robot and by the simulator in test/host, so this header stays free
// of HAL types.

typedef enum {
    MOVE_OK = 0,
    MOVE_BLOCKED,   // obstacle right in front early in the move; backed up to the start point
    MOVE_ABORTED,   // STOP command or START button
    MOVE_TIMEOUT,
    MOVE_STALLED,   // wheels commanded but not turning
    MOVE_SLIPPED,   // left/right travel mismatch too large to trust the move
    MOVE_LOST,      // obstacle mid-move: position unknown
} move_result_t;

// A side reading can also be doubtful: the pose made a phantom wall likely
// (see SIDE_YAW_DOUBT_MM). Doubtful readings must not be recorded.
typedef enum { SEEN_ABSENT = 0, SEEN_PRESENT = 1, SEEN_DOUBTFUL = 2 } sighting_t;
typedef struct { uint8_t front, left, right; } wall_sense_t;   // sighting_t (front never doubtful)

typedef enum { IND_GOAL, IND_DONE, IND_FAIL } indication_t;

// Drives `cells` cells straight. Merged straights cruise at `cruise_speed`
// and brake to the search speed for the last cell, so every stop happens at
// the speed the cell length was calibrated at. Stops early on a front wall.
move_result_t motion_forward(uint8_t cells, int16_t cruise_speed);
// Turns in place: -1 = 90 deg left, +1 = 90 deg right, 2 = 180 deg.
move_result_t motion_turn(int8_t quarter_turns);
// Majority-voted wall readings with the robot stopped.
move_result_t motion_sense_walls(wall_sense_t *out);
// Side readings exposed to phantom walls, given the front sensor votes and
// their average readings (mm). Pure: shared with the host tests.
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
        // Something ahead on one side only (strong yaw or a post): the side
        // beam on that side can be hitting it too.
        doubt_left = fl_seen;
        doubt_right = fr_seen;
    }
    if(doubt_left && w->left == SEEN_PRESENT) w->left = SEEN_DOUBTFUL;
    if(doubt_right && w->right == SEEN_PRESENT) w->right = SEEN_DOUBTFUL;
}
// If a wall is in front, nudges to the calibrated distance from it.
void motion_align_front(void);
// Between actions: handles commands, pause and single-step mode. Returns 0
// when the run must stop.
uint8_t motion_checkpoint(void);
void motion_indicate(indication_t what);

static inline const char *move_result_name(move_result_t r){
    switch(r){
        case MOVE_OK:      return "OK";
        case MOVE_BLOCKED: return "BLOQUEADO";
        case MOVE_ABORTED: return "ABORTADO";
        case MOVE_TIMEOUT: return "TIMEOUT";
        case MOVE_STALLED: return "ATASCADO";
        case MOVE_SLIPPED: return "DESLIZAMIENTO";
        case MOVE_LOST:    return "PERDIDO";
    }
    return "?";
}

// ---- Robot only (not used by search.c) ---------------------------------------
void motion_tick_1ms(void);             // from SysTick: steering controller at 100 Hz
void motion_stop(void);
uint8_t motion_wait(uint32_t ms);       // keeps polling inputs; 0 if aborted meanwhile
// Both wheels at `pwm` (ramped, no steering) until the robot moved `ticks`.
move_result_t motion_drive_straight(int16_t pwm, int32_t ticks);
void motion_request_abort(void);
void motion_clear_abort(void);
uint8_t motion_abort_requested(void);
void motion_set_paused(uint8_t paused);
uint8_t motion_is_paused(void);
void motion_set_step_mode(uint8_t on);
uint8_t motion_step_mode(void);

#endif // MOTION_H
