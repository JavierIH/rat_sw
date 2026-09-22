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

typedef struct { uint8_t front, left, right; } wall_sense_t;

typedef enum { IND_GOAL, IND_DONE, IND_FAIL } indication_t;

// Drives `cells` cells straight. Merged straights cruise at `cruise_speed`
// and brake to the search speed for the last cell, so every stop happens at
// the speed the cell length was calibrated at. Stops early on a front wall.
move_result_t motion_forward(uint8_t cells, int16_t cruise_speed);
// Turns in place: -1 = 90 deg left, +1 = 90 deg right, 2 = 180 deg.
move_result_t motion_turn(int8_t quarter_turns);
// Majority-voted wall readings with the robot stopped.
move_result_t motion_sense_walls(wall_sense_t *out);
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
void motion_request_abort(void);
void motion_clear_abort(void);
uint8_t motion_abort_requested(void);
void motion_set_paused(uint8_t paused);
uint8_t motion_is_paused(void);
void motion_set_step_mode(uint8_t on);
uint8_t motion_step_mode(void);

#endif // MOTION_H
