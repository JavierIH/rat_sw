#ifndef APP_H
#define APP_H

#include <stdint.h>
#include "calib.h"
#include "storage.h"

// What a run does; the numbers go to the monitor (@M), which knew 1-6 before the menu: keep them.
typedef enum {
    MODE_SEARCH = 1,    // explore, optimise the speed-run path, return, save
    MODE_FOLLOW_LEFT,   // left-hand wall follower to the goal
    MODE_FOLLOW_RIGHT,  // right-hand wall follower to the goal
    MODE_FAST_SAFE,     // speed run at FAST_SAFE_* (set when selected), return, save
    MODE_FAST,          // speed run at FAST_FULL_* (set when selected), return, save
    MODE_ERASE,         // erase the saved map (confirm with START)
    MODE_NO_CURVES,     // speed run turning in place, straights at FAST_SAFE_SPEED, return, save
    MODE_FAST_MID,      // speed run at FAST_MID_* (set when selected), return, save
} app_mode_t;

#define MODE_COUNT 8

// SELECT cycles modes 1-3; START on mode 2 opens the race menu (SELECT cycles, START launches; only a reset leaves).
enum { MENU_SEARCH = 1, MENU_RACE, MENU_ERASE };
#define MENU_COUNT 3
#define RACE_COUNT 6

uint8_t app_run_active(void);
uint8_t app_mode(void);                 // app_mode_t
uint8_t app_set_mode(uint8_t menu, uint8_t race);   // race 0: the last one; 0 if out of range
const char *app_mode_label(void);       // "2.4 RAPIDA SEGURA"
void app_request_start(void);
void app_telemetry_sync(void);          // full map/state for the monitor (robot stopped)
void app_request_cal(cal_test_t test, int32_t a, int32_t b);    // run from the main loop
void app_request_check(void);           // CHECK (docs/lighting.md), run from the main loop
// SAVE, ERASE, mode 3 (at rest): storage_save(0), compacting first if full.
storage_save_t app_save_now(void);

#endif // APP_H
