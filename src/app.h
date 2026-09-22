#ifndef APP_H
#define APP_H

#include <stdint.h>
#include "calib.h"

// Run modes: SELECT cycles them (LED n lit = mode n), START launches one.
typedef enum {
    MODE_SEARCH = 1,    // explore, optimise the speed-run path, return, save
    MODE_FAST,          // speed run on the known map, return, save
    MODE_FOLLOW_LEFT,   // left-hand wall follower
    MODE_FOLLOW_RIGHT,  // right-hand wall follower
    MODE_SENSORS,       // live IR readings, walls on the LEDs; motors stay off
    MODE_ERASE,         // erase the saved map (confirm with START)
} app_mode_t;

#define MODE_COUNT 6

uint8_t app_run_active(void);
uint8_t app_mode(void);
uint8_t app_set_mode(uint8_t mode);     // 0 if out of range
const char *app_mode_name(uint8_t mode);
void app_request_start(void);
void app_telemetry_sync(void);          // full map/state for the monitor (robot stopped)
void app_request_cal(cal_test_t test, int32_t a, int32_t b);    // run from the main loop

#endif // APP_H
