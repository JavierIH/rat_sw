#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdint.h>
#include "maze.h"

// '@' lines for tools/robot_monitor.py, sent only between actions (format: docs/design.md, telemetry).

typedef enum {
    TM_IDLE      = 'I',
    TM_COUNTDOWN = 'C',
    TM_TO_GOAL   = 'G',
    TM_OPTIMIZE  = 'O',
    TM_TO_START  = 'H',
    TM_FAST      = 'F',
    TM_RETURN    = 'R',
    TM_FOLLOW    = 'W',
    TM_ERASE     = 'E',
    TM_CALIBRATE = 'K',
} telemetry_activity_t;

void telemetry_mode(uint8_t mode);
void telemetry_activity(telemetry_activity_t activity);
void telemetry_pose(uint8_t x, uint8_t y, heading_t h);
void telemetry_cell(uint8_t x, uint8_t y, heading_t h);
void telemetry_background_row(void);
// Every map row in use, waiting for UART space (robot stopped): after a map repair.
void telemetry_map(void);
// Goal, mode, activity, pose and every map row; waits for UART space (robot stopped only).
void telemetry_sync(uint8_t mode, telemetry_activity_t activity, uint8_t x, uint8_t y, heading_t h);

#endif // TELEMETRY_H
