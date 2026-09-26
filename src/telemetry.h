#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdint.h>
#include "maze.h"

// Machine-readable lines for tools/robot_monitor.py (live maze view). All
// start with '@', are at most 21 bytes and are only sent between actions,
// never from a control loop. TELEM OFF silences them.
//
//   @M<m>                      selected mode, 1-6
//   @A<a>                      activity (telemetry_activity_t)
//   @P<x><y><h>                pose: x, y one hex digit each, h one of NESW
//   @C<x><y><h><n><e><s><w>    pose after sensing + that cell's four walls:
//                              '#' wall, 'o' open (verified), '.' open (seen
//                              once), '?' unknown
//   @R<y><16 cells>            map row y, one char per cell from
//                              0-9A-V = bits 0-1 north wall, bits 2-3 east
//                              wall (0 unknown, 1 wall, 2 open once,
//                              3 open verified), bit 4 visited
//   @G<x0><y0><x1><y1>         goal rectangle
//   @Y<n>                      full sync follows: forget the map, rows 0..n
//
// While running, every sensing also sends one map row in rotation, so the
// monitor's copy converges even when the UART queue drops a line.

typedef enum {
    TM_IDLE      = 'I',
    TM_COUNTDOWN = 'C',
    TM_TO_GOAL   = 'G',
    TM_OPTIMIZE  = 'O',
    TM_TO_START  = 'H',
    TM_FAST      = 'F',
    TM_RETURN    = 'R',
    TM_SENSORS   = 'S',
    TM_ERASE     = 'E',
    TM_CALIBRATE = 'K',
} telemetry_activity_t;

void telemetry_mode(uint8_t mode);
void telemetry_activity(telemetry_activity_t activity);
void telemetry_pose(uint8_t x, uint8_t y, heading_t h);
void telemetry_cell(uint8_t x, uint8_t y, heading_t h);
void telemetry_background_row(void);
// Every map row in use, waiting for UART space (robot stopped): after a map
// repair, which may change walls anywhere.
void telemetry_map(void);
// Goal, mode, activity, pose and every map row in use. Waits for UART space:
// robot stopped only.
void telemetry_sync(uint8_t mode, telemetry_activity_t activity, uint8_t x, uint8_t y, heading_t h);

#endif // TELEMETRY_H
