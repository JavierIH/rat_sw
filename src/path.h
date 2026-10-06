#ifndef PATH_H
#define PATH_H

#include <stdint.h>
#include "control.h"

// Continuous runs (pure C): straights on the centre line, a clothoid-arc-clothoid curve in each turning cell.

#define PATH_MAX_CELLS 255

// turn[i]: 0 straight through the i-th cell entered, +1 curve right, -1 left; the last never curves. NULL: all straight.
typedef struct {
    const int8_t *turn;
    uint8_t cells;
} run_path_t;

typedef struct {
    float radius;       // mm, of the circular part
    float ramp;         // mm, each clothoid
    float angle;        // deg the encoders must see for a real 90 (like TURNTICKS for in-place turns)
    float length;       // mm along the curve
    float footprint;    // mm the curve advances along each axis
    float pre, post;    // mm straight inside the cell before and after the curve (geometry + tuning)
    float k, k_ramp;    // 1/radius (per mm) and its rate along a ramp (per mm^2): no divisions in SysTick
    float slip_k;       // deg added to `angle` per (mm/s)^2 of the path's curve speed (0 after curve_setup)
    float pre_k;        // mm added to `pre` per (mm/s)^2 of the path's curve speed above...
    float pre_v0;       // ...this, mm/s (0 after curve_setup)
} curve_t;

// The curve from its shape and adjustments (later start, farther exit); 0 if impossible or it does not fit its cell.
uint8_t curve_setup(curve_t *c, float radius, float ramp, float angle, float pre_adjust, float post_adjust,
                    float cell_mm);
// Heading into the curve, 0..1 of its turn, `u` mm after its start.
float curve_progress(const curve_t *c, float u);

typedef struct {
    run_path_t path;
    curve_t curve;
    float cell_mm;
    float v_straight, v_curve, accel;   // mm/s, mm/s, mm/s^2
    float length;           // mm, start to end as planned
    float stop_at;          // mm, where the run ends: moves with a wall seen at the end, or earlier to stop short
    float s, v;             // reference: distance along the path, speed
    float heading;          // reference heading, deg (> 0 right), turns included
    float base;             // heading before the next curve
    // The straight the reference is on or approaching: its first cell and entry edge, then the next curve.
    uint8_t first;
    float first_edge;
    uint8_t next;           // cell of the next curve; path->cells if none left
    int8_t dir;             // its direction
    float curve_start;      // mm where it starts (a huge value if none)
    float last_curve_end;   // mm where the previous curve ended (0 if none yet)
    uint8_t curves;         // curves finished
    uint8_t hold;           // brake to a stop on the path and wait there (PAUSE)
    uint8_t done;
    // Input: mm the robot is behind; past PATH_LAG_FREE_MM the reference slows (scale < 1) so it never runs away.
    float lag;
    float scale, scale_min; // time scale of the last step, and the lowest one of the run
    float w;                // reference angular speed before the time scale, deg/s
} path_run_t;

// v_curve is lowered if the last straight cannot brake from it; slip_k and pre_k grow the curves with speed. 0 if malformed.
uint8_t path_start(path_run_t *r, const run_path_t *path, const curve_t *curve, float cell_mm,
                   float v_straight, float v_curve, float accel);
// One period: advances the reference into the two profiles control_step() follows.
void path_step(path_run_t *r, profile_t *fwd, profile_t *rot, float dt);
// A path decided on the way grows one cell (turn[cells - 1] written by the caller); 0 once arrived, moved, full or curving.
uint8_t path_grow(path_run_t *r);

// Supervision queries (main context). 1 if `s` lies on the current straight.
uint8_t path_on_straight(const path_run_t *r, float s);
// Centre of the cell the current straight leads into (the next curve's, or the last).
float path_straight_end(const path_run_t *r);
// The cell centre on the current straight nearest `s` (or the last before it): cells entered there and where; 0 if none.
uint8_t path_straight_centre(const path_run_t *r, float s, uint8_t nearest, uint8_t *entered, float *centre);

#endif // PATH_H
