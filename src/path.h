#ifndef PATH_H
#define PATH_H

#include <stdint.h>
#include "control.h"

// Continuous runs with smooth curves, pure C (no HAL). A speed run follows a
// whole verified path without stopping: straights along the centre line of
// the cells and a 90 deg curve inside every cell where the path turns,
// instead of braking to the cell centre and turning in place. SysTick steps
// it every millisecond in place of the plain profiles and feeds the same
// position loops (control.h); the host tests run the same code.
//
// Geometry (axle centre): a curve enters its cell on the centre line through
// one edge and leaves on the centre line through the side edge. It runs
// `pre` mm straight, the curve, then `post` mm straight to the exit edge.
// The curve is clothoid-arc-clothoid: the curvature ramps linearly from 0 to
// 1/radius over `ramp` mm, holds, and ramps back to 0, so the robot's
// angular speed never jumps (the wheels could not follow a step). The
// heading is a function of the distance travelled, not of time, so the
// shape is the same whatever the speed does in the curve.

#define PATH_MAX_CELLS 255

// Cells to drive into, in order, with the curve made in each: turn[i] is 0
// to go straight through the i-th cell entered, +1 to curve right in it, -1
// left. The robot starts at the centre of its cell facing the first one and
// stops at the centre of the last one, which never curves. The caller owns
// the array, which must outlive the run; NULL = straight through them all.
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
} curve_t;

// Derives the curve from its shape. The adjustments move the start later
// (pre) and the exit edge farther (post), for what the real robot does.
// Returns 0 if the shape is impossible or does not fit its cell (curves in
// consecutive cells would overlap).
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
    // The straight the reference is on or approaching: its first cell (the
    // one after the start or after the previous curve) and that cell's entry
    // edge, then the next curve.
    uint8_t first;
    float first_edge;
    uint8_t next;           // cell of the next curve; path->cells if none left
    int8_t dir;             // its direction
    float curve_start;      // mm where it starts (a huge value if none)
    float last_curve_end;   // mm where the previous curve ended (0 if none yet)
    uint8_t curves;         // curves finished
    uint8_t hold;           // brake to a stop on the path and wait there (PAUSE)
    uint8_t done;
    // Input before every step: mm the robot is behind the reference. Past
    // PATH_LAG_FREE_MM the reference runs slower (time scale < 1) so it never
    // runs away from a robot that cannot keep up: a curve's heading follows
    // the reference's distance, and a robot far behind would turn early.
    float lag;
    float scale, scale_min; // time scale of the last step, and the lowest one of the run
    float w;                // reference angular speed before the time scale, deg/s
} path_run_t;

// v_curve is lowered if the shortest straight after a curve (to the centre
// of the next cell) could not brake from it at `accel`. Returns 0 if the
// path is malformed.
uint8_t path_start(path_run_t *r, const run_path_t *path, const curve_t *curve, float cell_mm,
                   float v_straight, float v_curve, float accel);
// One control period: advances the reference and writes it into the two
// profiles control_step() follows (pos, delta, speed, accel, active).
void path_step(path_run_t *r, profile_t *fwd, profile_t *rot, float dt);
// Paths decided on the way (the search): the owner of the turn array has
// written turn[cells - 1], the curve (or 0) in what was the last cell, and
// turn[cells] = 0 (a straight path, turn NULL, needs none of that); the path
// now runs one cell further. Refused (0) once the
// reference has arrived, when the end was moved (a wall, a short stop), when
// full, or if the curve would already have started. Not to be interleaved
// with path_step() (on the robot: with SysTick masked).
uint8_t path_grow(path_run_t *r);

// Queries for the supervision (main context, between steps).
// 1 if `s` (at most the reference's position) lies on the current straight:
// past the last curve and before the next one.
uint8_t path_on_straight(const path_run_t *r, float s);
// Centre of the cell the current straight leads into: the next curve's
// cell (on the entry line), or the last cell.
float path_straight_end(const path_run_t *r);
// A cell centre on the current straight (the start cell's too, if the
// straight starts there): the one nearest to `s`, or the last at or before
// it. Writes how many cells the robot will have entered there (0 = still
// the start cell) and where the centre is. 0 if there is no such centre.
uint8_t path_straight_centre(const path_run_t *r, float s, uint8_t nearest, uint8_t *entered, float *centre);

#endif // PATH_H
