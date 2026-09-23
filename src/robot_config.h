#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

// Compile-time configuration: everything calibrated on the real robot lives
// here, with its unit. Speeds and gains that can be tuned live over Bluetooth
// only have their *defaults* here (PARAM_*); the live values are in params.c.

// ---- Maze ------------------------------------------------------------------
#define START_X                 0
#define START_Y                 0

// Default goal (inclusive cell rectangle). PRACTICE_MAZE is set per
// environment in platformio.ini; the GOAL command overrides it at runtime.
#if PRACTICE_MAZE
#define GOAL_X0                 3   // 4x3 practice maze, single goal cell
#define GOAL_Y0                 2
#define GOAL_X1                 3
#define GOAL_Y1                 2
#else
#define GOAL_X0                 7   // competition 16x16: center 2x2 block
#define GOAL_Y0                 7
#define GOAL_X1                 8
#define GOAL_Y1                 8
#endif

// ---- Geometry / odometry -----------------------------------------------------
#define CELL_MM                 180
#define LANE_WIDTH_MM           168.0f  // free space between the two walls of a corridor
#define TICKS_PER_MM            9       // measured: ~1424 ticks / 158 mm
#define CELL_TICKS              (CELL_MM * TICKS_PER_MM)    // 1620, geometric
// Single-cell moves were measured 16-20 mm short at 1600 ticks and retuned to
// 1760 = CELL_TICKS + 140. That shortfall is treated as a per-move effect
// (start-up slip / stop point), so a merged N-cell straight adds it once.
// If long straights end consistently long/short, tune CELL_TICKS vs this.
#define MOVE_EXTRA_TICKS        140
#define TICKS_FOR_CELLS(n)      ((int32_t)(n) * CELL_TICKS + MOVE_EXTRA_TICKS)
// Default of TURNTICKS: half the wheel difference at which a 90 deg in-place
// turn brakes (~5 ticks per degree). 490 landed at 100-105 deg; at 430 the
// wheels stopped at 438 +- 1.4 and turns were still slightly over 90.
#define TICKS_PER_TURN          422

// ---- IR sensors ----------------------------------------------------------------
#define WALL_DETECT_MM          140     // closer than this = wall present
#define SIDE_WALL_TRACK_MM      130     // a side wall is used as steering reference only below this
#define FRONT_WALL_REF_MM       94      // front IR average when centered in a cell facing a wall
#define FRONT_STOP_LEAD_MM      11      // IR stop fires this much early: measured coast after it (was ending 10.8 mm too close)
#define FRONT_EMERGENCY_MM      60      // both front sensors this close before the final approach = unexpected obstacle
#define FRONT_IR_MAX_DIFF_MM    30      // FL/FR disagreement beyond this: front not trusted for alignment
#define WALL_SAMPLES            5       // wall sensing: samples per sensor...
#define WALL_VOTES              4       // ...that must agree to call a wall
#define WALL_SAMPLE_MS          8       // spacing between samples
#define SENSE_SETTLE_MS         30      // let the chassis stop rocking before sensing
// The side sensors sit at the nose, angled 15 deg forward. Stopped too far
// forward or yawed, their beam leaves the cell next to the post and hits the
// post or the front wall: a phantom side wall. With something in front, the
// front sensors reveal that pose, and a "wall" reading on the exposed side is
// then treated as doubtful (not recorded) instead of trusted.
#define FRONT_SQUARE_OFFSET_MM  0       // FL - FR when square to a wall at the cell centre (calibrate: CAL NOISE)
#define SIDE_YAW_DOUBT_MM       20      // |FL - FR - offset| beyond this: yawed, doubt the side it turns towards
#define SIDE_CLOSE_DOUBT_MM     25      // front wall this much closer than FRONT_WALL_REF_MM: doubt both sides

// ---- Motion ----------------------------------------------------------------------
#define PD_STRAIGHT_MAX         150     // clamp of the steering correction (PWM)
#define ACCEL_STEP_PER_MS       4       // max PWM change per ms (start ramp)
#define FRONT_STOP_ZONE_TICKS   (TICKS_FOR_CELLS(1) / 4)   // front-wall stop only armed in the last quarter cell
#define FRONT_STOP_CONFIRM_MS   3       // consecutive 1 ms readings needed for an IR stop
#define ENCODER_MAX_DIFF_TICKS  300     // L/R travel mismatch that invalidates a 1-cell move...
#define ENCODER_MAX_DIFF_PER_CELL 100   // ...plus this per extra cell of a merged straight
#define FAST_APPROACH_TICKS     (CELL_TICKS / 2)   // straights end at search speed for this long...
#define FAST_DECEL_TICKS        CELL_TICKS         // ...after braking linearly from cruise over this
#define DRIFT_CORRECT_MAX_MM    30      // front-wall alignment ignored beyond this error (unreliable)
#define ALIGN_DEADBAND_MM       5       // ...and skipped below this one (not worth a stop-and-go)
#define DRIFT_CORRECT_SPEED     105     // PWM for alignment nudges and backing up (80 needed the breakaway boost every time)
#define DRIFT_CORRECT_TIMEOUT_MS 1000
#define TURN_STILL_MS           20      // after a turn, wait until the wheels are this long still...
#define TURN_SETTLE_MS          150     // ...but no longer than this (it used to be a fixed 150 ms)
#define MOVE_TIMEOUT_BASE_MS    5000    // move watchdog: base...
#define MOVE_TIMEOUT_PER_CELL_MS 2000   // ...plus per cell
// Static friction: wheels still this long into a move get extra duty, 1 PWM
// every BREAKAWAY_MS_PER_PWM ms up to BREAKAWAY_MAX_PWM, dropped as soon as
// they turn (in-place turns at low PWM need it: the wheels scrub sideways).
#define BREAKAWAY_DELAY_MS      120
#define BREAKAWAY_MS_PER_PWM    4
#define BREAKAWAY_MAX_PWM       150
#define STALL_TIMEOUT_MS        800     // still not turning after that (full boost included) = stalled
#define STALL_MIN_TICKS         20      // travel that counts as progress for the stall detector
#define START_DELAY_MS          2000    // countdown after START (hands away)
#define BUTTON_DEBOUNCE_MS      20

// ---- Planner / strategy ------------------------------------------------------------
// Costs in arbitrary units per cell and per 90 deg turn. Search moves stop at
// every cell, so a turn costs about half a cell; speed runs merge straights,
// so there a turn (brake, turn, settle, accelerate) costs about two cells.
#define SEARCH_COST_CELL        2
#define SEARCH_COST_TURN        1
#define FAST_COST_CELL          2
#define FAST_COST_TURN          4
#define OPTIMIZE_MAX_STEPS      300     // extra exploration after the goal, looking for a better path
#define SEARCH_MAX_STEPS        2000    // hard budget of actions per run
#define MAP_MAX_RECOVERIES      3       // "goal unreachable" map repairs allowed per run

// ---- Runtime parameter defaults (see params.h) ----------------------------------------
#define PARAM_SEARCH_SPEED      150     // PWM 0-1000
#define PARAM_FAST_SPEED        220
#define PARAM_TURN_SPEED        110
#define PARAM_TURN_TICKS        TICKS_PER_TURN
#define PARAM_KP                2.0f
#define PARAM_KD                30.0f
#define PARAM_KE                0.0f    // encoder heading hold without side walls: off until tuned
#define PARAM_LOG_LEVEL         2
#define PARAM_TELEMETRY         1       // '@' lines for tools/robot_monitor.py

#endif // ROBOT_CONFIG_H
