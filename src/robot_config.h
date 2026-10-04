#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

// Compile-time configuration, one line per constant: value, unit and why.
// The measurements behind them are in docs/control.md. Values tunable live
// (params.h, TUNE) only have their defaults here.

// ---- Maze ------------------------------------------------------------------
#define START_X                 0
#define START_Y                 0

// Default goal (inclusive cell rectangle): PRACTICE_MAZE comes from
// platformio.ini; the GOAL command overrides it at runtime.
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
#define WHEEL_TICKS_PER_MM      9.05f   // encoder ticks per mm travelled (ruler, 1-3 cell straights)
#define WHEEL_DIFF              (-0.0025f)  // TUNE WHEEL_DIFF: left wheel travel per tick / right - 1 (drift)
#define TICKS_PER_TURN          403     // TURNTICKS: half the wheel difference of a real 90 deg in-place turn

// ---- IR sensors ----------------------------------------------------------------
#define WALL_DETECT_MM          140     // closer than this = wall present
#define IR_DELAY_MS             50      // the IR report where the robot was this long ago
#define FRONT_TRACK_MM          170     // the front wall ending a straight is tracked from this reading on
#define FRONT_TRACK_REF_MM      92.0f   // where that tracking aims: the readings settle after the stop (94 ended long)
#define SIDE_WALL_TRACK_MM      130     // a side wall is used as steering reference only below this
#define SIDE_CENTER_L_MM        89.0f   // SL reading with the robot on the centre line (reads long)
#define SIDE_CENTER_R_MM        76.0f   // SR reading with the robot on the centre line (reads short)
#define FRONT_WALL_REF_MM       94      // front IR average when centered in a cell facing a wall
#define FRONT_EMERGENCY_MM      60      // both front sensors this close before the final approach = unexpected obstacle
#define FRONT_IR_MAX_DIFF_MM    30      // FL/FR disagreement beyond this: front not trusted for alignment
#define WALL_SAMPLES            5       // wall sensing: samples per sensor...
#define WALL_VOTES              4       // ...that must agree to call a wall
#define WALL_SAMPLE_MS          8       // spacing between samples
#define SIDE_PASS_MM            (CELL_MM / 2)   // sides read this far before the end of a move, not at the stop
// Search legs (CONT ON): each cell decided inside it with its three walls in view.
#define SEARCH_SIDE_FROM_MM     20.0f   // sides read from this far before the cell's entry edge...
#define SEARCH_SIDE_TO_MM       30.0f   // ...to this far into it (the angled beams hit ~70 mm ahead)
#define SEARCH_LATE_MARGIN_MM   8.0f    // decision due this far before the reference must brake for the cell's centre
#define SEARCH_LEG_SPEED_MAX    450     // mm/s: the fastest at which the front wall is known at the decision
#define SEARCH_MIN_READINGS     3       // side readings that must all agree, else the side is doubtful
#define SEARCH_FRONT_OPEN_MM    205.0f  // inside the cell at the decision: no wall in front above this
// Phantom side walls: stopped yawed or too far forward, a side beam (at the
// nose, 15 deg forward) hits the post or the front wall.
#define FRONT_SQUARE_OFFSET_MM  -15     // FL - FR square to a wall at the cell centre (median of 44 stops)
#define SIDE_YAW_DOUBT_MM       10      // |FL - FR - offset| beyond this (~8 deg): doubt the side it turns towards
#define SIDE_CLOSE_DOUBT_MM     25      // front wall this much closer than FRONT_WALL_REF_MM: doubt both sides

// ---- Speed control (control.c) ------------------------------------------------------
#define CONTROL_DT_S            0.001f
#define MOTOR_KV_L              1.036f  // feedforward (CAL STEP fits): PWM per mm/s, left wheel
#define MOTOR_KV_R              1.044f  // right wheel
#define MOTOR_TAU_S             0.053f  // s, motor time constant: PWM per mm/s^2 = KV * TAU
#define MOTOR_KS_PWM            25.0f   // PWM lost to friction while moving
#define FWD_KP                  40.0f   // PWM per mm of forward error (gains chosen in host_tests --control)
#define FWD_KD                  0.9f    // PWM per mm/s
#define ROT_KP                  40.0f   // PWM per deg of heading error (20 left the chassis stick-slipping in yaw)
#define ROT_KD                  0.8f    // PWM per deg/s (0.6 rang at ~7 Hz at 700 mm/s)
#define ROT_KI                  150.0f  // PWM per deg*s: learns the motors' imbalance
#define ROT_I_MAX               100.0f  // PWM
#define SETTLE_KI_FWD           3000.0f // PWM per mm*s once the profile arrived: beats the ~85 PWM of static friction
#define SETTLE_KI_ROT           1500.0f // PWM per deg*s
#define SETTLE_I_MAX            150.0f  // PWM
#define CONTROL_PWM_LIMIT       1000.0f
#define FWD_ERROR_MAX_MM        25.0f   // this far behind the reference: blocked or slipping -> MOVE_STALLED
#define PATH_LAG_FREE_MM        3.0f    // a path's reference slows down past this lag (normal: <= 2.7 mm at 900)...
#define PATH_LAG_SPAN_MM        6.0f    // ...reaching PATH_SCALE_MIN of its pace this much further on
#define PATH_SCALE_MIN          0.2f    // never stopped: a blocked robot still reaches FWD_ERROR_MAX_MM
#define ROT_ERROR_MAX_DEG       15.0f   // -> MOVE_SLIPPED
#define SETTLE_MM               0.5f    // a move ends once the errors are this small...
#define SETTLE_DEG              0.8f    // turns stall ~0.45-0.7 deg short (TURNTICKS absorbs it)
#define SETTLE_MAX_MS           200     // ...or after this long past the end of the profile
#define EMERGENCY_DECEL         6000.0f // mm/s^2: the short brake, for the obstacle distance

// ---- Wall centring (steer_step in control.c; KP, KI in params.h) ----------------------------
#define STEER_MAX_DEG           15.0f   // clamp of the heading offset (turns and hand placements leave 5-9 deg)
#define STEER_CURVE_DEG_PER_MM  0.2f    // how fast the heading offset may change: curves of radius >= 290 mm
#define STEER_AVERAGE_MS        32      // side readings averaged over two sensor periods
#define STEER_VREF_MM_S         500.0f  // above this KP falls as 1/speed (KP 0.7 weaved at 700)
#define STEER_SLEW_MM_PER_MS    0.5f    // lateral error change accepted per ms (posts, wall edges jump more)
#define STEER_ERROR_MAX_MM      25      // wall error clamp: beyond it the reading is a transient, not the robot
#define STEER_OBSERVER_MM       40.0f   // the bias observer follows the readings over this distance

// ---- Smooth curves (speed run, path.h) ------------------------------------------------
#define CURVE_RADIUS_MM         70.0f   // clothoid-arc-clothoid of this radius...
#define CURVE_RAMP_MM           30.0f   // ...and ramps: 85.5 mm along each axis, consecutive curves fit
#define CURVE_ANGLE_DEG         90.0f   // encoder degrees a slow curve asks for (TUNE CURVE_ANGLE)
#define CURVE_SLIP_DEG          2.0f    // ...plus this times (v / VREF)^2: faster curves slip and turn less
#define CURVE_SLIP_VREF_MM_S    480.0f
#define CURVE_PRE_ADJUST_MM     0.0f    // TUNE CURVE_PRE: mm more straight before a curve (CAL CURVE)
#define CURVE_POST_ADJUST_MM    0.0f    // TUNE CURVE_POST: mm more straight after it, to the exit edge
#define CURVE_PRE_SLIP_MM       7.0f    // mm more `pre` at VREF, as v^2 - V0^2: the sideways slip (layout E)
#define CURVE_PRE_V0_MM_S       300.0f  // ...from this speed up
#define CURVE_PWM_SHARE         0.9f    // PWM share the outer wheel may ask in a curve: caps it at ~480 mm/s

// ---- Other moves ------------------------------------------------------------------
#define SQUARE_MM_PER_DEG       1.2f    // FL - FR change per degree of yaw
#define SQUARE_TOL_MM           4       // |FL - FR - offset| tolerated (~3 deg)
#define SQUARE_MAX_SKEW_MM      15      // beyond this (~12 deg) not a flat wall ahead: no squaring
#define ALIGN_DEADBAND_MM       3       // distance to the front wall: no correction below this...
#define ALIGN_MAX_MM            30      // ...nor beyond this (unreliable)
#define ALIGN_SPEED             150     // mm/s of alignment and backing-up moves
#define MOVE_TIMEOUT_BASE_MS    5000    // move watchdog: base...
#define MOVE_TIMEOUT_PER_CELL_MS 2000   // ...plus per cell
#define START_DELAY_MS          5000    // countdown after START of a mode (hands away)
#define CAL_DELAY_MS            2000    // countdown of a CAL test (sent over Bluetooth)
#define COUNTDOWN_BLINK_MS      50      // every LED on/off this long during a countdown
// Race presets: selecting 2.4-2.6 sets FAST and CURVE (the console can still
// change them before START); 2.3, without curves, sets FAST_SAFE_SPEED.
#define FAST_SAFE_SPEED         800     // mm/s, race 2.4
#define FAST_SAFE_CURVE         300
#define FAST_MID_SPEED          900     // mm/s, race 2.5
#define FAST_MID_CURVE          400
#define FAST_FULL_SPEED         900     // mm/s, race 2.6
#define FAST_FULL_CURVE         480
#define BUTTON_DEBOUNCE_MS      20

// ---- Flash store (flash_store.c) ----------------------------------------------------------
#define FLASH_SETTLE_MS         1000    // a write waits until the motors are off this long (docs/freezes.md)

// ---- Planner / strategy ------------------------------------------------------------
// Costs per cell and per 90 deg turn (host_tests --costs): a turn is half a cell.
#define SEARCH_COST_CELL        2
#define SEARCH_COST_TURN        1
#define FAST_COST_CELL          2
#define FAST_COST_TURN          1
#define OPTIMIZE_MAX_STEPS      800     // actions after the goal to verify the best path (docs/competition.md)
#define SEARCH_MAX_STEPS        2000    // hard budget of actions per run
#define MAP_MAX_RECOVERIES      3       // "goal unreachable" map repairs allowed per run

// ---- Runtime parameter defaults (see params.h) ----------------------------------------
#define PARAM_SEARCH_SPEED      600     // mm/s (the search legs cap it at SEARCH_LEG_SPEED_MAX)
#define PARAM_FAST_SPEED        900     // mm/s, never below SPD (the motor model holds ~937)
#define PARAM_CURVE_SPEED       480     // mm/s, capped by FAST, the braking room and CURVE_PWM_SHARE
#define PARAM_ACCEL             3000    // mm/s^2 of every straight, up and down (5000 slipped)
#define PARAM_TURN_SPEED        500     // deg/s peak of in-place turns
#define PARAM_TURN_ACCEL        5000    // deg/s^2: a 90 deg turn takes ~0.3 s
#define PARAM_TURN_TICKS        TICKS_PER_TURN
#define PARAM_KP                0.5f    // deg of heading per mm off-centre (0.7 weaved with KI 20)
#define PARAM_KI                20.0f   // deg per mm off-centre per m travelled: how fast the observer learns a yaw
#define PARAM_LOG_LEVEL         2
#define PARAM_TELEMETRY         1       // '@' lines for tools/robot_monitor.py

#endif // ROBOT_CONFIG_H
