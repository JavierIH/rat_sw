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
// Encoder ticks per mm of travel: 1-, 2- and 3-cell straights measured with a
// ruler (4885 ticks ~ 540 mm). Every distance the robot drives is exact to
// this: the speed control stops where it is told, with no coasting to model.
#define WHEEL_TICKS_PER_MM      9.05f
// Default of TURNTICKS: half the wheel difference (encoder ticks) of a real
// 90 deg in-place turn, i.e. the wheel track as the encoders see it (the
// wheels scrub). Calibrated with the robot facing a wall: CAL NOISE, CAL TURN
// 4, CAL NOISE, CAL TURN -4, CAL NOISE, comparing FL - FR (~1.2 mm per
// degree, SQUARE_MM_PER_DEG). With the speed control: 400 turned 355.3 deg
// per 360; 405 turns 360.8 right / 359.4 left (0.2 deg per turn), and the
// side readings do not move after 8 turns (truly in place).
#define TICKS_PER_TURN          405

// ---- IR sensors ----------------------------------------------------------------
#define WALL_DETECT_MM          140     // closer than this = wall present
// The IR sensors (Sharp-type, a new value every ~16 ms) report where the
// robot was ~50 ms earlier: at 400 mm/s the front wall read 18 mm farther
// than it was. Position + reading 50 ms old matched the wall within 0.6 mm
// over a whole approach (CAL STRAIGHT 3 400).
#define IR_DELAY_MS             50
#define IR_PERIOD_MS            16      // a new value this often, in ~2 mm steps at 80 mm (side sensors)
// The wall at the end of a straight is tracked from this reading on. Below
// 140 mm, at 500 mm/s and 50 ms late, the robot learnt about it with ~20 mm
// left: too late to brake if the wall was not where planned. Beyond 150 mm
// the readings err short (the wall looks closer): it brakes a bit early and
// the target recedes as the readings improve.
#define FRONT_TRACK_MM          170
// Where the tracking aims: the readings keep settling for a while after the
// stop, and aiming at FRONT_WALL_REF_MM itself ended 2-4 mm long (a front
// realignment in 5 of the stops of a search). At 92 the stops read 93-94.
#define FRONT_TRACK_REF_MM      92.0f
#define SIDE_WALL_TRACK_MM      130     // a side wall is used as steering reference only below this
// Side readings with the robot on the centre line. A 180 deg turn in place
// mirrors the robot across it, so the readings before and after give it
// regardless of where the robot stood: SR 87/65 and SL 76/101 (twice, alike).
// SR reads ~8 mm short and SL ~5 mm long of LANE_WIDTH_MM / 2: centring on
// 84 kept the robot ~8 mm towards the left wall. Hand-centred readings
// agree (SR 76-80, SL 84-89). TUNE CENTER_L / CENTER_R to try values.
#define SIDE_CENTER_L_MM        89.0f
#define SIDE_CENTER_R_MM        76.0f
#define FRONT_WALL_REF_MM       94      // front IR average when centered in a cell facing a wall
#define FRONT_EMERGENCY_MM      60      // both front sensors this close before the final approach = unexpected obstacle
#define FRONT_IR_MAX_DIFF_MM    30      // FL/FR disagreement beyond this: front not trusted for alignment
#define WALL_SAMPLES            5       // wall sensing: samples per sensor...
#define WALL_VOTES              4       // ...that must agree to call a wall
#define WALL_SAMPLE_MS          8       // spacing between samples
// Side walls are read on the way into a cell, this far before the end of the
// move: the angled side beams then hit the middle of its walls. At the stop
// they aim a couple of cm from the next post and caught it as phantom walls
// (9 in the first test logs, some at 82-98 mm, as close as real walls).
#define SIDE_PASS_MM            (CELL_MM / 2)
// Search legs (motion_explore, CONT ON): straight on without stopping, each
// cell decided inside it once its front wall is in plain view, so with the
// same walls the robot would see stopped there. Sides read from
// SEARCH_SIDE_FROM_MM before the cell's entry edge to SEARCH_SIDE_TO_MM into
// it: the angled beams hit ~70 mm ahead of the robot (at a stop, a couple of
// cm short of the next post), so from ~50 mm into the cell to ~100. From 60
// mm before the edge they still caught the post there, or the previous
// cell's wall: on the practice maze 1-cell legs left open sides and border
// walls doubtful. A side counts only if SEARCH_MIN_READINGS or more (one
// each WALL_SAMPLE_MS) all agree, else it is doubtful. The decision is
// due SEARCH_LATE_MARGIN_MM before the reference would start braking (at
// ACCEL) for the cell's centre, so a stop there is an ordinary one. The front
// wall must be known by then: it reads reliably under ~170 mm, which with the
// 50 ms IR delay happens ~14 mm + 50 ms of travel into the cell (a wall reads
// ~135 at the decision, none over SEARCH_FRONT_OPEN_MM). Up to ~500 mm/s that
// comes before the decision; SEARCH_LEG_SPEED_MAX leaves ~12 mm (25 ms) to
// spare. Faster, the robot would decide blind and brake harder at walls (at
// 600, ~4100 mm/s^2; the wheels slipped at 5000).
#define SEARCH_SIDE_FROM_MM     20.0f
#define SEARCH_SIDE_TO_MM       30.0f
#define SEARCH_LATE_MARGIN_MM   8.0f
#define SEARCH_LEG_SPEED_MAX    450
#define SEARCH_MIN_READINGS     3
#define SEARCH_FRONT_OPEN_MM    205.0f   // inside the cell at the decision: no wall in front above this
// Pause before sensing at a stop. The old stop-and-coast rocked the chassis
// (30 ms); the controlled stops do not: searches at 10 and 0 ms mapped the
// practice maze exactly, with no doubtful wall.
#define SENSE_SETTLE_MS         0
// The side sensors sit at the nose, angled 15 deg forward. Stopped too far
// forward or yawed, their beam leaves the cell next to the post and hits the
// post or the front wall: a phantom side wall. With something in front, the
// front sensors reveal that pose, and a "wall" reading on the exposed side is
// then treated as doubtful (not recorded) instead of trusted.
// FL - FR when square to a wall at the cell centre: median of 44 IR stops
// on the practice maze (sd 5.7 mm), confirmed with CAL NOISE (-15.4, -16.2).
#define FRONT_SQUARE_OFFSET_MM  -15
#define SIDE_YAW_DOUBT_MM       10      // |FL - FR - offset| beyond this: yawed (~8 deg), doubt the side it turns towards
#define SIDE_CLOSE_DOUBT_MM     25      // front wall this much closer than FRONT_WALL_REF_MM: doubt both sides

// ---- Speed control ------------------------------------------------------------------
// Every move follows a motion profile (control.h): the speed ramps up at
// ACCEL, cruises and ramps down to zero exactly at the target. SysTick runs
// it every ms together with two position loops, forward (mm) and rotation
// (deg), that make the wheels follow it, plus a feedforward from the motor
// model so the loops only correct what the model misses.
#define CONTROL_DT_S            0.001f
// Motor model per wheel (open-loop PWM steps, CAL STEP 200/400/600;
// calib_analyze.py fits it): PWM = KV * speed + KS, first order with time
// constant TAU (34-79 ms, shorter at higher PWM), no dead time. Both wheels
// measured alike: 393/393 mm/s at PWM 400. The battery changes KV; the loops
// absorb that.
#define MOTOR_KV_L              1.036f  // PWM per mm/s
#define MOTOR_KV_R              1.044f
#define MOTOR_TAU_S             0.053f  // s: PWM per mm/s^2 = KV * TAU
#define MOTOR_KS_PWM            25.0f   // PWM lost to friction while moving: 100 PWM ran at 73 mm/s, turns cruise 25 PWM above KV * v
// Loop gains, chosen on a simulated robot with this motor model (test/host):
// ~5 Hz bandwidth, damping ~0.7, tolerant of +-20 % in KV and TAU.
// Without the D terms it oscillated. The rotation integral learns the motor
// imbalance the feedforward misses.
#define FWD_KP                  40.0f   // PWM per mm of forward error
#define FWD_KD                  0.9f    // PWM per mm/s
// The chassis resists changes of heading (static friction in yaw, likely
// the skids): at ROT_KP 20 the heading stuck until 2-4 deg of error, then
// jumped (stick-slip, S-curves). 40/0.6 on the robot: heading oscillation
// 0.83 -> 0.35 deg rms, tracking error 3.9 -> 1.5 deg max. At 700 mm/s KD
// 0.6 rang at ~7 Hz; 0.8 did not (lateral noise +-2.0 -> +-0.8 mm).
#define ROT_KP                  40.0f   // PWM per deg of heading error (~0.5 mm of wheel travel)
#define ROT_KD                  0.8f    // PWM per deg/s
#define ROT_KI                  150.0f  // PWM per deg*s
#define ROT_I_MAX               100.0f  // PWM
// Once the profile has arrived the wheels stall ~1 deg / ~1 mm short: static
// friction (~85 PWM: 70 never moved the wheels, 100 did) beats what the
// loops push with such small errors. While settling, an integral that builds
// that push within ~50 ms finishes the move.
#define SETTLE_KI_FWD           3000.0f // PWM per mm*s
#define SETTLE_KI_ROT           1500.0f // PWM per deg*s
#define SETTLE_I_MAX            150.0f  // PWM
#define CONTROL_PWM_LIMIT       1000.0f
// Following error: far behind the reference means blocked (a wall, a post)
// or slipping. The move fails instead of pushing on.
#define FWD_ERROR_MAX_MM        25.0f   // -> MOVE_STALLED
// A path's reference slows down (path.c) while the robot is more than
// PATH_LAG_FREE_MM behind it (normal tracking: <= 2.7 mm at 900 mm/s),
// reaching PATH_SCALE_MIN of its pace PATH_LAG_SPAN_MM further on: never
// stopped, so a blocked robot still reaches FWD_ERROR_MAX_MM.
#define PATH_LAG_FREE_MM        3.0f
#define PATH_LAG_SPAN_MM        6.0f
#define PATH_SCALE_MIN          0.2f
#define ROT_ERROR_MAX_DEG       15.0f   // -> MOVE_SLIPPED
#define SETTLE_MM               0.5f    // a move ends once the errors are this small...
#define SETTLE_DEG              0.8f    // turns stall ~0.45-0.7 deg short (TURNTICKS absorbs it); 0.5 waited 200 ms more
#define SETTLE_MAX_MS           200     // ...or after this long past the end of the profile
#define EMERGENCY_DECEL         6000.0f // mm/s^2: the short brake, for the obstacle distance

// ---- Wall centring (steer_step in control.c) ----------------------------------------
// The heading offset is proportional to how far off-centre the robot is, so
// it converges over the same distance at any speed. KP and KI are runtime
// parameters (see params.h).
#define STEER_MAX_DEG           15.0f   // clamp of the heading offset: above the 5-9 deg a hand placement or turn leaves (8 could not even get parallel)
// The side IR step ~2.4 mm every ~16 ms and the real heading follows its
// target ~50 ms late: KP 1 with 0.35 deg/mm and a 16 ms average made fast
// small S-curves (~5.5 Hz, +-2 deg). Gentler settings, the simulator with a
// slower heading response agrees (3.8 -> 1.5 deg heading error).
#define STEER_CURVE_DEG_PER_MM  0.2f    // how fast the heading offset may change: curves of radius >= 290 mm, no pivoting at the start
#define STEER_AVERAGE_MS        32      // side readings averaged over two sensor periods
// Above this speed KP scales down as 1/speed. The heading loop takes the
// same time at any speed, so at speed it lags over more distance and the
// centring loses damping: at 700 mm/s KP 0.7 weaved (1.0 deg rms, 4 Hz),
// 0.5 did not (0.67 deg rms), same as KP 0.7 at 400.
#define STEER_VREF_MM_S         500.0f
#define STEER_SLEW_MM_PER_MS    0.5f    // lateral error change accepted per ms (posts, wall edges jump more)
#define STEER_ERROR_MAX_MM      25      // wall error clamp: beyond it the reading is a transient, not the robot
// KI learns only this close to the centre (no overshoot after big
// corrections). At 5 mm a turn that left the robot 5 deg off kept it 12 mm
// off-centre for a whole straight (KP 0.5 alone: 10 mm per 5 deg).
#define STEER_BIAS_WINDOW_MM    12.0f
// The bias (the encoder heading parallel to the walls) learned from how the
// readings move against what the encoders predict (steer_step()), following
// the readings over this distance. 0: the old integral of the lateral error
// (within STEER_BIAS_WINDOW_MM), which took an off-centre start for a heading
// error.
#define STEER_OBSERVER_MM       40.0f

// ---- Smooth curves (speed run, path.h) ------------------------------------------------
// The speed run turns without stopping: a 90 deg clothoid-arc-clothoid inside
// the cell, entering and leaving on the centre lines. Radius 70 with 30 mm
// ramps advances 85.5 mm along each axis, so 4.5 mm of the cell are
// straight before and after it: curves in consecutive cells (staircases)
// join without overlapping. At 400 mm/s: 2.3 m/s^2 sideways (braking at 5
// slipped, 3 was near the grip limit), 4400 deg/s^2 at the ramps (in-place
// turns use 5000), outer wheel at 560 mm/s. TUNE CURVE_* changes them live.
#define CURVE_RADIUS_MM         70.0f
#define CURVE_RAMP_MM           30.0f
// Encoder degrees per curve: 90 unless the robot turns more or less than
// the encoders say while moving (the wheels scrub less than in place).
#define CURVE_ANGLE_DEG         90.0f
// Straight before / after the curve beyond the geometric one: > 0 starts it
// later / reaches the exit edge later. The robot follows the reference a few
// ms late, so it may need to start earlier (exits displaced to the outside).
#define CURVE_PRE_ADJUST_MM     0.0f
#define CURVE_POST_ADJUST_MM    0.0f
// Share of CONTROL_PWM_LIMIT the curves may ask of the outer wheel (the
// loops need the rest): its feedforward peaks where a ramp meets the arc,
// fastest and still accelerating. It caps the curve speed at ~480 mm/s with
// the motor model above. In the simulator 500 still tracked within 1 mm, 600
// saturated for a moment and 700 fell 15-57 mm behind the reference.
#define CURVE_PWM_SHARE         0.9f

// ---- Other moves ------------------------------------------------------------------
// Front alignment after a straight that ends facing a wall: square first
// (rotate by the angle FL - FR says), then fix the distance.
#define SQUARE_MM_PER_DEG       1.2f    // FL - FR change per degree of yaw (TURNTICKS calibration: 1.07-1.16 mm per tick over 4 turns)
#define SQUARE_TOL_MM           4       // |FL - FR - offset| tolerated (~3 deg)
// Beyond this (12 deg) the readings are not a flat wall square ahead: leave
// it. Turns and hand placements leave up to ~9 deg (every squaring in the
// logs was under 15 mm). At 35, a speed run that arrived off-centre scraping
// the wall read 17 mm by the corner and turned 15 deg into it.
#define SQUARE_MAX_SKEW_MM      15
#define ALIGN_DEADBAND_MM       3       // distance to the front wall: no correction below this...
#define ALIGN_MAX_MM            30      // ...nor beyond this (unreliable)
#define ALIGN_SPEED             150     // mm/s of alignment and backing-up moves
#define MOVE_TIMEOUT_BASE_MS    5000    // move watchdog: base...
#define MOVE_TIMEOUT_PER_CELL_MS 2000   // ...plus per cell
#define START_DELAY_MS          2000    // countdown after START (hands away)
#define BUTTON_DEBOUNCE_MS      20

// ---- Flash store (flash_store.c) ----------------------------------------------------------
// Every write that wedged the flash (docs/freezes.md) began within ms of the
// end of a move; 92 writes at rest or 1-2 s after hard moves were all fine.
// A write first waits until the motors have been off this long and the
// UART has nothing left to send.
#define FLASH_SETTLE_MS         1000

// ---- Planner / strategy ------------------------------------------------------------
// Costs in arbitrary units per cell and per 90 deg turn. Search moves stop at
// every cell, so a turn costs about half a cell. Speed runs take every turn
// as a smooth curve without stopping: with a turn at half a cell the routes
// are the fastest at FAST 500-1000 and curves at 400-480 mm/s, 0.2-2 % faster
// than at two cells (the cost of braking to turn in place). Measured on the
// path reference over 600 random mazes: host_tests --costs [FAST CURVE].
#define SEARCH_COST_CELL        2
#define SEARCH_COST_TURN        1
#define FAST_COST_CELL          2
#define FAST_COST_TURN          1
// Extra exploration after the goal, looking for a better path. The cheap
// speed-run turns make more routes tie, and one simulated 16x16 maze in 100
// needed more than 300 (on average the search got shorter).
#define OPTIMIZE_MAX_STEPS      400
#define SEARCH_MAX_STEPS        2000    // hard budget of actions per run
#define MAP_MAX_RECOVERIES      3       // "goal unreachable" map repairs allowed per run

// ---- Runtime parameter defaults (see params.h) ----------------------------------------
// Practice-maze search + return: 22.3 s at 400 mm/s, 20.3 s at 500 (settle
// 10 ms), 19.4 s at 600 (settle 0), same map every time. 1-cell moves
// barely cruise at 3000 mm/s^2, so more speed gains little.
#define PARAM_SEARCH_SPEED      600     // mm/s
// Validated on 3-cell straights in a corridor: 700 mm/s stops within ~2 mm
// of the front-wall reference and centres (0.98 s); 900 works too (0.92 s)
// but runs out of PWM at the end of the acceleration. The motor model holds
// ~937 mm/s at most. Speed runs on the practice maze at 900 (with CURVE
// 480): 3 of 3 clean, tracking within 2.4 mm / 3.1 deg. With motors 20 %
// weaker (a low battery) 900 is out of PWM: the path's reference then slows
// down to the robot's pace (PATH_LAG_*), which keeps it within 4.3 mm of the
// reference in the simulator (24 mm and curves cut 47 mm inside without).
#define PARAM_FAST_SPEED        900     // mm/s, never below SPD
// Speed through the smooth curves, capped by the straights' speed, by the
// braking room after the last curve and by what the motors can follow
// (CURVE_PWM_SHARE, ~478 mm/s). Speed runs on the practice maze (9 cells, 4
// curves, a U-turn): 400 in 3.28 s, 450 in 2.98 s, 480 (478) in 2.79 s, the
// tracking within 2.5 mm and the heading within 1.9 / 2.7 / 3.1 deg.
#define PARAM_CURVE_SPEED       480     // mm/s
// 5000 made the wheels slip when braking (the encoders stopped on target,
// the robot 6 mm further): 3000 is near the grip limit.
#define PARAM_ACCEL             3000    // mm/s^2 of every straight, up and down
#define PARAM_TURN_SPEED        500     // deg/s peak of in-place turns
#define PARAM_TURN_ACCEL        5000    // deg/s^2: a 90 deg turn takes ~0.3 s
#define PARAM_TURN_TICKS        TICKS_PER_TURN
#define PARAM_KP                0.7f    // deg of heading per mm off-centre (1.0 weaved with ROT_KP 20; 0.5 left 10 mm per 5 deg of yaw)
#define PARAM_KI                8.0f    // deg per mm off-centre per m travelled: a turn 5 deg off is centred within ~2 mm in 3 cells
#define PARAM_LOG_LEVEL         2
#define PARAM_TELEMETRY         1       // '@' lines for tools/robot_monitor.py

#endif // ROBOT_CONFIG_H
