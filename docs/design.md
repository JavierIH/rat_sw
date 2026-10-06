# Design notes

Why the speed control, the centring, the IR handling, the curves, the search
legs, the health checks and the telemetry are the way they are. The numbers
behind the constants: docs/measurements.md. AGENTS.md has the rules.

- Speed control: every move is a motion profile (ramps at `ACCEL` or
  `TACCEL`, cruises, arrives at rest exactly on its target) that two position
  loops follow in SysTick, forward (mm) and rotation (deg), with a
  feedforward from the motor model (`MOTOR_KV_L/R`, `MOTOR_TAU_S`,
  `MOTOR_KS_PWM`, fitted from `CAL STEP` recordings by `calib_analyze.py`).
  The robot stops where it is told, so there are no coasting calibrations.
  Position loops, not speed loops: one encoder tick per ms is 111 mm/s of
  quantisation. Out of PWM the rotation keeps its share and the forward
  drive gives way. Far behind the reference means blocked or slipping:
  `MOVE_STALLED` / `MOVE_SLIPPED` (`FWD_ERROR_MAX_MM`, `ROT_ERROR_MAX_DEG`).
  Once a profile has arrived, settling integrals beat the static friction
  (~85 PWM) that used to leave the wheels ~1 mm / 1 deg short.
- The chassis resists changes of heading (stick-slip in yaw, likely the
  skids): `ROT_KP` 40 with `ROT_KD` 0.8 (0.6 rang at ~7 Hz at 700 mm/s).
  `ROT_KI` learns the motors' imbalance. Gains were chosen on the simulated
  robot first (`host_tests --control`), then on the robot with `TUNE`, which
  changes them live but only until reset: write validated values into
  `robot_config.h`.
- `ACCEL` 3000 mm/s^2 is near the grip limit: at 5000 the wheels slipped
  when braking (the encoders stopped on target, the robot 6 mm further).
- The IR (Sharp-type, a new value every ~16 ms) report the past. The front
  sensors are paired with where the robot was `IR_DELAY_MS` (50) earlier (a
  wall approach at 400 mm/s); on in-place turns all four respond in 4-8 ms
  and wall approaches give FL ~16 / FR ~28 ms, so the 50 is open
  (`docs/faults/centring.md`, section 7). `motion.c` keeps the forward position of the
  last 64 ms (`trail`) and pairs each reading with where it was taken. The
  wall at the end of a straight is tracked from `FRONT_TRACK_MM` (170) and
  the stop aimed at `FRONT_TRACK_REF_MM` from it: stops within ~2 mm at
  400-700 mm/s (the plain reading stopped 12 mm short). Before the last half
  cell, both front sensors closer than `FRONT_EMERGENCY_MM` plus the braking
  distance is an obstacle: brake and back up to the last cell centre passed
  (`MOVE_BLOCKED`). A wall seen square from farther away, where the map had a
  passage, stops the move at the centre of the cell before it (`end=WALL`).
- Centring (`steer_step()`, since 10-06, GitHub issue #1): the wall follower
  most micromice use, a PD on the turn rate. The heading offset added to the
  rotation reference turns at `KP` deg/s per mm off-centre plus `KD` deg per
  mm the error changes, so it keeps turning while the robot is off-centre
  and no yaw has to be learned. Both walls: their average; one: that one;
  none: the heading is held. Readings averaged over one sensor period
  (`STEER_AVERAGE_MS` 16: they step ~2 mm every ~16 ms), the error clamped
  at `STEER_ERROR_MAX_MM` (25), no derivative across a change of walls (a
  new reference). It fades out over the last 40 mm of a move and before
  every curve, and holds its offset through a curve until the IR read the
  new corridor. Defaults KP 8, KD 0.6, chosen in the simulator with the side
  IR 8 ms late (robust from 5 to 20 ms): against the old controller on the
  same robot model every case ends closer to the centre, crooked starts of
  30-35 deg included (`docs/faults/centring.md`). The controller it replaced
  (a P on the heading plus a yaw observer and a 66 ms delay prediction, KI,
  `STEER_VREF`, the agreement and far-pull rules) and why: the same doc.
  Side centres `SIDE_CENTER_L/R_MM` (89/76: SL reads long and SR short),
  confirmed on 2026-09-27 at 88.1/75.3 by rounds of four quarter turns at
  three offsets (slopes 0.95 SL, 0.99 SR); a 180 deg pair is only valid with
  the robot centred front to back (12-17 mm off, the beams land by the post).
- Side walls after a straight are read on the way in, `SIDE_PASS_MM` before
  its end, where the angled beams hit the middle of the walls (`sides=` in
  the log). Read at the stop they caught the next post as phantom walls (9 in
  the first logs, some at 82-98 mm). `wall_sense_t.moving` says which way
  they were read. Read at the stop right after a turn they gave 5 phantoms in
  14, so `sense_here()` then records only "no wall" (those walls were seen
  before the turn anyway). At the start they are trusted: the robot was
  placed centred by hand, and doubting them cost the 16x16 search 33% more
  actions in the simulator. No pause before sensing: the controlled stops do
  not rock the chassis (searches with 10 and 0 ms mapped the practice maze
  alike; `SENSE_SETTLE_MS` was removed 2026-09-30).
- After a move that ends facing a wall, `motion_align_front()` first squares
  the robot (rotates in place by (FL - FR - `FRONT_SQUARE_OFFSET_MM`) /
  `SQUARE_MM_PER_DEG` when beyond `SQUARE_TOL_MM`), which resets the heading
  error moves leave behind, then corrects the distance if it is off by more
  than `ALIGN_DEADBAND_MM`. Only up to `SQUARE_MAX_SKEW_MM` (15, ~12 deg;
  every squaring in the logs was under 15 mm): a robot arriving far
  off-centre reads the corner, and at 35 it turned 15 deg into the wall.
- `TURNTICKS` (403) is the wheel track as the encoders see it in in-place
  turns (the wheels scrub): half the wheel difference of a real 90 deg.
  Calibrated facing a wall with `CAL NOISE`, `CAL TURN 4`, `CAL NOISE`,
  `CAL TURN -4`, `CAL NOISE`, comparing FL - FR (~1.2 mm per degree): 0.2
  deg per turn, and the turns are truly in place. Every turn ends ~0.78 deg
  short by the encoders (it is done once within `SETTLE_DEG`) and the next
  move starts from there, so a scale fitted to 90s absorbs that and a 180
  over-turns: at 405 the speed runs of layout D started yawed +1.1..+2.0 deg
  right after the 180 at the start (the "-1.2 deg" the centring held on every
  straight; fitted from the side walls along the first straight, constant,
  not growing with distance: no wheel mismatch). At 403: +0.8 and -0.5.
  Handing that rest to the next move (`TURN_CARRY`) made 90+90 turn as a
  180 in the simulator (179.85 vs 179.62 deg), but gave no gain on the
  robot (layout D) and would need its own `TURNTICKS`: removed 2026-09-30.
- Smooth curves (`path.c`, speed run only; search moves still stop in every
  cell). A curve enters its cell on the centre line and leaves on the centre
  line through the side edge: `pre` mm straight, the clothoid-arc-clothoid
  (`CURVE_RADIUS_MM` 70, `CURVE_RAMP_MM` 30), `post` mm straight. It advances
  85.5 mm along each axis, so 4.5 mm are left before and after it and curves
  in consecutive cells (staircases, u-turns over two cells) join without
  overlapping. The heading is a function of the distance travelled, not of
  time, so the shape holds whatever the speed does (a PAUSE stops on the
  curve and resumes it). The curve speed is capped by `FAST`, by the room to
  brake after the last curve, and by the motors (`CURVE_PWM_SHARE`: ~480
  mm/s; in the simulator 700 fell 15-57 mm behind and cut inside). The
  centring holds its offset through a curve and restarts in the new
  corridor once the delayed IR read it. The front sensors are only used with
  readings taken on a straight.
- Tuning the curves: `CAL CURVE [+-1] [mm/s]` from a cell centre (one cell, a
  curve, one cell; best with side walls and a front wall in the last cell),
  then `calib_analyze.py` suggests `TUNE CURVE_PRE` (from where the side
  walls put the robot after the curve: wide = start earlier), `CURVE_POST`
  (from where the front wall moved the stop) and `CURVE_ANGLE` (FL - FR at
  the end; noisy, average several). The side readings of this maze carry a
  per-cell bias of up to ~7 mm (a wall slightly off, SL/SR references), which
  looks like a curve exiting wide or inside: judge `CURVE_PRE` from left and
  right curves between cells with walls on both sides (the bias flips sign
  between the two directions and cancels in the average), never from one
  run. `TUNE CURVE_R|CURVE_RAMP` change the
  shape and refuse shapes that do not fit a cell. `LOG 2` prints each route
  as `route N cells, C curves: end=... v=vmax/vcurve`.
- Curve slip: moving, the wheels slip sideways and a curve turns less than
  the encoders say, the more the faster. Layout C's staircase (1R1L1R2, net
  one curve) at 478 mm/s, heading the centring had to hold on the last
  straight (average over it, 2026-09-26): `CURVE_ANGLE` 90 -> +2.4 and
  +3..+5 deg, 92.5 -> -0.2 and -3.1 (runs after a search start ~8 mm off
  and yawed); best ~92, +-2 per run. At 300 mm/s single curves matched 90
  earlier. So a curve at v asks for `CURVE_ANGLE` + `CURVE_SLIP` (v/480)^2
  encoder degrees (2.0; `TUNE CURVE_SLIP`, applied in `path_start()` at the
  path's curve speed). In the simulator (`curve_slip` of the plant) the
  staircase comes out -1.9 deg off without it, within 0.2 with it at 480
  and 300 mm/s. But every speed run on layout C started yawed ~+1.2 deg by
  the 180 at the start (see `TURNTICKS`), so 2.0 may be too much now.
  Layout E (a staircase of six curves, no straight between them), lateral
  at the first reading of the last straight: CURVE 480 +19..+21 mm left (4
  runs, entering the staircase within 0.25 deg of parallel), CURVE 300 +1
  and -3; `CURVE_PRE -8` +43 (crashed on the return); `CURVE_SLIP 0` +12,
  then +84 (PWM saturated, pace 81 %: crashed). In a staircase a lateral
  error becomes a longitudinal one at the next curve and back, so small
  per-curve changes add up: a kinematic model gives +24 mm for `CURVE_PRE
  -8` alone (measured +23) and ~10 mm per degree of angle error per curve.
  `calib_analyze.py` now fits, per straight, the encoder heading that runs
  parallel to the walls: on E the last straight's is ~+3.5 deg (1.7..6.3)
  beyond the first's at both 300 and 480 (3 right and 3 left curves: left
  ones turn ~0.6 deg more each?), so the speed-dependent +20 mm is a
  displacement (sideways slip), not the heading. Never tune curves live on
  a full-speed staircase: calibrate them on single curves with a straight
  after (the centring recovers there).
  The displacement is compensated by starting each curve later: `pre` +=
  `CURVE_PRE_SLIP` (v^2 - V0^2) / (480^2 - V0^2) mm above `CURVE_PRE_V0`
  (300). On E, lateral at the last straight's first reading (09-26 22:53 and
  09-27 00:47, TURNTICKS 403 the second time), slip 0: 300 -6.2/+5.4, 400
  +10.0/+8.1 (v^2 from 0 would predict ~+14: V0 300 fits), 480 +19.7/+21.3;
  at 480 slip 3 +6.8/+12.3, 5 +1.1/+2.0/+9.8, 7 +17.4 (once)/+4.6/0.0;
  slip 7 at 400 0.0, at 300 +0.4. Run to run +-5 mm (the start's yaw after
  the previous run's 180 varies -4..+2 deg); ~-2.7 mm per mm of slip, zero
  near 7: the default.
- Curve angle on the ring (layout F, 2026-09-26 20:03-20:31, CURVE_SLIP
  2.0, 300 mm/s): four `CAL CURVE` of one side round the island, back to
  the same spot. Physical rotation from FL-FR at a border stop (`CAL
  STRAIGHT 1` ending `end=IR`: same distance every time; FL-FR facing the
  island after a turn was off by several degrees: the end spot varies
  +-15 mm), against the encoder totals `IR` prints; in-place turns in +-
  pairs cancel. Without centring (`KP 0`, `KI 0`): right loop physical
  354.3 / encoders 362.8, left 363.0 / 362.4 (one of each): the curves
  turn ~1.0 deg less than the encoders count on both sides (90.78 asked ~
  89.8 real: fine at 300), plus a common leftward drift of ~4.6 deg per
  loop (~1.7 m, 6 stops: 0.3 % between the wheels, or the stops). With the
  centring on, the same right loop gave encoders 365.4 and 374.5 for ~360
  physical. Without centring the yaw carries from move to move: the third
  loop started 3 deg yawed (TURN 2 leaves ~4.5) and hit the island at the
  end of its first curve (300 mm/s).
  Offline, with `calib_analyze.py --chain` (the encoder heading summed over
  the session against the walls', per wall stretch; it agrees with FL-FR at
  every border stop within ~1.3 deg): the centring's rotations are real
  (inside a move the side walls see at least ~75 % of them). What does not
  add up is the encoders against the robot: they run ahead of it, to the
  right, ~3 deg per metre (layout E, no stops, same at 300 and 480: the
  "+3.5 deg" there; ~4.8 on the ring, a stop every 0.33 m: maybe ~0.5 per
  stop more; 0.3 % between the wheels would do it), and the curves turn
  1.3 +- 0.7 deg less than the encoders count (loops without centring:
  right 10.3, left 0.2 deg ahead): 90.78 asked turns ~89.5, so a real 90
  at 300 needs ~91.3 (the drift aside: it adds ~1.3 to every right curve
  and takes it from every left one). The
  centring keeps the robot parallel and the encoders count that correction
  (374.5 for 361.7 real by FL-FR); without it the same error stays as yaw
  (362.8 for 352.5 real). L1's "360" was FL-FR facing the island after a
  turn: the walls put it at ~359, and L4 had 0.38 m and two 180s more. The
  robot was re-placed by hand between 20:08 and 20:12 (-9 deg in the
  chain). The side readings also move ~1 mm per degree of yaw (the sensors
  at the nose, `SIDE_LEVER_MM`, the simulator's `side_lever_mm`): to the
  PD it is extra damping. Compensating it in the old observer
  (`SIDE_LEVER` 55) gave no clear gain: removed 2026-09-30.
  At 400 and 480 (21:06-21:22, centring on: KP 0.7 KI 8, CURVE_ANGLE 90,
  CURVE_SLIP 2.0; one right and one left loop each, same moves; every
  move OK, tracking <= 2.1 mm / 2.7 deg, <= ~10 mm off-centre at the
  stops): encoders minus real per loop, border stop to border stop, 400:
  right +20.6, left +4.8; 480: right +9.9, left +4.5. With the centring
  the robot ends every move parallel and the encoders count the
  correction, so a loop measures the slip whatever angle the curve asks:
  (right - left) / 8 = 2.0 (400), 0.7 (480); from the encoder totals of
  each curve move 1.6 and 1.5, from the chain's fits after each curve 2.1
  and 1.8. So ~1.5 +- 0.7 deg at 300-480: the slip does not grow as v^2
  here, the curves need ~91.5 encoder deg at every speed, and the asked
  90.78 / 91.39 / 92.0 are within ~0.7 of it, under the noise of a loop
  pair: CURVE_ANGLE / CURVE_SLIP stay. The common part, (right + left) /
  2, 12.7 and 7.2 per loop (~1.7 m, two 180s; 5.3 at 300 without
  centring), is more than the ~3 deg/m drift. The right loops end 7-23 mm
  short of (0,1) at every speed (the last straight to the border 188-204
  mm, 181 from the centre), no trend. A lone `CAL CURVE` cannot calibrate
  `CURVE_PRE`: its exit lateral (-12..+20 mm) follows the entry's (up to
  15 mm: it starts from the previous stop, 90 mm before the curve). The
  encoder drift can be trimmed live: `TUNE WHEEL_DIFF` (left wheel travel
  per tick / right - 1) moves the encoder heading by ~1011 x WHEEL_DIFF
  deg per metre; -0.003 would cancel 3 deg/m to the right.
  Trimmed on layout D (21:41-21:56, `CAL STRAIGHT` / `CAL TURN` at 300,
  centring on): three laps of the border, 2 N, 3 E, 2 S, 2 W, a 180 and
  back (3.1 m, 8 straights, 8-9 turns in place), with WHEEL_DIFF 0,
  -0.003 and -0.0025; every move OK, the stops alike in all three. The
  raw encoders (unweighted) turn right during the straights, where the
  centring holds the robot parallel: +2.28, +2.74, +3.51 deg/m (25.9 deg
  over 9.05 m: 2.9); `--chain` from the lap's first straight to its last,
  +1.4, +2.8, +2.5 (2.2; it includes the turns in place, whose errors
  swing it up to +-8 deg mid-lap and cancel over a lap). The mismatch did
  not change with the trim: it is the wheels. Weighted as the firmware
  does, -0.003 left -0.3 deg/m on the straights and ~0 over its lap,
  -0.0025 +1.0 and 0; the lap-to-lap spread (+-0.6) is larger than the
  difference, so the default is the middle of the two measures,
  `WHEEL_DIFF` -0.0025 (`robot_config.h`). The ring's recordings weighted
  with -0.003 keep ~1.5 deg/m, the curves' part (right loops gain, left
  ones lose). `TUNE` prints it rounded to 3 decimals (-0.003); the dumps
  carry it as `wheel_diff_ppm`, which `--chain` applies (`--wheel-diff`
  for older dumps, recorded with 0).
- Search legs (`motion_explore()`, `explore_next()`, `CONT ON`): each leg
  starts from rest and grows a cell at a time (`path_grow()`, SysTick
  masked) as the search decides each next cell, straight on or stop. The
  decision comes inside the cell, just before the reference would have to
  start braking for its centre, with its three walls in view: the sides read
  from `SEARCH_SIDE_FROM_MM` (20) before its entry edge to `SEARCH_SIDE_TO_MM`
  into it (only unanimous readings count; the angled beams hit ~70 mm ahead,
  and from 60 mm before the edge they caught its post: 1-cell legs left open
  sides and border walls doubtful), the front once it reads reliably
  (under ~170 mm). So the search sees the same walls as stopping there and
  explores exactly the same cells; if a wall is in front, or a turn is
  needed, it stops there as an ordinary stop (no hard braking) and turns in
  place. An unclear front means stopping to look. The legs run at
  `SEARCH_LEG_SPEED_MAX` (450 mm/s): the fastest at which the front wall is
  known before that decision (see the measurements below). In the simulator (`sim.c`
  has a time model from the robot's logs) it takes 35-45 % fewer stops and
  4-7 % less time than stopping in every cell (most straights between turns
  are short), and with sensor noise its maps are right more often (96/93 %
  at 1/3 % noise, against 63/31 %). A version that also curved in the search
  (30 % faster) was dropped: the user wants curves only in the speed run.
- Speed-run planner costs: with smooth curves a turn costs half a cell
  (`FAST_COST_TURN` 1): fastest in `host_tests --costs` at every speed tried,
  0.2-2% faster than pricing it as a stop-and-turn. The cheap turns make
  more routes tie, so the search's OPTIM phase needs a larger budget
  (`OPTIMIZE_MAX_STEPS` 800: 400 left 3 of the 520 real mazes short).

## Health checks and clock

- Health checks (`health.c`), for rare failures on the robot: the free
  stack is painted at boot and `STATUS` shows how much was never used
  (static estimate of the deepest chain: ~1.65 KB); if the main program
  stops calling `health_alive()` (every wait loop does) for more than
  `HEALTH_STALL_MS`, SysTick notes the program counter it interrupted and
  the main loop prints "!! the program stalled N ms at PC=..." when it
  resumes (map the PC with `arm-none-eabi-addr2line -e firmware.elf`); the
  banner says why the last reset happened. Until 0221cd6 that report was
  only printed in mode 5 (it had gone into the sensor monitor's loop), so
  earlier logs saying nothing about stalls prove nothing. Every flash
  operation starts the HSI fresh (the flash needs it; it is off otherwise,
  `docs/faults/oshwdem2026.md`), times itself with the DWT cycle counter
  and SysTick, and prints "!! flash: ..." with RCC_CR, FLASH_SR and
  FLASH_CR when slow or failed; `STATUS` shows those registers and the last
  write's times. (An oscillator watch on RCC_CR, which never fired, was
  removed 2026-10-06.)
- Clock (`sysclock.c`): crystal x 9 = 72 MHz. If it does not start at boot
  the robot runs on the internal oscillator / 2 x 16 = 64 MHz (+-1 %) and the
  banner says so; before, `Error_Handler` ran before the LEDs and UART were
  set up and the robot sat dark. The clock security system watches the
  crystal while running: on a failure the NMI stops the motors and aborts
  the run (`clock_failure_hook()` in motion.c), and the main loop brings the
  clock back to 64 MHz (`sysclock_recover()`, `uart_retime()`) and reports
  "!! crystal failure". (A `CLOCK HSI` command made that switch on
  purpose, to test it; removed on 2026-09-26 to save flash.)

## Telemetry (`telemetry.c`)

Machine-readable lines for tools/robot_monitor.py (live maze view). All
start with '@', are at most 21 bytes and are only sent between actions,
never from a control loop. TELEM OFF silences them. `@D` lines are
calibration dumps (`calib.c`).

```
  @M<m>                      selected run, app_mode_t 1-8
  @A<a>                      activity (telemetry_activity_t)
  @P<x><y><h>                pose: x, y one hex digit each, h one of NESW
  @C<x><y><h><n><e><s><w>    pose after sensing + that cell's four walls:
                             '#' wall, 'o' open (verified), '.' open (seen
                             once), '?' unknown
  @R<y><16 cells>            map row y, one char per cell from
                             0-9A-V = bits 0-1 north wall, bits 2-3 east
                             wall (0 unknown, 1 wall, 2 open once,
                             3 open verified), bit 4 visited
  @G<x0><y0><x1><y1>         goal rectangle
  @Y<n>                      full sync follows: forget the map, rows 0..n
```

While running, every sensing also sends one map row in rotation, so the
monitor's copy converges even when the UART queue drops a line.
