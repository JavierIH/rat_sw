# Motion, control and sensing: design notes

Why the speed control, the centring, the IR handling, the curves and the
search legs are the way they are, with the measurements behind each choice.
Read the section you are about to touch; AGENTS.md has the rules.

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
- The IR (Sharp-type, a new value every ~16 ms) report where the robot was
  `IR_DELAY_MS` (50) earlier. `motion.c` keeps the forward position of the
  last 64 ms (`trail`) and pairs each reading with where it was taken. The
  wall at the end of a straight is tracked from `FRONT_TRACK_MM` (170) and
  the stop aimed at `FRONT_TRACK_REF_MM` from it: stops within ~2 mm at
  400-700 mm/s (the plain reading stopped 12 mm short). Before the last half
  cell, both front sensors closer than `FRONT_EMERGENCY_MM` plus the braking
  distance is an obstacle: brake and back up to the last cell centre passed
  (`MOVE_BLOCKED`). A wall seen square from farther away, where the map had a
  passage, stops the move at the centre of the cell before it (`fin=PARED`).
- Centring (`steer_step()`): the heading offset is proportional to the
  lateral error (`KP` deg per mm), so it converges over the same distance at
  any speed; above `STEER_VREF_MM_S` KP scales as 1/speed (it weaved at 700;
  `TUNE STEER_VREF 900` on a speed run swung it into a wall). The bias (the
  encoder heading that is parallel to the walls: the misalignment a turn
  leaves) is learned by an observer: the encoders predict how the readings
  should move, and what they do beyond that, per mm travelled, is bias still
  missing (`KI`, the same units as before, and `STEER_OBSERVER_MM`, how
  slowly the prediction follows the readings; a new wall restarts it). An
  off-centre robot moving parallel teaches it nothing. The old integral of
  the lateral error (`TUNE OBSERVER 0`, within `STEER_BIAS_WINDOW_MM`) took
  a start 8-12 mm off-centre for ~5 deg of bias in the first cell of a speed
  run, carried it through the curves and aimed the last straight at the
  wall (layout C: it arrived yawed 5.5 deg and 24 mm off, and once scraped
  and wedged). The encoders' sideways motion since the reading is added to
  the lateral error (a Smith predictor for the IR delay). Readings are slew-limited (posts and wall edges jump), averaged
  over a sensor period and referred to `SIDE_CENTER_L/R_MM` (89/76: SL reads
  long and SR short; measured with 180 deg turns, which mirror the robot
  across the centre line). With both walls their average is used, unless one
  reading is implausible: the angled beams catch posts and walls ahead
  (`STEER_ERROR_MAX_MM`), which once swerved the robot 35 deg. The offset is
  clamped (`STEER_MAX_DEG`) and curvature-limited (`STEER_CURVE_DEG_PER_MM`),
  and fades out over the last 40 mm so the robot stops parallel.
- Side walls after a straight are read on the way in, `SIDE_PASS_MM` before
  its end, where the angled beams hit the middle of the walls (`lados=` in
  the log). Read at the stop they caught the next post as phantom walls (9 in
  the first logs, some at 82-98 mm). `wall_sense_t.moving` says which way
  they were read. Read at the stop right after a turn they gave 5 phantoms in
  14, so `sense_here()` then records only "no wall" (those walls were seen
  before the turn anyway). At the start they are trusted: the robot was
  placed centred by hand, and doubting them cost the 16x16 search 33% more
  actions in the simulator. `SENSE_SETTLE_MS` is 0: the controlled stops do
  not rock the chassis.
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
  corridor once the delayed IR read it; its KI (the heading misalignment) is
  kept across curves. The front sensors are only used with readings taken on
  a straight.
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
  as `ruta N celdas, C curvas: fin=... v=vmax/vcurva`.
- Curve slip: moving, the wheels slip sideways and a curve turns less than
  the encoders say, the more the faster. Layout C's staircase (1D1I1D2, net
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
  known before that decision (see robot_config.h). In the simulator (`sim.c`
  has a time model from the robot's logs) it takes 35-45 % fewer stops and
  4-7 % less time than stopping in every cell (most straights between turns
  are short), and with sensor noise its maps are right more often (96/93 %
  at 1/3 % noise, against 63/31 %). A version that also curved in the search
  (30 % faster) was dropped: the user wants curves only in the speed run.
- Speed-run planner costs: with smooth curves a turn costs half a cell
  (`FAST_COST_TURN` 1): fastest in `host_tests --costs` at every speed tried,
  0.2-2% faster than pricing it as a stop-and-turn. The cheap turns make
  more routes tie, so the search's OPTIM phase needs a larger budget
  (`OPTIMIZE_MAX_STEPS` 400).

## Health checks and clock

- Health checks (`health.c`), for rare failures on the robot: the free
  stack is painted at boot and `STATUS` shows how much was never used
  (static estimate of the deepest chain: ~1.65 KB); if the main program
  stops calling `health_alive()` (every wait loop does) for more than
  `HEALTH_STALL_MS`, SysTick notes the program counter it interrupted and
  the main loop prints "!! el programa estuvo parado N ms en PC=..." when it
  resumes (map the PC with `arm-none-eabi-addr2line -e firmware.elf`); the
  banner says why the last reset happened. Until 0221cd6 that report was
  only printed in mode 5 (it had gone into the sensor monitor's loop), so
  earlier logs saying nothing about stalls prove nothing. `health_alive()`
  also watches the oscillator bits of RCC->CR against those the clock setup
  left (`health_clock_baseline()` after every intended change): a change is
  reported ("!! osciladores cambiados sin pedirlo") and the HSI turned back
  on. Every flash write checks the HSI (the flash needs it to erase and
  program) and starts it if stopped, times itself with the DWT cycle
  counter and SysTick, and prints "!! flash: ..." with RCC_CR, FLASH_SR and
  FLASH_CR when slow (> 200 ms), failed, HSI stopped or flagged; `STATUS`
  shows those registers and the last write's duration.
- Clock (`sysclock.c`): crystal x 9 = 72 MHz. If it does not start at boot
  the robot runs on the internal oscillator / 2 x 16 = 64 MHz (+-1 %) and the
  banner says so; before, `Error_Handler` ran before the LEDs and UART were
  set up and the robot sat dark. The clock security system watches the
  crystal while running: on a failure the NMI stops the motors and aborts
  the run (`clock_failure_hook()` in motion.c), and the main loop brings the
  clock back to 64 MHz (`sysclock_recover()`, `uart_retime()`) and reports
  "!! fallo del cristal". (A `CLOCK HSI` command made that switch on
  purpose, to test it; removed on 2026-09-26 to save flash.)
- OPEN INVESTIGATION, freezes: see "Freezes (open)" at the end.
