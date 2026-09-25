# AGENTS.md

## Project objective
"rat": micromouse firmware for an STM32F103 "Blue Pill" robot. It explores an
unknown maze, maps it, finds the fastest route and runs it:

1. **Search** (mode 1): explore to the goal, keep exploring the cells that
   could still shorten the speed-run path until the best path is verified,
   return to the start exploring on the way, face north, save the map. It
   stops in every cell: the user wants the search robust before fast. With
   `CONT ON` it drives through the cells without stopping instead, deciding
   each one on the way (smooth curves into openings it has just seen), and
   stops only to turn in place, at walls it finds in front, at the goal and
   at the end.
2. **Speed run** (mode 2): drive the verified fastest path in one continuous
   move: straights at `FAST`, every turn a smooth curve at `CURVE` (no stop,
   no turning in place). Then return to the start the same way at `SPD` and
   save again.

Goal configuration:
- Competition 16x16: goal = center 2x2 block (default without PRACTICE_MAZE).
- Practice maze 4x3: goal = cell (3,2), selected by `-DPRACTICE_MAZE=1` in the
  `bluepill_f103c8` environment of `platformio.ini`. **Remove it (or send
  `GOAL 7 7 8 8` + `SAVE`) before a real 16x16 competition.**

## Hardware
- The robot is 72 mm wide. No IMU/gyro: the heading comes only from the
  encoders, corrected by the side walls (centring) and front walls (squaring).
- MCU STM32F103C8T6 (Cortex-M3, no FPU, 20 KB RAM, 64 KB flash). Bare HAL via
  `framework = stm32cube`; no RTOS, no CubeMX `main.h`.
- 2 DC motors: H-bridge direction pins + PWM on TIM4 CH3 (right, PB8) / CH4
  (left, PB9), ~72 kHz.
- 2 quadrature encoders: TIM1 = left, TIM2 = right (16-bit counters, extended
  to 32-bit odometry every 1 ms in `encoder_tick()`).
- 4 IR distance sensors (FL/FR/SL/SR) on ADC1, scanned continuously by circular
  DMA into a 16x oversampling buffer; `ir_mm()` applies a cubic calibration
  (float, Horner). Raw calibration tables in `src/calib.txt`.
- The side sensors SL/SR are mounted at the nose, angled 15 deg forward (not
  perpendicular). Stopped too far forward or yawed, their beam leaves the cell
  by the post and reports a phantom side wall (yawed left: SR; right: SL).
- 6 LEDs (1-3 left, 4-6 right) + 2 buttons (`START` PC13, `SELECT` PB5),
  `GPIO_NOPULL` with external resistors, debounced in SysTick (20 ms) because
  motor PWM noise can glitch them.
- HC-05 Bluetooth on USART3 (9600 baud), `/dev/rfcomm0` on the PC (RFCOMM
  channel 1, see `tools/rfcomm_reconnect.sh`).
- `MOTORS_ENABLED` in `src/motor.h` is a hard kill-switch (0 = bridge never
  driven, for bench testing on USB power). Currently 1.

## Build / flash / test
Environments in `platformio.ini` (`default_envs = bluepill_f103c8`):
- `bluepill_f103c8`: robot firmware (`main.c`), `-DPRACTICE_MAZE=1`.
- `uart_test`, `diag_test`: hardware smoke tests (UART loopback; LED + IR +
  encoder panel for `tools/dashboard.py`). They exclude the app modules
  (`[common] app_sources`).

Commands:
- `pio run` builds the robot firmware; `pio run -e <env>` any other.
- `pio run -t upload` flashes the robot firmware (only it, thanks to
  `default_envs`; without it every env was flashed in turn, the last winning).
- `make -C test/host` builds and runs the host test suite (see below).
  `test/host/build/host_tests --demo` shows a full search + speed run log and
  the ASCII map exactly as the robot prints them; `--transcript <seed>
  <openings> [practice] [phantom]` prints everything the robot would send
  over Bluetooth (telemetry included) during a search + speed run;
  `--control` prints the speed control's numbers on the simulated robot
  (straights, turns and curves at several speeds), for tuning gains away
  from it; `--costs [FAST CURVE]` times the speed run's routes for several
  planner costs over random mazes (how `FAST_COST_*` were chosen).
- `python3 -m unittest discover -s tools -p 'test_*.py'`: monitor and
  calibration analysis tests. They replay `--transcript` output, so they also
  check that the monitor's planner makes the same decisions as the firmware.
- CI (`.github/workflows/ci.yml`) builds all envs and runs both test suites.

**Flashing quirk**: the ST-Link V2 clone has old firmware that only supports
the deprecated HLA transport. `upload_protocol = custom` calls openocd with
`interface/stlink-hla.cfg`. Do not switch back to `upload_protocol = stlink`
(fails with "unable to connect to the target"). If upload fails, suspect the
SWD/USB cable first (`lsusb` should show 0483:3748).

`board_upload.maximum_size = 64512`: the last 1 KB flash page (0x0800FC00)
stores the map; the build fails if the program would grow into it.

## Architecture (`src/`)
Strategy code is pure C with no HAL, so the same files run on the PC tests.

- `robot_config.h`: every compile-time constant with its unit (geometry,
  calibration, thresholds, planner costs) and the defaults of the runtime
  parameters.
- `params.c/.h`: runtime parameters in physical units (`SPD`, `FAST`,
  `CURVE`, `ACCEL`, `TURN`, `TACCEL`, `TURNTICKS`, `KP`, `KI`, `LOG`,
  `TELEM`), persisted with the map.
- `maze.c/.h` (pure): map + planner.
  - Walls carry signed evidence in [-3, 3], one slot per wall shared by both
    cells. > 0 wall, <= 0 passable for exploration, <= -2 (two consistent
    sightings or physically crossed) trusted by speed runs. The border is
    always a wall.
  - Planner: shortest paths over (cell, heading) states with a cost per cell
    and per 90 deg turn (SPFA). Optimal in time, not only in cells.
  - `maze_route()`: the optimal path as the speed run drives it, an in-place
    turn first and then every cell with the turn made inside it.
- `search.c/.h` (pure): strategies. `search_explore()` (phases META ->
  OPTIM -> VUELTA; at rest it senses, plans and turns in place, and every
  move forward is a leg decided cell by cell in `explore_next()`), `search_fast_run()` (the whole verified route in one
  move, replanned at every stop, falls back to exploring if the map proves
  wrong), `search_wall_follow()`. Also `search_print_map()` (ASCII map).
- `control.c/.h` (pure): the speed control. Trapezoidal motion profiles, two
  position loops (forward mm, rotation deg) with a motor-model feedforward,
  settling integrals against static friction, and the wall centring
  (`steer_step()`).
- `path.c/.h` (pure): continuous runs through several cells. A reference
  generator stepped in SysTick in place of the profiles: straights on the
  cells' centre lines, a clothoid-arc-clothoid 90 deg curve inside every cell
  where the route turns, speed limits and braking for the curves and the end.
- `motion.h`: the robot actions the strategies use. `motion.c` implements
  them on the robot; `test/host/sim.c` implements them in a simulated maze.
- `motion.c`: SysTick steps the profiles (or the path), the centring and the
  loops every millisecond and drives the motors; the move functions (main
  context) watch the sensors and decide when a move is over. Every forward
  move is a path (`motion_run_path()`; `motion_forward()` is one without
  curves; `motion_explore()`, the search's legs, is one that grows as the
  search decides each next cell); also turns in place, front-wall alignment, back-up, wall sensing
  (5 samples, 4 votes), run control (abort/pause/step) and `TUNE`.
- `storage.c/.h` (pure) + `flash_store.c/.h`: CRC-32 protected record (map,
  goal, parameters) in the last flash page.
- `telemetry.c/.h` (pure): compact `@` lines for the live monitor (format
  documented in `telemetry.h`).
- `calib.c/.h`: calibration recorder (CAL command): samples encoders,
  requested PWM (`motor_get()`), raw IR and the profile reference from
  SysTick into a 320-sample buffer (6.4 KB; when full it halves its
  resolution instead of dropping the end), then dumps them as `@D` lines
  with every constant.
- `commands.c/.h`: Bluetooth console (table in `COMMANDS[]`).
- `main.c`: init, mode selection UI, run dispatch, `app_systick()`.
- Drivers: `motor`, `pwm`, `encoder`, `infrared`, `gpio`, `uart`, `msp.c`
  (pins/DMA/IRQs), `sysclock.c`, `stm32f1xx_it.c` (SysTick, fault handlers),
  `error.c`, `stm32f1xx_hal_conf.h`.
- `test_uart.c`, `test_diag.c`: entry points of the smoke-test envs.
- `test/host/`: `sim.c` implements `motion.h` in a simulated maze (strategy
  tests); `control_sim.c` runs `control.c` (and `path.c`: `sim_path()`)
  against simulated motors (first order, friction, stiction, dead time),
  quantised encoders, delayed side IR and the chassis' stick-slip in yaw
  (speed-control tests).
- `tools/robot_monitor.py`: live maze monitor + console (curses). Rebuilds
  the map from the telemetry, recomputes the route with a Python port of the
  planner (costs read from `robot_config.h`), renders each frame into an
  off-screen `Canvas` then blits it (clipped, resize-safe), records sessions
  to `tools/logs/`, replays them (`--replay`), saves `@D` dumps as CSV in
  `tools/calib_data/` (`/nota` appends measurements).
- `tools/calib_analyze.py`: reports and suggested constants from those CSVs.
- `tools/dashboard.py`: live panel for `diag_test`.

## Operating the robot
- SELECT cycles the mode (LED n = mode n): 1 search, 2 speed run, 3/4 left/
  right wall follower, 5 sensor monitor (motors off), 6 erase map (confirm with
  a second START within 3 s). START launches it after a 2 s countdown.
- During a run, START (or `STOP`) aborts. The robot knows it is ready when it
  finished a run back at the start; after an abort, place it at the start
  facing north and press START (or send `HOME`, then `START`).
- The map survives resets and reflashes. Boot prints whether one was loaded:
  send `ERASE` (or run mode 6) when moving to a different maze. A firmware
  with new parameter defaults keeps the map and goal but starts from its own
  defaults; one built for another maze (default goal) or with another record
  layout (`STORE_VERSION`) ignores the saved record: tell the user before
  such a flash, the robot will need a new search.

## Bluetooth console (9600 baud, one command per line, case-insensitive)
`HELP` lists everything. Main ones: `MODE n`, `START`, `STOP`, `PAUSE`,
`RESUME`, `STEP ON|OFF` (alias `DEBUG`), `STATUS`, `MAP`, `IR`, `WALLS`,
`SPD n`, `FAST n`, `CURVE n`, `ACCEL n`, `TURN n`, `TACCEL n`, `TURNTICKS n`,
`KP f`, `KI f`, `TUNE [name value]` (control constants live, not saved),
`LOG 0-2`, `DEFAULTS`, `GOAL x y [x1 y1]`, `SAVE`, `ERASE`, `HOME`, `RESET`,
`SYNC`, `TELEM ON|OFF`, `CONT ON|OFF` (search without stopping: off by default,
until reset), `CLOCK [HSI]` (clock source; HSI switches to the internal one),
`CAL NOISE|STRAIGHT|TURN|CURVE|STEP|IR|DUMP`.
- Lines starting with `@` are telemetry for the monitor (`@D` = calibration
  dump); human-readable output never starts with `@`.
- Commands that block, write flash or use the planner (`MAP`, `WALLS`,
  `SAVE`, `ERASE`, `GOAL`, `HOME`, `MODE`, `HELP`, `SYNC`, `CAL`) are refused
  during a run. `CAL` only validates and queues the test: it runs from the
  main loop, so `STOP` keeps working during the test and the dump.
- Decimals are parsed by hand (`parse_decimal`): this nano-libc has no `%f`
  in scanf/printf. Print floats with `format_fixed()` / `format_fixed2()`.

## Conventions / hard-won gotchas
- `stm32f1xx_hal_conf.h` needs `board_build.stm32cube.custom_config_header =
  yes`, or the framework silently uses its own copy.
- Keep strategy and control code (`maze`, `search`, `storage`, `params`,
  `crc32`, `telemetry`, `control`) free of HAL includes and run `make -C
  test/host` after touching it. Headers used by it (`motion.h`, `uart.h`,
  `flash_store.h`) must stay HAL-free too.
- A move only updates the pose when it is confirmed (`MOVE_OK`). `MOVE_BLOCKED`
  means the robot stopped at a known cell centre facing a wall (backed up to
  where it started, or, in a path, at the cell `entered` reports), so the
  pose is still valid. Anything else invalidates it and ends the run
  (`search_ready()` = 0).
- Never drive forward while the front sensors see a wall, whatever the map
  says: the sighting raises the wall's evidence and the planner converges.
- Side readings can be `SEEN_DOUBTFUL` (`motion_doubt_sides()`): with
  something in front, a "wall" reading on the side exposed by the pose (too
  close to the front wall, FL-FR skew, or only one front sensor seeing
  something) is not recorded at all. Never turn a doubtful reading into
  "absent": the failure only produces phantom walls, and "absent" readings
  stay trusted. `FRONT_SQUARE_OFFSET_MM` (FL-FR when square to a wall, -15:
  median of 44 IR stops) makes the yaw rule symmetric and is the target of the
  front squaring. Confirm it with the robot centred and squared by hand to a
  wall: `CAL NOISE`, then `calib_analyze.py` prints the value.
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
  any speed; above `STEER_VREF_MM_S` KP scales as 1/speed (it weaved at 700).
  `KI` learns the heading misalignment a turn leaves, only within
  `STEER_BIAS_WINDOW_MM` of the centre (it overshot otherwise). The encoders'
  sideways motion since the reading is added to it (a Smith predictor for the
  IR delay). Readings are slew-limited (posts and wall edges jump), averaged
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
  than `ALIGN_DEADBAND_MM`.
- `TURNTICKS` (405) is the wheel track as the encoders see it in in-place
  turns (the wheels scrub): half the wheel difference of a real 90 deg.
  Calibrated facing a wall with `CAL NOISE`, `CAL TURN 4`, `CAL NOISE`,
  `CAL TURN -4`, `CAL NOISE`, comparing FL - FR (~1.2 mm per degree): 0.2
  deg per turn, and the turns are truly in place.
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
- Search without stopping (`motion_explore()`, `explore_next()`): each leg
  starts from rest and grows cell by cell (`path_grow()`, SysTick masked).
  With the slow IR the robot decides a cell with what it can have read by
  then: after a straight, the side walls read from `SEARCH_SIDE_FROM_MM`
  before it (the angled beams hit its walls), decided just in time to start
  a curve there (`SEARCH_DECIDE_S`); after a curve those sides cannot be read
  in time, so it decides halfway in (`SEARCH_LATE_*`, straight on or stop
  only). The cell's front wall is unknown when deciding before it: going
  straight on, the existing wall stop ("PARED") halts it at the cell's
  centre; curving, it reads that wall at the start of the curve, still
  facing it (`SEARCH_FRONT_WALL_MM`); without that, every corner left an
  unknown wall the search had to come back for, and perfect mazes took 37 %
  longer than stopping. A curve needs the side read open by every reading
  (`SEARCH_MIN_READINGS`) and the map agreeing: a false "open" would curve
  into a wall with no way to stop. Legs run at min(`SPD`, `CURVE`), constant
  speed. Turning in place, the goal (writing flash stalls the CPU) and the
  end happen at rest. In the simulator (`sim.c` has a time model from the
  robot's logs) it drives the same cells as stopping in every cell, finds
  the same verified optimum and takes ~30 % less time; with sensor noise
  its maps are right more often. `CONT OFF` goes back to stopping.
- Speed-run planner costs: with smooth curves a turn costs half a cell
  (`FAST_COST_TURN` 1): fastest in `host_tests --costs` at every speed tried,
  0.2-2% faster than pricing it as a stop-and-turn. The cheap turns make
  more routes tie, so the search's OPTIM phase needs a larger budget
  (`OPTIMIZE_MAX_STEPS` 400).
- Memory: the map and planner are sized for 16x16 in every build
  (`PRACTICE_MAZE` only changes the goal), so the practice and competition
  builds use the same RAM (86.8 %: ~2.7 KB left for the stack) and flash
  (90.4 % of 63 KB). Keep that headroom: report sizes after every change,
  reuse buffers (the search's legs borrow the speed run's route buffer).
- Health checks (`health.c`), for rare failures on the robot: the free
  stack is painted at boot and `STATUS` shows how much was never used
  (static estimate of the deepest chain: ~1.65 KB); if the main program
  stops calling `health_alive()` (every wait loop does) for more than
  `HEALTH_STALL_MS`, SysTick notes the program counter it interrupted and
  the main loop prints "!! el programa estuvo parado N ms en PC=..." when it
  resumes (map the PC with `arm-none-eabi-addr2line -e firmware.elf`); the
  banner says why the last reset happened.
- Clock (`sysclock.c`): crystal x 9 = 72 MHz. If it does not start at boot
  the robot runs on the internal oscillator / 2 x 16 = 64 MHz (+-1 %) and the
  banner says so; before, `Error_Handler` ran before the LEDs and UART were
  set up and the robot sat dark. The clock security system watches the
  crystal while running: on a failure the NMI stops the motors and aborts
  the run (`clock_failure_hook()` in motion.c), and the main loop brings the
  clock back to 64 MHz (`sysclock_recover()`, `uart_retime()`) and reports
  "!! fallo del cristal". `CLOCK HSI` does that switch on purpose, to test
  it.
- Unexplained incident (first robot run of the search without stopping):
  the robot froze twice for ~200 s (no output; a map save hung and failed;
  later a `SAVE` at rest took 199 s and failed), and after a software reset
  did not boot, all LEDs off, until a power cycle. Then both searches ran
  clean. Suspects: the crystal (but it had worked for 10 years), the supply,
  or software; static analysis rules out a stack overflow. The health checks
  and the crystal monitor above tell them apart if it happens again: a stall
  report points at software (and where), a crystal report at the crystal,
  neither at the supply or the flash itself.
- The UART TX queue drops messages when full (never blocks a control loop).
  Bulk output while stopped uses `uart_wait_space()`.
- Only `print()`/`uart_send()` from the main context, never from interrupts.
- Telemetry is sent only between actions, never from a control loop. Any map
  change the per-cell `@C` lines do not cover must be followed by
  `telemetry_map()` (see the map repair in `plan_explore()`).
- `tools/robot_monitor.py` ports `maze_plan_to/from`, `maze_best_action`
  (same tie order), `maze_route()` with the route text of the log ("2D1I3":
  cells, then a curve right/left in the last of them) and the OPTIM
  candidates. Change both together; the transcript tests fail if they
  diverge.
- float, never double: the M3 has no FPU (`-Wdouble-promotion` is on).
- Motors are stopped at register level in every fault handler and in
  `Error_Handler()`; fault = slow blink, `Error_Handler` = fast blink.

## Current status
Verified on the PC simulator (hundreds of random 16x16 and 4x3 mazes, with and
without sensor noise): searches always complete, the verified speed-run path
is optimal with perfect sensing, and nothing crashes (also with 20% of side
readings doubtful). The speed control is tested on the simulated robot
(exact distances and angles, centring from bad starts, +-20% model errors).
On the practice maze with the speed control: search + return in 19.4 s at
SPD 600 (22.3 s at 400), the same map every time and no doubtful wall;
3-cell straights at 700 mm/s stop within ~2 mm of the front-wall reference
and centre (900 works but runs out of PWM at the end of the acceleration);
turns within 0.2 deg each (TURNTICKS 405); FRONT_SQUARE_OFFSET_MM confirmed
with CAL NOISE (-15.4).

Smooth curves on the robot (practice maze): the first speed run at FAST 700
/ CURVE 400 drove the 9-cell route with 4 curves (a U-turn included) in one
move, 3.14 s to the goal, tracking within 2.7 mm / 2.6 deg, and the IR stop
at the goal landed 0.5 mm from the encoders' plan after 1.49 m; the return
at 600/400 took 3.27 s. Six CAL CURVE at 300 mm/s: the curve itself turns
89-90 deg on the encoders and, with the maze's lateral bias taken out
(two left and two right between (2,0) and (3,1)), ends 1.6 +- 3 mm early:
CURVE_PRE/POST/ANGLE stay at 0/0/90. The heading offset the centring holds
through a curve (up to 3.2 deg in these tests) is its learned bias, the
misalignment of the encoders' frame: in the one test where it could be
checked (start heading measured on a wall, 3.35 deg off) it put the robot
into the curve within 0.1 deg of the corridor, and the exit came out centred
once the maze's lateral bias was taken out. Keep holding it. Raising the
speeds (practice maze, 9 cells, 4 curves): CURVE 400 / 450 / 480 (478, the
motor cap) reached the goal in 3.28 / 2.98 / 2.79 s, FAST 900 in 2.74-2.92 s
(3 of 3 clean). Motors that cannot keep up (a low battery) used to fall
24-42 mm behind the reference at FAST 800-900 in the simulator, and as the
curves follow the reference's distance they started early and cut inside.
The reference now slows down while the robot lags (`PATH_LAG_*`): on the
robot at FAST 900 with `TUNE MOTOR_SCALE 0.8` (the motors get 80 % of the
PWM, as with a LiPo at its cutoff) the run slowed to 81 % where needed,
stayed within 4.1 mm of the reference and took 2.84 s instead of 2.76, as
the simulator predicted (4.3 mm, 78 %). Defaults FAST 900, CURVE 480.

Search without stopping (`CONT ON`): same cells and optimum as stopping in
every cell in the simulator, ~30 % faster. On the practice maze the whole
search took 15.2 s against 20.7 s, map right and saved; its first robot run
hit the unexplained incident above. Off by default until that is
understood. Still to validate: the curve's front-wall reading
(`SEARCH_FRONT_*`), other layouts.
