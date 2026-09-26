# AGENTS.md

## How to work here (keep sessions short)
- Start by reading `docs/ISSUES.md`: the open issues, each with its state
  and next step. Work on one; before stopping, close it or update its entry
  (result and next step in a line or two), so the next session resumes from
  the file alone. One session per issue.
- Details live in `docs/`: `control.md` (design notes and measurements of
  motion, sensing, curves, search legs), `history.md` (what was validated
  on the robot), `freezes.md` (the freeze investigation), `mazes.md` (the
  test layouts A-D, drawn). Read only what the issue needs; keep this file
  to rules and commands.
- The robot over Bluetooth: `python3 tools/bt_logger.py &` (holds
  `/dev/rfcomm0`, logs to `tools/logs/`, saves `@D` dumps) and
  `tools/robot.sh [-w regex] [-t s] "CMD" ...`. Only one program may hold
  the port (close `robot_monitor.py` first). Keep outputs short: grep the
  lines that matter, summarise recordings with `tools/calib_analyze.py`.

## Project objective
"rat": micromouse firmware for an STM32F103 "Blue Pill" robot. It explores an
unknown maze, maps it, finds the fastest route and runs it:

1. **Search** (mode 1): explore to the goal, keep exploring the cells that
   could still shorten the speed-run path until the best path is verified,
   return to the start exploring on the way, face north, save the map. It
   drives straight stretches without stopping, deciding each cell on the
   way with the walls it would see stopped there, and stops to turn in place
   (and at the goal and the end). No curves in the search: the user wants it
   robust before fast; curves are for the speed run. `CONT OFF` makes it stop
   in every cell.
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

`board_upload.maximum_size = 63488`: the last 2 KB (two flash pages from
0x0800F800) store the map; the build fails if the program would grow into
them.

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
  wrong). Also `search_print_map()` (ASCII map).
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
- `storage.c/.h` (pure) + `flash_store.c/.h`: CRC-32 protected records (map
  packed in nibbles, goal, parameters) as a log in the last two flash pages
  (six slots): saves only program erased slots, the boot erases.
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
- SELECT cycles the mode (LED n = mode n): 1 search, 2 speed run, 3 sensor
  monitor (motors off), 4 erase map (confirm with a second START within 3 s).
  START launches it after a 2 s countdown. (The wall followers, modes 3/4
  until 2026-09-26, were removed to save flash.)
- During a run, START (or `STOP`) aborts. The robot knows it is ready when it
  finished a run back at the start; after an abort, place it at the start
  facing north and press START (or send `HOME`, then `START`).
- The map survives resets and reflashes (up to five saves per power-on;
  then `SAVE` compacts). Boot prints whether one was loaded:
  send `ERASE` (or run mode 4) when moving to a different maze. A firmware
  with new parameter defaults keeps the map and goal but starts from its own
  defaults; one built for another maze (default goal) or with another record
  layout (`STORE_VERSION`) ignores the saved record: tell the user before
  such a flash, the robot will need a new search.

## Bluetooth console (9600 baud, one command per line, case-insensitive)
Every command is in the table of README.md ("Consola Bluetooth"; the
firmware has no HELP, to save flash). Main ones: `MODE n`, `START`, `STOP`, `PAUSE`,
`RESUME`, `STEP ON|OFF` (alias `DEBUG`), `STATUS`, `MAP`, `IR`, `WALLS`,
`SPD n`, `FAST n`, `CURVE n`, `ACCEL n`, `TURN n`, `TACCEL n`, `TURNTICKS n`,
`KP f`, `KI f`, `TUNE [name value]` (control constants live, not saved),
`LOG 0-2`, `DEFAULTS`, `GOAL x y [x1 y1]`, `SAVE`, `ERASE`, `HOME`, `RESET`,
`SYNC`, `TELEM ON|OFF`, `CONT ON|OFF` (search straights without stopping, the
default, or stopping in every cell; until reset),
`CAL NOISE|STRAIGHT|TURN|CURVE|STEP|IR|DUMP|RUN` (`RUN` arms the recorder
for the next continuous move of a run: the speed run to the goal, or a
search leg; the dump comes when the run ends).
- Lines starting with `@` are telemetry for the monitor (`@D` = calibration
  dump); human-readable output never starts with `@`.
- Commands that block, write flash or use the planner (`MAP`, `WALLS`,
  `SAVE`, `ERASE`, `GOAL`, `HOME`, `MODE`, `CONT`, `SYNC`, `CAL`) are refused
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
- Speed control, sensing, curves and search legs: the reasons and the
  measurements are in `docs/control.md`; read the section before changing
  that code. The rules:
  - Every move is a profile (or a path) that two position loops follow in
    SysTick, forward mm and rotation deg, with a motor-model feedforward;
    far behind means `MOVE_STALLED` / `MOVE_SLIPPED`. Choose gains in
    `host_tests --control` first, then `TUNE` on the robot, then write the
    validated values into `robot_config.h`. `ACCEL` 3000 is near the grip
    limit.
  - The IR report where the robot was `IR_DELAY_MS` (50) ago: pair readings
    with the `trail` positions; use the front sensors only with readings
    taken on a straight.
  - Centring: `KP` deg per mm, falling as 1/speed above `STEER_VREF_MM_S`
    (500; more weaves, 900 swung a speed run into a wall). The bias comes
    from the observer (`KI`, `STEER_OBSERVER_MM`): never integrate the
    lateral error itself (it learned an off-centre start as a heading error
    and aimed a speed run at a wall).
  - Side walls are read on the way in (`SIDE_PASS_MM`), not at the stop
    (posts give phantoms); right after a turn only "no wall" is recorded.
    Squaring on a front wall only within `SQUARE_MAX_SKEW_MM`.
  - `TURNTICKS` (403) is the in-place turn's scrub, calibrated with `CAL
    NOISE` / `CAL TURN` against a wall.
  - Curves are for the speed run only (the user's rule): clothoid-arc-
    clothoid, R 70, in one cell; curve speed capped by the motors (~480);
    each asks the encoders for 90 + `CURVE_SLIP` (v/480)^2 deg (sideways slip).
  - Search legs decide each cell inside it with its three walls in view, at
    most `SEARCH_LEG_SPEED_MAX` (450); no curves in the search.
  - Planner: `FAST_COST_TURN` 1 (a curve costs half a cell), which needs
    `OPTIMIZE_MAX_STEPS` 400 in the search's OPTIM phase.
- Memory: the map and planner are sized for 16x16 in every build
  (`PRACTICE_MAZE` only changes the goal), so the practice and competition
  builds use the same RAM (85.9 %: ~2.9 KB left for the stack) and flash
  (94.2 % of 62 KB, 3.6 KB left). Keep that headroom: report sizes after every change,
  reuse buffers (the search's legs borrow the speed run's route buffer).
- Health checks and clock (details in `docs/control.md`): `STATUS` shows the
  stack never used, the reset cause, RCC/FLASH registers, the chip's
  identity and the flash's free slots and last write's times; "!! ..." lines
  report stalls, unrequested oscillator changes, crystal failures and slow
  or failed flash operations. Clock: crystal x 9 = 72 MHz, else HSI 64 MHz
  (at boot, or after a crystal failure; never on purpose).
- Flash (`docs/freezes.md`): the chip is a clone (IDCODE 0x307) whose flash
  sometimes wedges until a power cycle, every operation ~9000x slower (an
  erase ~200 s, stalling the CPU). So nothing erases during runs: the map
  is saved once, at the end of a run, only if it changed, into an erased
  slot, a halfword at a time, each timed (the first slow one stops it and
  blocks the store until a power cycle; `RESET` refuses then: it once did
  not boot); only the boot erases (compacts), after a power-on or a good
  probe. Writes also wait for the motors off `FLASH_SETTLE_MS` and the UART
  quiet. Never add an erase to a run.
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
