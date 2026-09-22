# AGENTS.md

## Project objective
"rat": micromouse firmware for an STM32F103 "Blue Pill" robot. It explores an
unknown maze, maps it, finds the fastest route and runs it:

1. **Search** (mode 1): explore to the goal, keep exploring the cells that
   could still shorten the speed-run path until the best path is verified,
   return to the start exploring on the way, face north, save the map.
2. **Speed run** (mode 2): follow the verified fastest path with merged
   straights at `FAST` speed, then return to the start and save again.

Goal configuration:
- Competition 16x16: goal = center 2x2 block (default without PRACTICE_MAZE).
- Practice maze 4x3: goal = cell (3,2), selected by `-DPRACTICE_MAZE=1` in the
  `bluepill_f103c8` environment of `platformio.ini`. **Remove it (or send
  `GOAL 7 7 8 8` + `SAVE`) before a real 16x16 competition.**

## Hardware
- MCU STM32F103C8T6 (Cortex-M3, no FPU, 20 KB RAM, 64 KB flash). Bare HAL via
  `framework = stm32cube`; no RTOS, no CubeMX `main.h`.
- 2 DC motors: H-bridge direction pins + PWM on TIM4 CH3 (right, PB8) / CH4
  (left, PB9), ~72 kHz.
- 2 quadrature encoders: TIM1 = left, TIM2 = right (16-bit counters, extended
  to 32-bit odometry every 1 ms in `encoder_tick()`).
- 4 IR distance sensors (FL/FR/SL/SR) on ADC1, scanned continuously by circular
  DMA into a 16x oversampling buffer; `ir_mm()` applies a cubic calibration
  (float, Horner). Raw calibration tables in `src/calib.txt`.
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
  the ASCII map exactly as the robot prints them.
- CI (`.github/workflows/ci.yml`) builds all envs and runs the host tests.

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
- `params.c/.h`: runtime parameters (`SPD`, `FAST`, `TURN`, `KP`, `KD`, `KE`,
  `LOG`), persisted with the map.
- `maze.c/.h` (pure): map + planner.
  - Walls carry signed evidence in [-3, 3], one slot per wall shared by both
    cells. > 0 wall, <= 0 passable for exploration, <= -2 (two consistent
    sightings or physically crossed) trusted by speed runs. The border is
    always a wall.
  - Planner: shortest paths over (cell, heading) states with a cost per cell
    and per 90 deg turn (SPFA). Optimal in time, not only in cells.
- `search.c/.h` (pure): strategies. `search_explore()` (phases META ->
  OPTIM -> VUELTA), `search_fast_run()` (verified path, replanned at every
  stop, falls back to exploring if the map proves wrong),
  `search_wall_follow()`. Also `search_print_map()` (ASCII map).
- `motion.h`: the robot actions the strategies use. `motion.c` implements
  them on the robot; `test/host/sim.c` implements them in a simulated maze.
- `motion.c`: 1 kHz move loops (forward N cells with cruise/brake profile,
  90/180 deg turns, front-wall alignment, back-up), wall sensing (5 samples,
  4 votes), 100 Hz steering in SysTick, run control (abort/pause/step).
- `storage.c/.h` (pure) + `flash_store.c/.h`: CRC-32 protected record (map,
  goal, parameters) in the last flash page.
- `commands.c/.h`: Bluetooth console (table in `COMMANDS[]`).
- `main.c`: init, mode selection UI, run dispatch, `app_systick()`.
- Drivers: `motor`, `pwm`, `encoder`, `infrared`, `gpio`, `uart`, `msp.c`
  (pins/DMA/IRQs), `sysclock.c`, `stm32f1xx_it.c` (SysTick, fault handlers),
  `error.c`, `stm32f1xx_hal_conf.h`.
- `test_uart.c`, `test_diag.c`: entry points of the smoke-test envs.
- `tools/robot_monitor.py`: curses console for the robot firmware.
  `tools/dashboard.py`: live panel for `diag_test`.

## Operating the robot
- SELECT cycles the mode (LED n = mode n): 1 search, 2 speed run, 3/4 left/
  right wall follower, 5 sensor monitor (motors off), 6 erase map (confirm with
  a second START within 3 s). START launches it after a 2 s countdown.
- During a run, START (or `STOP`) aborts. The robot knows it is ready when it
  finished a run back at the start; after an abort, place it at the start
  facing north and press START (or send `HOME`, then `START`).
- The map survives resets. Boot prints whether one was loaded: send `ERASE`
  (or run mode 6) when moving to a different maze.

## Bluetooth console (9600 baud, one command per line, case-insensitive)
`HELP` lists everything. Main ones: `MODE n`, `START`, `STOP`, `PAUSE`,
`RESUME`, `STEP ON|OFF` (alias `DEBUG`), `STATUS`, `MAP`, `IR`, `WALLS`,
`SPD n`, `FAST n`, `TURN n`, `KP f`, `KD f`, `KE f`, `LOG 0-2`, `DEFAULTS`,
`GOAL x y [x1 y1]`, `SAVE`, `ERASE`, `HOME`, `RESET`.
- Commands that block, write flash or use the planner (`MAP`, `WALLS`,
  `SAVE`, `ERASE`, `GOAL`, `HOME`, `MODE`, `HELP`) are refused during a run.
- Decimals are parsed by hand (`parse_decimal`): this nano-libc has no `%f`
  in scanf/printf. Print floats with `fixed2()`.

## Conventions / hard-won gotchas
- `stm32f1xx_hal_conf.h` needs `board_build.stm32cube.custom_config_header =
  yes`, or the framework silently uses its own copy.
- Keep strategy code (`maze`, `search`, `storage`, `params`, `crc32`) free of
  HAL includes and run `make -C test/host` after touching it. Headers used by
  it (`motion.h`, `uart.h`, `flash_store.h`) must stay HAL-free too.
- A move only updates the pose when it is confirmed (`MOVE_OK`). `MOVE_BLOCKED`
  means the robot backed up to where it started, so the pose is still valid.
  Anything else invalidates it and ends the run (`search_ready()` = 0).
- Never drive forward while the front sensors see a wall, whatever the map
  says: the sighting raises the wall's evidence and the planner converges.
- The steering PD prefers the right wall because the SL and FR calibrations
  read ~10 mm long compared with `calib.txt` (SR and FL fit within ~5 mm).
  Recalibrate before switching to two-wall centering.
- Merged straights use `TICKS_FOR_CELLS(n) = n*CELL_TICKS + MOVE_EXTRA_TICKS`,
  assuming the +140-tick correction found on single-cell moves is per move.
  Verify on hardware: if long straights end long/short, tune the split.
- Stops are hard brakes at `search_speed` (fast straights brake to it first):
  `TICKS_PER_CELL`/`TICKS_PER_TURN`/`FRONT_WALL_REF_MM` were calibrated that
  way. Turns wait `TURN_SETTLE_MS` after stopping; the calibration includes it.
- The UART TX queue drops messages when full (never blocks a control loop).
  Bulk output while stopped uses `uart_wait_space()`.
- Only `print()`/`uart_send()` from the main context, never from interrupts.
- float, never double: the M3 has no FPU (`-Wdouble-promotion` is on).
- Motors are stopped at register level in every fault handler and in
  `Error_Handler()`; fault = slow blink, `Error_Handler` = fast blink.

## Current status
Verified on the PC simulator (hundreds of random 16x16 and 4x3 mazes, with and
without sensor noise): searches always complete, the verified speed-run path
is optimal with perfect sensing, and nothing crashes. Still to validate on the
real robot and maze: the search itself after the rework, merged straights and
`FAST` speed, `KE` heading hold (off by default).
