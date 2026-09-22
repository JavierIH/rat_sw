# AGENTS.md

## Project objective
"rat" — a micromouse-style maze-solving robot firmware. Goal: explore an
unknown maze cell by cell, build a wall map, use flood-fill to always know
the shortest known path to the goal, and navigate there autonomously
(competition rules allow multiple runs, so the long-term plan is
explore-then-speed-run, though the speed-run phase isn't built yet).

Two goal configurations coexist in the code:
- Real competition maze: 16x16 grid, goal = center 2x2 block (default in
  `maze_init()`, `src/maze.c`).
- Current practice maze: 4 cells wide x 3 tall, goal = single cell `(3,2)`,
  forced via the `-DPRACTICE_MAZE=1` build flag (only set for the
  `bluepill_f103c8` env in `platformio.ini`). **Remove/guard this override
  before a real 16x16 competition run.**

## Hardware
- MCU: STM32F103C8T6 "Blue Pill" (Cortex-M3, 20KB RAM, 64KB flash). Bare HAL
  via `framework = stm32cube` — no RTOS, no CubeMX-generated `main.h`.
- 2 DC motors via H-bridge (GPIO sense pins) + PWM (TIM4 CH3/CH4).
- 2 quadrature encoders (TIM1 = left, TIM2 = right), 16-bit counters.
- 4 IR distance sensors (FL/FR/SL/SR) via ADC1 + circular DMA; `get_ir_mm()`
  converts raw ADC to millimeters via a cubic calibration polynomial
  (coefficients in `src/calib.txt`).
- 6 LEDs + 2 buttons (`BUTTON_START`=PC13, `BUTTON_SELECT`=PB5). Both
  `GPIO_NOPULL` — relies entirely on external pull resistors on the PCB, so
  the start button in particular can read spurious presses from motor PWM
  noise (see `abort_requested()`'s debounce in `main.c`).
- HC-05 Bluetooth module on USART3 (9600 baud) for live telemetry + remote
  tuning; bound to `/dev/rfcomm0` on the dev machine (RFCOMM channel 1).
- `MOTORS_ENABLED` in `src/motor.h` is a hard killswitch: `0` makes
  `set_output()` a no-op (safe bench/USB-only testing), `1` drives the
  H-bridge for real. Currently `1`.

## Build / flash
PlatformIO project, 5 environments in `platformio.ini`, each isolating one
`src/*.c` entry point via `build_src_filter`:
- `bluepill_f103c8` — the real robot firmware (`main.c`), built with
  `-DPRACTICE_MAZE=1`.
- `uart_test`, `diag_test`, `maze_test`, `flood_test` — standalone smoke
  tests (UART loopback, LED+IR panel, wall-sensing, flood-fill logic). Each
  excludes `main.c` and the other tests' `.c` files. No unit-test framework;
  these are separate firmware images you flash and observe.

Build one env: `pio run -e <env>`. Build + flash: `pio run -t upload -e <env>`.

**Flashing quirk**: the attached ST-Link V2 clone has old firmware that only
supports the deprecated HLA transport. `upload_protocol = custom` in
`platformio.ini` calls `openocd` directly with `interface/stlink-hla.cfg`
(not the default `stlink.cfg`). Do not change `upload_protocol` back to
`stlink` — it will fail with "unable to connect to the target". If upload
fails, first suspect the physical SWD/USB cable connection (check `lsusb`)
before suspecting the code or the openocd config.

## Architecture (`src/`)
- `main.c` — robot behavior. `explore_run()` is the main real behavior:
  flood-fill maze exploration loop. Confirmed-motion primitives:
  `cross_cell()`, `turn_left()`, `turn_right()` (each returns 1 only if
  encoder/IR actually confirmed the move — never assume success from a
  timeout-bounded loop finishing). `sense_walls_here()` feeds IR readings
  into the maze map. `correct_against_front_wall()` trims small drift
  against a detected front wall. `process_uart_commands()` implements the
  live tuning protocol (below). `run_left_side()`/`run_right_side()` are an
  older simple wall-following mode, kept but unused by default.
- `maze.c` / `maze.h` — 16x16 grid, per-cell wall bitmask (N/E/S/W =
  UP/RIGHT/DOWN/LEFT, matches `robot_heading_t`), confidence-gated wall
  observation (`maze_observe_wall`), BFS flood-fill (`maze_compute_flood`),
  next-move selection (`maze_next_move`, ties prefer keeping heading).
- `motor.c`/`motor.h`, `pwm.c`/`pwm.h` — H-bridge + PWM output. `set_output()`
  is the raw driver; `main.c`'s movement functions wrap it with
  `set_output_ramped()` for acceleration ramping (see Conventions).
- `encoder.c`/`encoder.h` — quadrature tick counters; `get_encoder_diff()`
  handles 16-bit counter wraparound.
- `infrared.c`/`infrared.h` — ADC+DMA IR sensor reads.
- `uart.c`/`uart.h` — non-blocking DMA UART: queued TX (`print()`/
  `send_uart()`, up to 8 pending messages, `UART_TX_QUEUE_LEN`) and queued
  line-based RX (`uart_read_line()`, up to 4 pending commands,
  `UART_RX_QUEUE_LEN`).
- `gpio.c`/`gpio.h` — LEDs + buttons.
- `sysclock.c`, `msp.c`, `system_stm32f1xx.c`, `stm32f1xx_it.c` — clock
  config, HAL MSP init (DMA/GPIO wiring per peripheral), interrupt vectors.
- `stm32f1xx_hal_conf.h` — project-owned HAL feature-selection header
  (formerly `config.h`; see Conventions for why renaming alone wasn't enough).
- `error.c`/`error.h` — `Error_Handler()`.
- `test_uart.c`, `test_diag.c`, `test_maze.c`, `test_flood.c` — entry points
  for the 4 non-`main.c` PlatformIO environments above.
- `tools/robot_monitor.py`, `tools/dashboard.py` — Python/curses live
  monitor + command sender over the Bluetooth serial link.

## Live Bluetooth protocol (USART3, 9600 baud, `/dev/rfcomm0` on the PC side)
One command per line, case-insensitive:
`SPD n | TURN n | KP f | KD f | RESET | START | PAUSE | RESUME | DEBUG ON | DEBUG OFF`
- `RESET` — full `NVIC_SystemReset()`; the only thing that clears the maze
  map (`maze_init()` runs again from `main()`).
- `START` is rejected if a run is already active, or if `localization_valid`
  is false (previous run aborted or reached the goal). Physically re-place
  the robot at the true origin cell and send `RESET` before retrying — plain
  `START` does not resync position or clear the map.
- `KP`/`KD` parse decimals manually (`parse_gain()`); this toolchain's
  nano-libc `sscanf`/`print` do not support `%f`.

## Conventions / hard-won gotchas
- `stm32f1xx_hal_conf.h` requires `board_build.stm32cube.custom_config_header
  = yes` in `platformio.ini`, or the framework silently uses its own shadow
  copy instead (GCC's quote-include search checks the current file's
  directory before any `-I` path).
- `send_uart()`/`print()` silently DROP a message if the TX queue is full —
  intentional, so debug prints never block the control loop. Don't add
  blocking waits to the shared queue logic in `uart.c`.
- `cross_cell()`/`turn_left()`/`turn_right()` must only mutate
  `robot_position_x/y`/`robot_heading` when the move is encoder/IR-confirmed,
  never unconditionally after a timeout-bounded loop exits — an unconfirmed
  move must not touch position, or the maze map permanently desyncs from
  reality.
- `maze_observe_wall()` requires a symmetric confidence threshold (net
  `>= 2` observations) to both SET and CLEAR a wall. A single noisy IR
  reading must never weld a wall shut permanently — this was a real bug
  (fixed 2026-09-22): setting used to be instant/unconditional while clearing
  needed 2 confirmations, so one false reading could strand an entire run
  with a false "no path to goal".
- Movement speed is ramped (`set_output_ramped()`/`reset_speed_ramp()` in
  `main.c`, step rate `ACCEL_STEP_PER_MS`), not snapped straight to target.
  Stopping is deliberately left immediate/unramped — ramping the
  deceleration would risk overshooting the encoder/IR-confirmed stop point.
- `explore_run()` aborts (motors off, blink LEDs, `localization_valid = 0`)
  rather than looping forever if the flood fill reports the goal unreachable
  (`0xFFFF`) or a move times out — both mean the map or position can no
  longer be trusted.
- Globals are defined directly in a few headers (not `extern` + a separate
  `.c` definition) — only safe because the toolchain defaults to `-fcommon`;
  don't assume this is portable if the toolchain ever changes.

## Current status
Maze exploration + flood-fill navigation works end-to-end on the real
practice maze (verified on hardware). Acceleration ramping was just added
and is not yet retested on hardware. Not yet implemented: the speed-run
phase (re-run using the known map without re-exploring), arc/curved turns,
and full tuning on the real 16x16 competition maze.
