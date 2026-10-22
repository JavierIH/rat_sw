# Code review and work plan (2026-10-04)

An outside look at the whole project after OSHWDEM 2026, asked by the user
to be critical: is it a good project, how does it compare with professional
micromice, is it too complex. The firmware was read in full (~7,200 lines of
our own code), the host tests (~3,000) and tools (~3,700) skimmed. The plan at
the end is the work it suggests, in order of gain for the effort.

## Verdict

A software-engineering project well above the usual hobby robot; as a
competition micromouse it is far from the professional ones, mostly because
of the hardware; and it is more complex than it needs to be on several
fronts, partly to make up for slow sensors and partly self-inflicted.

## What is good

- Strategy and control in pure C with no HAL (planner, search, control,
  curves), run identically on the PC: 3,900+ host checks, simulated mazes
  (520 real competition ones among them), CI.
- A time-optimal planner over (cell, heading) states, and a search that does
  not trust the speed-run path until it has verified it is the optimum.
- Clear failure semantics: every move result says whether the pose is still
  valid.
- The map store: CRC-checked records in a log of six slots, erased only at
  boot.
- Telemetry, a live monitor with replay, a calibration recorder with
  analysis in Python.
- Constants documented with their reason and measurement; small commits;
  faults chased to their cause (the flash wedge).

## Compared with a professional classic micromouse

The software architecture is close: a 1 kHz loop with a motor-model
feedforward, trapezoidal profiles, wall centring, front alignment, a
verified search, the map in flash, smooth turns in the speed run. The
hardware and the performance are not:

| | Top classic mouse | rat_sw |
|---|---|---|
| MCU | STM32F4/G4/H7 with FPU | F103, no FPU, flash at 94 % |
| Heading | gyro | encoders + walls only |
| Distance sensors | pulsed IR read in microseconds | Sharp-type analog: a reading every ~16 ms, ~50 ms late |
| Grip | suction fan (the best ones) | none |
| Speeds | several m/s, turns at 1.5-3 m/s, diagonals | 0.9 m/s straights, curves at 0.3-0.48 m/s, no diagonals |
| Search | smooth turns on the move, under a minute | turns in place, 1.5-3 min simulated on a 16x16 |
| 16x16 speed run | ~5 s or less | ~15-25 s (estimate) |

Much of the code's complexity exists to make up for that hardware: the
position trail against the IR delay, the centring's predictor and observer,
"doubtful" sides against phantom walls, map repairs, empirical curve-slip
terms (proportional to v^2). Each piece is justified and measured, but
together they make a system calibrated to these exact sensors: changing one
means retuning a dozen constants (centre left 89 vs right 76, front offset
-15, ...).

## Too complex: where, and why

1. The competition firmware carries all the development tools: the
   calibration recorder (6.4 KB of RAM, 31 % of it, and 3.3 KB of flash),
   36 live tunables, STATUS register dumps, the ASCII map, ~50 console
   messages. Hence 94 % flash and 86 % RAM, and the workarounds that
   pressure breeds (lines cut at 119 characters, short-code messages
   considered).
2. Flash defences built on correlations during the freeze month: the 1 s
   wait with the motors off, waiting for the UART to go quiet, the probe at
   boot, the reactive restart, the "blocked" state, `RESET` refusing, the
   oscillator watch. The cause is now known and handled (HSI stopped but
   for flash operations, validated 10-05): most of them are now superseded.
3. The UART at 9600 baud: a queue that drops messages, waits of up to
   140 ms, background map rows, bursts. Setting the HC-05 to 115200 (one AT
   command, once) would remove that.
4. Two planners kept identical: the robot's and a Python port inside the
   monitor (the transcript tests enforce it). The robot could send its route
   and the monitor just draw it.
5. `motion.c` does everything (957 lines): the 1 kHz control loop, move
   supervision, search legs, front-wall tracking, emergency braking, turns,
   alignment, sensing, LEDs and the tunables; `run_path()` is ~165 lines.
6. Legacy and small slips: modes numbered 1-8 for the monitor's sake,
   unlike the menu; the parameter defaults written twice in `params.c`;
   stale comments (`infrared.c` "100 Hz control loop", it is 1 kHz; a
   doubled comment above `drive_to()` in `search.c`); the virtual firmware
   builds test code in (`#include "../test/host/sim.c"`).
7. Process weight: `AGENTS.md` ~355 lines mixing rules and history, ~2,000
   lines of docs.

## Real risks, most serious first

1. The robot never ran a 16x16 before OSHWDEM: the 4x3 practice maze and
   layouts A-I have no long straights nor staircases of curves. Race 2.4
   crashed there. A process gap, not a code one.
2. No gyro: the heading rests on walls and empirical compensations, fragile
   with another floor, other tyres or a low battery.
3. Software floating point in the 1 kHz interrupt; its load never measured
   (issue 15 c).
4. The flash wedge's electrical cause (VDDA unfiltered on the Blue Pill) is
   still there; the firmware now lives with it.
5. Three fast blinks mean four different things: a failed run, START
   refused, a failed save, a full log.

## Plan, by gain for the effort

| # | Item | Status |
|---|---|---|
| 1 | Test on a large stretch before the next competition: straights of 8-15 cells and staircases of curves, even if only half a maze | open |
| 2 | Delete the flash defences the stopped HSI supersedes (settle wait, UART-quiet wait, `RESET` refusal, ...), keeping the timing and the reactive restart | done 10-06: settle and UART waits, boot probe, `RESET` refusal and oscillator watch removed; timing, reactive restart and blocked state kept |
| 3 | Two builds: competition (no calibration recorder, tunables, register dumps) and development (all of it): ~5 KB of flash and 6.4 KB of RAM back | done 10-06: `DEV_TOOLS` (CAL recorder, STATUS registers) in `practice` and the new `dev`; `competition` gets 6.5 KB of RAM and 5.4 KB of flash back; `TUNE` kept in all |
| 4 | HC-05 at 115200 baud (AT+UART), then `UART_BAUDRATE` and the tools | open |
| 5 | Split `motion.c`: control loop, moves, search legs, sensing | declined 10-06: every part shares the move state (profiles, path run, guard, side pass, tunables); a split would turn ~20 file statics into globals behind an internal header, more coupling for 900 lines already in sections |
| 6 | Hardware, the user's call: a filter on VDDA (ferrite or 10-47 ohm + 1 uF + 100 nF), bulk and ceramic capacitors at the H-bridge and across the motors; a gyro, if the hardware ever may change, is the largest single performance lever | open |
| 7 | Distinct LED signals for a failed save / full log vs a failed run | done 10-06 (6f82f77): map not saved = halves alternating, START refused = LEDs 3-4 |
| 8 | The monitor draws the robot's route instead of recomputing it; trim `AGENTS.md` to rules | open |

## Review of 2026-10-22 (paused)

The user asked for a critical review of everything, from scratch; paused at
the user's request. Read in full: every firmware module in `src/`, the
control simulator, this file, `design.md`, `testing.md`,
`faults/centring.md`; only skimmed: the tools, `test_main.c`, `sim.c`.

### Done

- Issue #1, the wall-end kick: fixed in the simulator (d42d8f1,
  `docs/faults/centring.md` section 11). The host tests pass again (7
  checks failed since 0cfc91f, so CI was red). Waits for the robot.

### To do (found, small, no robot needed)

1. `motion.c`: `end=OBSTACULO` (the emergency brake) -> `OBSTACLE`, and
   add it to the translation list in `docs/README.md`.
2. README, console table: `START` launches after 5 s (`START_DELAY_MS`),
   not 2 (that is `CAL`).
3. Spanish left in comments: `commands.c` ("Consola Bluetooth"), `app.h`
   ("2.4 RAPIDA SEGURA"), `platformio.ini` and `test_virtual.c` ("TUNE
   RUEDAS 1").
4. `robot_config.h`: the empty "Flash store" section.
5. `params.c`: the defaults written twice (`DEFAULTS` and `params`); one
   initializer macro (item 6 of the 10-04 review, never done).
6. `commands.c`, `cmd_tune()`: its upper-casing loop is redundant
   (`commands_poll()` already upper-cases the line).
7. AGENTS.md memory figures: now competition RAM 52.7 % / flash 85.3 %,
   practice and dev RAM 84.4 % / flash 93.9 % (3.9 KB left).
8. `docs/img/robot_monitor.gif` is untracked: commit it (for the README?)
   or delete it.

### Still to review

`tools/robot_monitor.py` and `tools/calib_analyze.py` (structure only so
far), `test/host/sim.c`, most of `test_main.c`, `docs/lighting.md`,
`docs/mazes.md`, `docs/faults/freezes.md`, `docs/faults/oshwdem2026.md`.

### Questions and proposals (the user's call, or a measurement first)

1. Search turn cost: `SEARCH_COST_TURN` 1 (half a cell) as in the speed
   run, but in the search every turn is a stop, ~0.3 s in place and a
   start (~1.5 cells of time). Proposal: time 16x16 searches in the
   simulator with 2 and 3 before deciding; it changes the search's routes
   (the monitor reads the costs from `robot_config.h`).
2. Front at rest: when only one front sensor sees a wall,
   `motion_sense_walls()` records the front as open (evidence -1), against
   the rule "never turn a doubtful reading into absent". Make it doubtful
   (not recorded)? Rare (stopped at a centre both see it), and a move
   stops at the wall anyway.
3. Wall ends: in the simulator a short edge reading that comes before the
   step away still counts once (~2 deg); every robot recording stepped
   away first. If the robot shows it, a large step inwards that comes
   back could be held too (careful: crooked starts close in by 13-20 mm
   per sensor period).
4. SysTick worst 424 us after a search (10-08): no cause known. A flash
   halfword stall is ~56 us; find what takes the rest, not only read it
   after a race (issue 19).
5. `CURVE_PRE`, `CURVE_POST` (0) and `CURVE_ANGLE` (90) still sit at their
   neutral values (kept 09-30 for `calib_analyze.py`'s suggestions): keep
   them?
6. The flash stress tools of the virtual env (`TUNE STRESS`, `REVERSAL`,
   `HSI`): the wedge is fixed (10-05). Keep them to reproduce it, or
   delete them?
