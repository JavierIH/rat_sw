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
| 2 | Delete the flash defences the stopped HSI supersedes (settle wait, UART-quiet wait, `RESET` refusal, ...), keeping the timing and the reactive restart | open (the stopped HSI validated 10-05) |
| 3 | Two builds: competition (no calibration recorder, tunables, register dumps) and development (all of it): ~5 KB of flash and 6.4 KB of RAM back | open |
| 4 | HC-05 at 115200 baud (AT+UART), then `UART_BAUDRATE` and the tools | open |
| 5 | Split `motion.c`: control loop, moves, search legs, sensing | open |
| 6 | Hardware, the user's call: a filter on VDDA (ferrite or 10-47 ohm + 1 uF + 100 nF), bulk and ceramic capacitors at the H-bridge and across the motors; a gyro, if the hardware ever may change, is the largest single performance lever | open |
| 7 | Distinct LED signals for a failed save / full log vs a failed run | open |
| 8 | The monitor draws the robot's route instead of recomputing it; trim `AGENTS.md` to rules | open |
