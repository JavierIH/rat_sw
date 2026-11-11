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

Items 1-6 done 11-11 (flash -24 B: `competition` 53984 B, `practice`
59428 B); 7 done 11-11; 8 is the user's call.

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
   Measured 11-11 (audit, step 2): no gain, stays 1.
2. Front at rest: when only one front sensor sees a wall,
   `motion_sense_walls()` records the front as open (evidence -1), against
   the rule "never turn a doubtful reading into absent". Make it doubtful
   (not recorded)? Rare (stopped at a centre both see it), and a move
   stops at the wall anyway.
   Done 11-11: doubtful, not recorded (`motion_sense_walls()`,
   `sense_here()`); the robot still drives on and the move decides. It
   matters more since races only read the map: two such readings made a
   wall a verified passage. `sim_front_doubt()`, `test_front_doubt`
   (the evidence stays 0; 30 searches with 30 % doubtful fronts all
   right). Flash +20 B. Robot: `WALLS` yawed by a wall shows `front=?`.
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

## Audit of 2026-11-11 (in progress)

The user asked for an audit of `src/` and `tools/robot_monitor.py`: bugs,
simplification and efficiency, ranked, each with its evidence; nothing is
fixed until the user picks. It goes in small steps, each written here
before stopping (the user's rule, 11-11). Line numbers are those of
45075fe.

Baselines (11-11): `make -C test/host` 3939 checks, 0 failures.
`--mazefile` on a fresh clone of the maze files: 523 of 525 classic 16x16
load (520 on 09-27); search 182.1 s on average (195.2 with 3 % noise),
every one optimal with perfect sensing, no blocked move, no crash, worst
decision 5144 pops. Sizes: `practice` RAM 84.4 % (17288 B), flash 93.9 %
(59596 B, 3892 left); `competition` RAM 52.7 % (10800 B), flash 85.3 %
(54156 B, 9332 left).

### Step 1 (11-11): everything read once; the races write the map

Read in full: every module of `src/` but the vendor files, `sim.c`,
`robot_monitor.py`. Not yet: `calib_analyze.py`, `control_sim.c`, most of
`test_main.c`. "Measured" below: reproduced in the simulator or found in a
robot log; "read": from the code alone.

**A1. A race writes walls into the map from a pose it cannot check (bug,
high; measured).**
- Where: `search.c:148-151` (`run_route()`: a route that ends
  `MOVE_BLOCKED` marks a wall, +2 even over a crossed passage at -3),
  `search.c:472-481` (`drive_to()` then repairs the map and explores),
  `search.c:225` (the repair's second pass forgets every wall of the map),
  `search.c:506` (the race saves that map at the goal).
- Why it is wrong: the route's passages are verified (crossed, or seen
  open twice). A wall "found" on one means the robot is not where it
  thinks, or is yawed. The simulator never blocks a race with the pose
  right: 0 in 1046 cycles on the real mazes (half of them with 3 %
  noise), 0 in the 900 random ones.
- On the robot, 2026-09-26 19:31 (practice maze, a race's return;
  `tools/logs/2026-09-26_11-22-09.log`, untracked, t=29370-29375):
  `ruta 9 celdas, 0 curvas: fin=OBSTACULO dist=168.78/1433.59mm ...
  IR(FL=57 FR=113)`, `obstaculo delante: marcha atras OK`, `!! destino
  inalcanzable segun el mapa: olvido 1 paredes dudosas (reparacion 1/3)`,
  then step by step "sin camino verificado", a second repair, `Fin:
  FALLO`. Right after the 180 at the goal it braked for something close
  ahead on the left (most likely yawed), 169 mm into a passage it had
  driven seconds before, and the wall went onto that passage. A `RESET`
  reloaded the map (cost 24).
- In the simulator (scratch harness over `sim.c`, not in the repo: after
  a perfect search one `motion_run_path()` call returns `MOVE_BLOCKED`
  with nothing there; 523 real mazes). On the leg out of the start: the
  race still ends OK in all 523, but the map goes from 172 known walls
  (average) to 1, and that map is saved at the goal: the flash copy keeps
  0 walls in 523 of 523. On the leg out of the goal: the RAM map loses
  over half its walls in 81 mazes and keeps a wall that does not exist in
  37 (the flash copy, saved just before at the goal, is right).
- It fits issue 17's open question (why no race would start after the
  2.4 crash at OSHWDEM): that firmware (6e19649) had this same code, and
  a START without a verified path only printed to the console. A stop at
  a wall while off the route, the wall marked on the verified path, the
  run ending `IND_FAIL`, no verified path left in RAM. No log of that
  day: it cannot be proved.

**A2. Races spend flash slots to save nothing (robustness, medium;
measured).**
- Where: `search.c:451` (`sense_here()` at every stop of a race),
  `search.c:138` (crossings), `search.c:506` (the save at the goal).
- Evidence (scratch harness, 523 real mazes, perfect sensing; a store as
  the boot leaves it, `ERASE`, a search, six races): after the search 2
  slots are free. Race 1 writes one in 286 mazes, race 2 in 369, race 3
  in 25, none after: the evidence of the walls at the start and the goal
  saturating, and passages going from seen twice to crossed. In 23 of 523
  the third race ends `!! flash full: the map is only in RAM` with the LED
  halves alternating (no reset, no power cycle), over a flash copy that
  races exactly the same.
- Each of those saves is also a flash write right after braking at the
  goal.

A1 and A2 have one cause: a race writes the map. Two ways out:

- a. Races only read the map (recommended). No sighting, crossing or wall
  recorded, no repair, no exploring, no save: nothing changes. A route
  that ends `MOVE_BLOCKED`, or a wall in front before a leg, ends the run
  as position lost (`RUN_FAILED`, the map untouched). `motion.c` does not
  change: it still stops short of the wall. Deletes `drive_to()`'s
  exploring branch and the race's save; `test_fast_run_surprise_wall`
  turns into "the run fails, map and flash unchanged, the next race
  starts". Risk: a passage on the route verified by two wrong sightings
  and never crossed fails every race at that wall until a new search
  (none in the 1946 simulated cycles above). It changes two of the user's
  rules (a race "saves at the goal if the map changed"; it "falls back to
  exploring if the map proves wrong"): the user's call.
- b. Keep the healing: in a race record only a `MOVE_BLOCKED`, never
  repair, and put the flash copy's map back when the race does not end
  OK. More code (a map-only reload in `storage.c`), and a lost robot that
  gets to "its" goal still saves a wrong map.

Done 11-11, option a (the user's choice), c41d383: `run_route()` moves
only the pose, `drive_to()` lost its exploring branch and its budget,
`search_fast_run()` its save. Same harnesses, before -> after: a blocked
stop with nothing there on the leg out of the start, the race OK with the
map wiped and saved in 523 -> the race ends there, RAM and flash maps
untouched in 523; on the leg out of the goal, over half the walls lost in
81 and a wrong wall kept in 37 -> 0 and 0; slots written by races 1/2/3:
286/369/25 mazes -> 0; "flash full" at the third race: 23 -> 0. One
addition to what was written above: a wall in front of the route at rest
ends the run only when seen twice. Ending at the first sighting cost 2-4
races in 100 in the noise suites (fast ok 97/96/98 at 1 %, 3 % and 3 %
with the competition goal); looking again keeps them at 100. Host tests
3911 checks, 0 failures (`test_fast_run_surprise_wall` rewritten,
`test_race_false_block` and `test_races_only_read` new; `sim_false_block()`
in the simulator, 41738c9), tools 35 OK, the 523 real mazes as before.
Flash: `practice` 59596 -> 59432 B, `competition` 54156 -> 53984 B; RAM
the same. The monitor's exploring overlay for races went too (89aaaf1).
It changes the robot's behaviour in races: to validate on the robot
(`docs/testing.md`, next session, step 3). The race no longer blinks at
the goal: it turns round at once.

**A3. A leg that never left its cell clears `turned` (bug, low; read).**
- Where: `search.c:353` (`drive_leg()`: `turned = 0` whatever `entered`);
  `forward()` (`search.c:117`) and `run_route()` (`search.c:143`) clear it
  only when the robot moved on.
- Scenario: a turn in place, a leg, an obstacle stop and back-up in the
  same cell (`entered` 0). The next `sense_here()` records the sides read
  at rest as walls, against the rule that right after a turn only "no
  wall" counts (5 phantoms in 14).
- Fix: `if(entered) turned = 0;`. Risk: none seen. The simulator's rest
  readings have no phantoms, so only a unit test of the flag covers it.
  No robot test needed.
- Done 11-11: `if(entered) turned = 0;`. `sim_false_block()` now also
  blocks a search leg where it starts; `test_blocked_leg_keeps_turned`
  (a turn, a leg blocked in its cell, the sides at rest: the wall that
  was in front keeps its evidence 2; without the fix it went to 3). Host
  tests 3916 checks, 0 failures; sizes unchanged.

**A4. The side walls read "on the way in" are one sensor reading
(robustness, low; read).**
- Where: `motion.c:530-534`: `side_pass` takes its 5 samples in 5
  consecutive ms; the IR give a new value every ~16 ms, so the 4-of-5 vote
  is one reading. The search's legs do it right (`explore_step()`: one
  every `WALL_SAMPLE_MS` over 50 mm, unanimous).
- It only feeds the moves that are not search legs: `CONT OFF` and the
  followers' single cells (races do not use the sides since c41d383).
- Fix: sample every `WALL_SAMPLE_MS` there too. Measure first on the `CAL
  STRAIGHT` recordings how often 5 ms and 32 ms disagree. Needs a robot
  check with `CONT OFF`.
- Measured 11-11 on the `CAL STRAIGHT` recordings (102 at 8 ms a sample
  that ended OK, 204 sides; scratch script): the first reading past the
  point against 4 of 5 readings 8 ms apart from it: 0 verdicts differ, 1
  side with mixed votes (by the 140 mm threshold). So no wrong wall is
  expected from either; the fix only makes the vote what it says.
- Done 11-11: one sample every `WALL_SAMPLE_MS` (a countdown in
  `side_pass`): 32 ms, 19 mm at 600 mm/s, inside the window the legs
  read (110 to 60 mm before the centre). RAM +4 B, flash +24 B
  (`competition` 54008 B, `practice` 59460 B). The host tests do not run
  `motion.c`: robot check in `docs/testing.md`, next session, step 4.

### Step 2 (11-11): efficiency, the planner's costs

Nothing to change: both measurements say the costs are where they
should be.

- `SEARCH_COST_TURN` (question 1 of 10-22), the whole cycle on the 523
  real mazes, perfect sensing / 3 % noise, average search: 1 (now)
  182.1 / 195.2 s; 2: 182.3 / 195.0; 3: 182.5 / 194.6; 4: 183.0 / 194.8.
  Within 0.5 %: a dearer turn saves a stop per search and pays it in
  cells (129 -> 128 stops, 273 -> 277 cells). Every perfect search
  optimal with all four. Stays 1; question 1 closed.
- `FAST_COST_*`, `--costs` on 600 random mazes: 2/1 is the best pair at
  900/400 and 900/480 (the best route in 198 of 200 on the open mazes).
  At 800/300 2/2 wins by 3 ms in 6.4 s: nothing. Stays 2/1. Race 2.3
  (turns in place) plans with the same costs and `--costs` does not time
  it: not looked at, it is the fallback.
- The SysTick load cannot be measured off the robot: `STATUS` after a
  race is in the next robot session (`docs/testing.md`, step 2), the
  424 us peak in the notes below.
- Found on the way: one search failed under 3 % noise (`shiba.txt` with
  cost 2), "target unreachable" after the three repairs. See A5.

**A5. The map repair loops (bug, medium; measured, fixed 11-11).**
- Where: `plan_explore()` (`search.c`): every repair first forgot only
  the walls seen once.
- Why it is wrong: a phantom wall seen twice seals the goal; forgetting
  the walls seen once opens a way through real walls, the robot sees
  them again in passing (once), the goal is sealed again, and the next
  repair does the same. The phantom stays; the third repair ends the
  search (`euro-58.txt`: the same dead end at (11,8) every time; with 10
  repairs allowed it looped 10 times).
- Measured, 3 % noise: 4 searches of 4184 failed on the real mazes
  (eight passes with other seeds), 14 of 6000 on random 16x16 with the
  competition goal. No such failure without noise.
- Fix: repair n forgets the walls with evidence up to n (seen once,
  twice, any). 0 of 4184 and 0 of 6000; average search time the same
  (193.6-196.0 s against 193.6-195.8). `test_repair_escalates` (three of
  the seeds that failed). Host tests 3922 checks, 0 failures; flash +4 B.
- Left as it was: at 3 % noise 1 race in ~3000 stops at a wall on a
  passage two wrong sightings verified (the risk named in A1's option a),
  and 7 in 6000 end at the start for a wall "seen" twice in front. Both
  end the race with the map untouched.

### Noted, not verified yet

- `explore_next()` counts an action the path then refuses to grow
  (`search.c`, `motion.c`'s `explore_step()`): one action too many in a
  budget of 2000 that a 16x16 search uses a quarter of. Left as it is
  (11-11).
- `leds_flash()` blocks up to 1.2 s in `HAL_Delay()` without polling STOP
  (`gpio.c`): only at rest, and the press is kept by the debounce in
  SysTick and taken at the next checkpoint. Left as it is (11-11).
- `curve_setup()` runs before every forward move (65 Simpson steps in
  soft float, at rest): ~2.1 ms on the emulated M3 (`plan_cycles.py`'s
  model, 152000 cycles), as long as one plan. 0.3 s in a whole search.
  Left as it is (11-11): caching it would add state for `TUNE`.
- The unexplained SysTick peak (424 us, question 4 of 10-22): the CAL
  recorder halves its buffer inside SysTick (`calib.c:96`, 160 struct
  copies), and `STATUS`'s worst is since boot, CAL tests included. Check:
  `STATUS` after a search with no CAL test since boot.
- `CHECK`: `light_check()` alone is 1116 B of flash in `competition`
  (issue 21, waiting on the user).
- Spanish left beyond the 10-22 list (the monitor's two replay
  messages and a docstring, `plan_cycles.py`'s output, `docs/mazes.md`)
  and `save_map()`'s comment ("3 fast = only in RAM"): fixed 11-11.
- The wall followers (races 2.1, 2.2) write the RAM map as they go and
  never save: what they record is what a search would (the same legs),
  and a later search or `SAVE` stores it. By design; nothing to change.
- Race 2.3 (turns in place) plans with `FAST_COST_TURN` 1 although its
  turns are stops. Measured 11-11 (300 random mazes with loops, there
  and back, everything planned with turn cost 1 / 2 / 3 / 4 / 6): race
  2.3 48.16 / 48.05 / 48.04 / 48.04 / 48.04 s, race 2.4 19.71 / 19.78 /
  19.79 / 19.81 / 19.81 s. 0.2 % to gain: stays 1.

### Next steps

A3 and A4 done 11-11 (A4 waits for its robot check).

Step 2 done 11-11 (above).

Second pass over `commands.c` and `main.c` done 11-11: nothing found
(`ERASE` with a full store compacts first; `PAUSE` with no run is
cleared at the next start). The Python tools: `pyflakes` clean, 35 tests
OK; `robot_monitor.py` and `calib_analyze.py` not read line by line.

3. Left: the SysTick peak on the robot, `CHECK`'s size (issue 21), a
   full read of `robot_monitor.py` and `calib_analyze.py`.
