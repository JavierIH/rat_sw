# Testing on the robot

What has been verified on the robot (and in the simulator), with numbers, and
how the 16x16 parts are tested without a 16x16. Open problems: docs/ISSUES.md;
the test layouts: docs/mazes.md.

## Validation history

### Status up to 2026-09-26
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

Search legs (straight on without stopping, the default): on the robot on
the practice maze and on layouts B and C (the 4x3 rearranged; goal (3,2)):
maps right; layout B 7.4 s against 8.5 s stopping in every cell, its speed
runs 1.48-1.69 s and clean, IR stops within 3.5 mm of the plan. 1-cell
legs left some sides doubtful (fixed by `SEARCH_SIDE_FROM_MM` 20, not yet
on the robot). Still to validate: long straights.

Layout C (2026-09-26; route 1R1L1R2, a staircase of three curves in
consecutive cells, then 2 cells along the north border): the speed run
drifted onto the north wall on the last straight, scraped it, and wedged
turning at the goal. Reproduced in step mode (IR at the goal): it arrived
yawed 5.5 deg and 24 mm off-centre. Cause: the start was 8-12 mm off-centre
(where the search's final 180 turn left it) and the centring's integral
learned that as ~5 deg of bias in the first cell (the simulator reproduces
3.3 deg for 10 mm), then aimed the last straight at the wall. With
`TUNE BIAS_WIN 0` (no integral) the same run arrived square, 17 mm off.
Fixed by the bias observer (not yet on the robot). Also measured: single
curves at 478 mm/s turn ~2.4 deg less than the encoders say (4.9 for a
right plus a left; single measurements scatter +-3 deg), and one exited
12 mm wide; at 300 mm/s they matched. Not compensated yet: re-measure with
`CAL RUN` on the staircase once the observer is on the robot.

Layout D (2026-09-26; route N 2 cells, E 3 to the goal): two searches with
`CONT ON` at 450 mm/s made the same 13 actions and the same map; every leg
(2 N, 3 E, 3 W, 2 S) decided its next cell in time and ended on the front
wall 0.3-0.5 mm short of its target, tracking error <= 1.7 mm / 2.7 deg.

## Getting ready for the 16x16 without a 16x16

The robot has only driven the 4x3 practice maze. What changes in a 16x16,
and how each part is tested without one (issue 11).

### What is the same

The firmware is the same code: the map, the planner and the flash record
are sized 16x16 in every build (`PRACTICE_MAZE` only changes the goal), and
the Bluetooth traffic per cell is the same (a whole 16x16 cycle is ~14 KB of
telemetry and log, spread over minutes). The strategy is tested in the
simulator: `make -C test/host` runs 600 random 16x16 mazes (search 47-111 s
simulated, every one verified optimal with perfect sensing).

Real competition mazes: `host_tests --mazefile` over the 520 classic
16x16 of github.com/micromouseonline/mazefiles (APEC, All Japan, UK...):
every search reaches the goal and gets home, every speed run is clean, no
crash, with perfect sensing and with 3 % noise; searches 181 s on average
(simulated). 3 of 520 end on `OPTIMIZE_MAX_STEPS` (400) short of the
optimum: japan2000 170 vs 168, alljapan-030-2009-exp-fin 163 vs 161,
uk2026-minos-classic 228 vs 214 (+6.5 %); 800 reaches it in all three, ~1
min more of search in them (the other 517 finish under 400). Set to 800
(09-27, next flash).

### What changes, and the test for each

1. **Decision time in the search legs** (CPU). Each cell decided on the way
   must be ready within `SEARCH_LATE_MARGIN_MM` (8 mm, 17.8 ms at 450). The
   worst 16x16 decision pops 3811 planner states, the practice maze's 3072
   (`host_tests --timing`), so a practice search measures it: the robot
   prints `decisions on the way: N, worst X us (P pops), late L` after
   the run. Cost a pop = X / P. Real mazes are worse than random ones: the
   worst decision in the 520 pops 5144. `tools/plan_cycles.py` (emulated
   M3, a cycle model) gave 235 cycles a pop, ~17 ms for 5144: no margin.
   The planner's loop was rewritten (same results: transcripts and the 520
   mazes identical): 133 cycles a pop, ~9.5 ms + the rest of the decision.
   If the robot still reports late ones: `TUNE LATE_MARGIN`.
   Measured (09-27): 1024 pops 4493 us (layout E), 3072 pops 13001 us
   (layout G, the OPTIM -> RETURN decision: 3 plans), so 4.15 us a pop
   and ~240 us fixed: 2.25x the emulator's model (the 1 ms control
   interrupt takes a large share of the CPU). The 16x16 worst: ~21.6 ms,
   ~4 ms late at 450: the reference has begun braking and the leg dips
   ~12 mm/s before the path grows again. Harmless; kept.

2. **Long straights: the distance scale and the drift.** A 16x16 has
   straights of up to 15 cells (2.7 m); the speed run places each curve by
   the encoders alone (no correction on posts), so a 1 % scale error puts
   the curve 27 mm off. Needed: under 0.3 % (8 mm in 2.7 m). Test on a flat
   floor with the same grip as the maze, no walls, 3 m clear:
   `CAL STRAIGHT 15 300`, then 450, 600, 800 (never higher before the lower
   ones are clean). Measure with a tape the distance travelled (start mark
   to the same point of the robot) and the sideways offset at the end.
   Scale: `TUNE TICKS_MM` (9.05 now; travelled short -> lower it). Drift:
   `TUNE WHEEL_DIFF` (~1011 x WHEEL_DIFF deg per metre). Write the result
   into `robot_config.h`.

3. **Long straight then a curve** (needs walls). With spare wall pieces
   outside the 4x3 board: a corridor as long as they allow, ending in an L.
   Without them: the longest the board gives, 4 cells along a row then a
   curve.

4. **The 2x2 goal, open inside, one entrance** (layout H, `docs/mazes.md`).
   On the 4x3: a layout with a 2x2 block without its inner walls (and without its centre post, if it
   comes out), one way in, `GOAL x0 y0 x1 y1` on it. Tests sensing and
   centring with no side walls, the 4-cell goal and the speed run into it.
   Done 09-27 (layout H, `GOAL 2 1 3 2`, centre post in): the search
   reached (2,2) after 5 actions, verified the path from there without
   entering the block and was back in 11 (8 decisions on the way, worst
   4474 us, none late). Races 2.4 and 2.5 there and back: `2R2` in one leg,
   the stop in (2,2) on the encoders with only the north border in view
   (err 2.7/2.8 mm, 4.0/3.0 deg; the 180 turn absorbs it), the return
   `end=IR`, err <= 2.3 mm / 3.6 deg. No "!!" except "flash full" (no
   power cycle after `ERASE`: the map stayed in RAM). The speed run stops
   in the goal's first cell, so the open interior is only seen, never
   driven, as it will be in the 16x16.

5. **Endurance.** A 16x16 session drives 3-4 min (search, speed runs,
   returns). On the practice maze: a search, then races 2.4 and 2.5 back to
   back for 5 min without touching the robot: "!!" lines, tracking error
   at the stops, the pace of the runs as the battery drops.
   Done 09-27 on G (12:54-13:04, a 4 min pause in the middle): a search
   (23 actions, 14 decisions on the way, none late) and 20 races alternating
   2.4/2.5, all `End: OK`, every stop `end=IR`, err <= 2.8 mm / 3.5 deg, no
   stall or clock line, stack 1316 B never used. No pace drop: 2.5 to the
   goal 2.58-2.87 s, 2.4 3.14-3.38 s, the first and the last alike. Found:
   selecting a race sets FAST/CURVE, saved in the map's record, so each
   switch of race wrote a record: the six slots were gone after three
   races ("!! flash full", harmless: the map stays in RAM). Fixed in
   a6d42d2 (not flashed yet): the run's end writes only a changed map.
   Until then, one race preset per power-on (the boot compacts). One flash
   wedge on those writes, cleared by the HSI restart (`docs/faults/freezes.md`,
   n=3).

### Results (09-27, issue 11)

From the issue's entry, as it stood:

Flashed 09-27. Decision time measured: 4.15 us a
pop (1024 pops 4493 us on E, 3072 13001 us on G), so the 16x16 worst
(5144) ~21.6 ms vs 17.8: ~4 ms late, which only dips the leg ~12 mm/s
(the path grows while braking); kept. Test 5 endurance done 09-27 on G:
search + 20 races 2.4/2.5 clean, no pace drop. Flashed again 09-27
(a6d42d2 map-only saves, OPTIM 800, KI 16): test 3 on the ring (layout
I) done: search 13 actions, 0 "!!", worst decision 4473 us; races 2.4,
2.5, 2.4 route `2R3` in one leg both ways, `end=IR`, err <= 3.8 mm /
2.9 deg. The first two saved (the goal's walls' evidence +1 -> +3), the
third did not despite the preset switch (test_races_settle_map). Test 4
(goal block, layout H) done: search 11 actions, races 2.4 and 2.5 clean
(`2R2` into (2,2), `end=ENC`, err <= 2.8 mm / 4.0 deg). Test 2 on the
room floor along a baseboard (SR, 2 x `CAL STRAIGHT 10 300`): this floor
reflects into FR (~70 mm always) and SL (40-60 in patches), a phantom
left wall that swayed the centring; to the baseboard alone -8..+16 mm.
The user wants oscillations checked, not the distance. Robot left on
defaults, goal (3,2), map erased. Next, if repeated: baseboard on the
left (SL on it; SR stayed clean there), 300/450/600/800.
09-27: the default build is now the 16x16 one (goal 7 7 8 8; 4x3 =
`pio run -e practice`), and `ERASE` / mode 3 restore the build's goal.
Flashed 09-27 (50b4be9), checked on the desk: boot goal (7,7)-(8,8);
`GOAL 3 2` + `SAVE`, `ERASE` -> (7,7)-(8,8), still so after `RESET`.
To practise on the 4x3 with it: `GOAL 3 2`, then `ERASE` when done.
Next flash: defaults KP 0.5 KI 20 (validated on races 2.5/2.6 09-27;
until then they are in the saved record, `SAVE` after setting them),
and the options removed on 09-30 (issue 16: no test needed).
The 149 ms decision on the way (goal unreachable, log
2026-09-27_19-11-58) is the map repair inside a leg: explore_next ->
plan_explore -> telemetry_map(), whose uart_wait_space() waits ~140 ms
for the 9600-baud UART (the 2001 pops are ~8 ms). Fix: issue 15 (a).

### Competition day checklist

- Goal: the default build is the competition one (goal 7 7 8 8; the 4x3 is
  `pio run -e practice`). A flash with another default goal ignores the
  saved record (the map and parameters go).
- `ERASE` (or mode 3) before the first search in the competition maze: it
  also puts the goal back to 7 7 8 8 (after any `GOAL` used to practise).
  `STATUS` must show `goal (7,7)-(8,8)`.
- Battery full; `STATUS`: no "!!", stack margin, flash slots free.
- `CHECK` in the start cell facing north, under the competition's light:
  `light: OK`, or read `docs/lighting.md` (report only until the
  correction is in). A reflective floor reads as walls too (a room floor
  gave FR ~70 and SL 40-60 in patches, a phantom wall the centring
  follows).
- `CONT ON` (default). Mode 1, then race 2.4 (FAST 800 CURVE 300) and only
  with it clean 2.5 (FAST 900 CURVE 400); 2.6 (CURVE 480) is not safe
  (issue 12). Race 2.3 (no curves) if the curves fail.
- After an abort: the robot at the start facing north, `HOME`, `START` (or
  START on the button).

## IR delay tests

The firmware pairs the front readings with where the robot was
`IR_DELAY_MS` (50 ms) earlier (the wall tracking at the end of straights,
the search legs' front decisions, `SEARCH_LEG_SPEED_MAX`); the centring
assumes nothing, and the simulator gives the side IR 8 ms
(`SIDE_IR_DELAY_MS`). The only measurement behind the 50 is a front wall
approached at 400 mm/s (docs/measurements.md). `tools/calib_analyze.py
--delay` measures every recording at hand (all of them up to 10-05):

| Method | FL | FR | SL | SR |
|---|---|---|---|---|
| in-place turns (median, quartiles) | 4 (0-8) | 8 (4-12) | 4 (0-8) | 4 (0-4) ms |
| wall approaches (median, quartiles) | 18 (10-26) | 34 (28-40) | - | - ms |

- In-place turns: each reading against the encoder angle over the first 25
  deg, delayed d; only readings that really move with the angle count (at
  least 0.5 mm/deg). No translation, so no calibration offset or slide can
  mimic a delay.
- Wall approaches: the front readings against the true distance (the final
  reading plus the encoder distance still to go), delayed d. A scale error
  of the calibration, or the robot sliding with its wheels stopped (which
  the encoders do not see), also shows up as delay here. The motors
  short-brake at 0 (TB6612FNG), and coasting is counted by the encoders.

On the robot (no flash needed; short batches for the battery):

1. **Turns** (all four sensors). In a cell closed on three sides, centred
   front to back and side to side, facing the open side: `CAL TURN 1`,
   `CAL TURN -1`, five each (recorded every 4 ms). The fronts need a wall
   within ~200 mm during the first 25 deg, the sides one within 130 mm.
   Expected: 4-8 ms for all four.
2. **Approaches** (front sensors). Layout I, row 0, from a corner cell:
   `CAL STRAIGHT 3 <v>` onto the end wall at 100, 300, 600 and 900 mm/s, two
   runs each. A real delay keeps the same ms at every speed; an offset or a
   slide keeps the same mm, so its ms falls as 1/speed.
3. **Slide** (only if 2 points at one): the 600 mm/s runs again with `ACCEL
   2000`; a slide shrinks with gentler braking, a delay does not.
4. Then `python3 tools/calib_analyze.py --delay tools/calib_data/<new>*.csv`.

If the front delay is not 50: try it live with `TUNE IR_DELAY <ms>` and
check the stops on end walls (`end=IR`, FL/FR ~85/101 squared, err a few
mm) at 300-900 before writing it into `robot_config.h`; the search legs'
speed cap (`SEARCH_LEG_SPEED_MAX`, set from the 50) can then be revisited.
