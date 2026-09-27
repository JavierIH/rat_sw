# Getting ready for the 16x16 without a 16x16

The robot has only driven the 4x3 practice maze. What changes in a 16x16,
and how each part is tested without one (issue 11).

## What is the same

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

## What changes, and the test for each

1. **Decision time in the search legs** (CPU). Each cell decided on the way
   must be ready within `SEARCH_LATE_MARGIN_MM` (8 mm, 17.8 ms at 450). The
   worst 16x16 decision pops 3811 planner states, the practice maze's 3072
   (`host_tests --timing`), so a practice search measures it: the robot
   prints `decisiones en marcha: N, la peor X us (P pops), tarde L` after
   the run. Cost a pop = X / P. Real mazes are worse than random ones: the
   worst decision in the 520 pops 5144. `tools/plan_cycles.py` (emulated
   M3, a cycle model) gave 235 cycles a pop, ~17 ms for 5144: no margin.
   The planner's loop was rewritten (same results: transcripts and the 520
   mazes identical): 133 cycles a pop, ~9.5 ms + the rest of the decision.
   If the robot still reports late ones: `TUNE LATE_MARGIN`.
   Measured (09-27): 1024 pops 4493 us (layout E), 3072 pops 13001 us
   (layout G, the OPTIM -> VUELTA decision: 3 plans), so 4.15 us a pop
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
   4474 us, none late). Races 2.4 and 2.5 there and back: `2D2` in one leg,
   the stop in (2,2) on the encoders with only the north border in view
   (err 2.7/2.8 mm, 4.0/3.0 deg; the 180 turn absorbs it), the return
   `fin=IR`, err <= 2.3 mm / 3.6 deg. No "!!" except "flash sin hueco" (no
   power cycle after `ERASE`: the map stayed in RAM). The speed run stops
   in the goal's first cell, so the open interior is only seen, never
   driven, as it will be in the 16x16.

5. **Endurance.** A 16x16 session drives 3-4 min (search, speed runs,
   returns). On the practice maze: a search, then races 2.4 and 2.5 back to
   back for 5 min without touching the robot: "!!" lines, tracking error
   at the stops, the pace of the runs as the battery drops.
   Done 09-27 on G (12:54-13:04, a 4 min pause in the middle): a search
   (23 actions, 14 decisions on the way, none late) and 20 races alternating
   2.4/2.5, all `Fin: OK`, every stop `fin=IR`, err <= 2.8 mm / 3.5 deg, no
   stall or clock line, stack 1316 B never used. No pace drop: 2.5 to the
   goal 2.58-2.87 s, 2.4 3.14-3.38 s, the first and the last alike. Found:
   selecting a race sets FAST/CURVE, saved in the map's record, so each
   switch of race wrote a record: the six slots were gone after three
   races ("!! flash sin hueco", harmless: the map stays in RAM). Fixed in
   a6d42d2 (not flashed yet): the run's end writes only a changed map.
   Until then, one race preset per power-on (the boot compacts). One flash
   wedge on those writes, cleared by the HSI restart (`docs/freezes.md`,
   n=3).

## Competition day checklist

- Goal: a build without `-DPRACTICE_MAZE=1` (default goal 7 7 8 8), or send
  `GOAL 7 7 8 8` + `SAVE` on the practice build. A flash with another
  default goal ignores the saved record (the map and parameters go).
- `ERASE` (or mode 3) before the first search in the competition maze.
- Battery full; `STATUS`: no "!!", stack margin, flash slots free.
- `IR` at the start with nothing around the nose: FL/FR ~230 and SL/SR
  far. A reflective floor reads as walls (a room floor gave FR ~70 and SL
  40-60 in patches, a phantom wall the centring follows).
- Flash firmware older than a6d42d2: each switch between races 2.4/2.5/2.6
  spends a flash slot (six; `STATUS` shows them free). Keep one preset per
  power-on, or power-cycle (the boot compacts) before switching.
- `CONT ON` (default). Mode 1, then race 2.4 (FAST 800 CURVE 300) and only
  with it clean 2.5 (FAST 900 CURVE 400); 2.6 (CURVE 480) is not safe
  (issue 12). Race 2.3 (no curves) if the curves fail.
- After an abort: the robot at the start facing north, `HOME`, `START` (or
  START on the button).
