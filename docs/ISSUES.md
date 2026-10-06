# Open issues

One entry per issue: state and next step, one or two lines each. Update the
entry before ending a session; move it to "Closed" with its result and
commit when done. Details go in the docs it points to, not here.

## Open

- GitHub issue #1, centring (`docs/faults/centring.md`): the controller is
  now the classic PD wall follower on the turn rate (KP 8, KD 0.6; side IR
  4-8 ms measured), better in the simulator in every case; merged into
  `develop` 10-06 (with the repo in English, one-line comments and the
  reorganised docs), not flashed. Next: flash the `practice` build (`CAL`
  is now only in `practice` and `dev`); crooked starts 30-35 deg at
  100-900, 180s at 600-900, `CAL CURVE`, races 2.4/2.5.

- Issue 20, the IR delay (10-06; `docs/testing.md`, IR delay tests): the
  front readings are paired with the position 50 ms back, but every
  recording says 4-8 ms on in-place turns (all four sensors) and FL ~18 /
  FR ~34 ms approaching walls. Next: the robot tests written there (turns,
  approaches at 100-900, `ACCEL 2000`), analysed with `calib_analyze.py
  --delay`; then `TUNE IR_DELAY` and the stops on end walls.

- Issue 17, OSHWDEM 2026 (2026-10-03; `docs/faults/oshwdem2026.md`): the map
  loss is solved (saves at the goal, the flash wedge fixed: issue 18,
  closed), flashed 10-05. Left: the race 2.4 crash well into the maze, and
  why no race would start after it (a wall marked on the verified path?);
  the user will bring them up. A smoke test of the competition firmware in
  the air needs ~30 cm clear around the sensors.

- Issue 21, lighting check for competitions (10-06; `docs/lighting.md`): the
  IR have no ambient subtraction; ~200 counts of extra light make an open
  right side a phantom wall. Proposed: four quarter turns in the start
  cell, per-sensor raw offsets measured, applied and saved, LED verdict.
  `CHECK` (report only) is in, tested on the PC against a synthetic cell.
  Next: on the robot, validation 1-2 with `CHECK`; the user picks the
  trigger (4th menu entry recommended); then the correction.

- Issue 19, work plan of the review (`docs/review.md`): done 10-06 items 2
  (flash defences removed), 3 (`DEV_TOOLS`: competition build lean) and 7
  (LEDs); 5 (split `motion.c`) declined. Left: 1 (large stretch), 4
  (HC-05 at 115200), 6 (hardware, the user's call), 8 (monitor draws the
  robot's route). Ideas from issue 15: a SysTick stop if the following
  error runs away while the main loop blocks; the SysTick load: `STATUS`
  shows the slowest tick since 10-06, read it after a race (the return's
  curves are capped at 300 since 10-06).

- Issue 11, 16x16 competition readiness (`docs/testing.md`): tests 2-5
  done 09-27 (results there). Left: test 1 on a large stretch (review item
  1), and the oscillation check along a baseboard at 300/450/600/800.

## Closed

- 2026-10-06 Review of 09-30 (issue 15): (a) map repairs only at rest, their
  first pass keeping the robot's cell's walls (test_repair_at_rest), and (b)
  the shorter STATUS "flash:" line, in the firmware since the 10-05 flash;
  calib_analyze.py unwraps its int16 columns (long CAL RUN recordings); the
  README's CAL table rewritten. Ideas left for the review's plan (issue 19):
  cap the race return's curve speed, a SysTick stop if the following error
  runs away while the main loop blocks, measure the SysTick load.

- 2026-10-05 The flash wedge (issue 18; `docs/faults/oshwdem2026.md`, `docs/faults/freezes.md`):
  hard motor reversals leave the running HSI crawling (~8000x slow flash
  operations, a reset that does not boot). Fixed by stopping the HSI except
  during flash operations (e3c5b21), validated on a stand: control wedged
  after 20 reversals, the fix clean over 600, reset boots, saves normal.
  Watch STATUS "HSI n" (should stay 0) and "!! flash" lines in real runs.

- 2026-09-30 Simplification (issue 16; the user found the code too
  complicated): removed `TURN_CARRY`, `SIDE_LEVER`, the old centring
  integral (`TUNE OBSERVER 0`, `BIAS_WIN`) and `SENSE_SETTLE`, all neutral
  after their tests (4ceda32: host transcripts identical, RAM -200 B,
  flash -544 B; next flash, keeps map and parameters). `robot_config.h`
  one line per constant (9c0ff65: 358 -> 157 lines, firmware identical
  but the build time), measurements in `docs/design.md`; the rule for
  new knobs in AGENTS.md. `CURVE_PRE`/`CURVE_POST` stay: `calib_analyze.py`
  suggests them after `CAL CURVE` (a new floor).

- 2026-09-27 Centring on straights (issue 14, layout I): no weave at
  300-600; the slow convergence was the observer learning a turn's yaw
  (KI 8: -1.5..-3 mm to the end of 3 cells). KI 16 is the new default
  (f95a56d, next flash; until then send `KI 16` after a reset): ~0 from the
  second cell; 24 weaved. At 800 `STEER_VREF` 400-600 no difference; a 2-4
  mm step at the two-wall to one-wall post. `docs/design.md`, centring.

- 2026-09-27 Race 2.5/2.6 curves and the goal on G (issue 12): the curves
  at 480 slide outwards (grip limit, ~3.3 m/s^2 lateral) and two left ones
  in a row left mode 5 77 mm short of the goal, then crashed on the return.
  At 400 (race 2.5, `FAST_MID_*` 900/400) two full cycles on G clean both
  ways, every end `end=IR` (goal: FL 96/FR 122 squared and aligned, then
  85/103), distances +-18 mm between runs, absorbed by the IR. Race 2.6
  (900/480) stays in the menu at the grip limit: not repeatable, use it
  knowing that.

- 2026-09-27 New mode menu (issue 13): 1 search, 2 races (2.1/2.2 left/right
  followers, 2.3 no curves, 2.4 800/300, 2.5 900/400, 2.6 900/480), 3 erase.
  Validated on G: the left follower reported the goal after 11 actions and
  went on 17 legs until STOP (no "!!"); race 2.3 there and back clean (5 + 5
  legs, all `end=IR`, err <= 3 mm). Its save hit the flash wedge and the HSI
  restart cleared it (`docs/faults/freezes.md`, n=2).

- 2026-09-27 Side-sensor calibration (issue 6, roadmap): no flash needed.
  `calib_analyze.py` now fits rounds of `CAL NOISE` + `CAL TURN 1` x4 (the
  fronts facing the side walls measure the offset): slopes SL 0.95, SR 0.99,
  centred 88.1/75.3 vs 89/76 in firmware (0.1 mm of centring), kept. The
  180 deg pair needs the robot centred front to back (warns now).

- 2026-09-27 Flash of the followers' straights and the boot sweep (issue
  10): sweep and SELECT OK; on E mode 2 reached the goal in 9 actions,
  mode 3 in 11 (dead end at (1,0) included), 2-cell legs, no "!!".
  `TURN_CARRY` stays 0 (would need its own in-place scale; no gain on D).

- 2026-09-27 Flash of the new modes (issue 9): LEDs and SELECT OK, mode 2
  to the goal (9 actions), modes 4 and 5 on E clean both ways (route
  `2R1R1L1R1L1L2` in one leg, no "!!"). Mode 3 moves to issue 10.

- 2026-09-27 `!! the program stalled 3000 ms` while mode 4 (now 6)
  waited for the erase confirmation: the wait now reports health (next
  flash).

- 2026-09-27 Speed-run curves at 480 off-centre on staircases (issue 1):
  the 180's over-turn fixed (`TURNTICKS` 403), wheel drift trimmed
  (`WHEEL_DIFF` -0.0025), and the curves' sideways slip compensated with
  `CURVE_PRE_SLIP` 7 above `CURVE_PRE_V0` 300: layout E's last straight
  went from +21 mm left to +4.6/0.0 at 480, 0.0 at 400, +0.4 at 300
  (`docs/design.md`, curve slip). `TURN_CARRY`, `SIDE_LEVER`: no clear
  gain, left off.

- 2026-09-27 Flash wedge again (09-27 00:26, `docs/faults/freezes.md`): a restart
  of the HSI cleared it (n=1), so a slow halfword now restarts the HSI and
  the write goes on (a bad record is rewritten in the next slot); only a
  second slow one blocks the store. Goes with the next flash (keeps map and
  params); if a "!! flash: 2 bytes en ..." line appears, copy it into
  `docs/faults/freezes.md`.

- 2026-09-26 Search legs on long straights (layout D, 450 mm/s): two
  searches (16:16, 16:23), both identical: 13 actions, the legs 2 N, 3 E,
  3 W, 2 S decided each next cell in time and stopped on the front wall
  (`end=IR`) 0.3-0.5 mm short, err <= 1.7 mm / 2.7 deg; map right, no "!!".

- 2026-09-26 Freezes, the flash wedging (`docs/faults/freezes.md`): fix 452756d
  validated on layout D (search, 4 speed runs, `SAVE`s to a full log and
  its compaction: 56-63 us a halfword, no "!!"). The wedge never showed in
  those writes; if a "!! flash" line appears, reopen with it.

- 2026-09-26 Robot checks of the 10:48 firmware: 1-cell search legs leave
  no doubtful sides (only rest readings do, by design); step mode does not
  pause before a speed run; `CLOCK HSI` (64 MHz), `CAL TURN 4`, `RESET`
  (boots, loads the map, back on the crystal) all work.
- 2026-09-26 Layout C scrape: the centring's integral learned an
  off-centre start as heading; replaced by an observer (1472d96). Squaring
  limited to 12 deg (93370fc). Stall reports only printed in mode 5
  (0221cd6). `CAL RUN` records a speed run (04c0caf, a6d7fac).
