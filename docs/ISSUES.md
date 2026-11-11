# Open issues

One entry per issue: state and next step, one or two lines each. Update the
entry before ending a session; move it to "Closed" with its result and
commit when done. Details go in the docs it points to, not here.

## Next session

- `docs/roadmap.md` has the order of work and its numbers. First: one
  flash of `practice` and the robot session in `docs/testing.md` ("Next
  robot session", steps 1-6: wall ends, a search and the races, the
  read-only races, the sides with `CONT OFF`, the doubtful front, the
  followers' own map), then merge `audit-11-11` into `develop`.
- Without the robot: nothing pending on the branch; do not stack more
  behaviour changes on it before the robot session.
- Waiting on the user: review item 8 (recommended: keep the
  monitor's planner port, the transcript tests' cross-check of the
  firmware); the hardware of item 6; the race 2.4 crash at OSHWDEM
  (issue 17).

## Open

- Issue 22, review of 10-22 and audit of 11-11 (`docs/review.md`, last
  two sections), branch `audit-11-11`. 10-22: every small fix done but
  the untracked `docs/img/` (the user's call); questions 3-6 left. Audit,
  all in the simulator, waiting for the robot (`docs/testing.md`, steps
  3-4): A1/A2 races only read the map (c41d383); A3 a leg blocked in its
  cell keeps "just turned"; A4 the sides on the way in sampled every 8
  ms; A5 the map repair looped under noise (14 searches in 6000 failed,
  now 0); a front only one sensor sees at rest is doubtful (question 2
  of 10-22). Step 2: `SEARCH_COST_TURN` 1 and `FAST_COST_*` 2/1
  measured, stay. Step 3: notes closed, `commands.c` and `main.c` read
  again, nothing found; the Python tools read (`calib_analyze.py`
  crashed on a damaged row; its pre-speed-control analyses deleted).
  A6: the wall followers no longer write the map (a blank map of
  their own). `CHECK` removed. Left: the robot; questions 3-6 of 10-22;
  the plan is `docs/roadmap.md`.

- Issue 17, OSHWDEM 2026 (2026-10-03; `docs/faults/oshwdem2026.md`): the map
  loss is solved (saves at the goal, the flash wedge fixed: issue 18,
  closed), flashed 10-05. Left: the race 2.4 crash well into the maze, and
  why no race would start after it (a wall marked on the verified path?
  A candidate with evidence: `docs/review.md`, audit of 11-11, A1);
  the user will bring them up. A smoke test of the competition firmware in
  the air needs ~30 cm clear around the sensors.

- GitHub issue #1, centring (`docs/faults/centring.md`): the PD wall
  follower (KP 8, KD 0.6), on the robot since 10-08: crooked starts of
  21-39 deg at 100-900, 36 mm off, 180s and curves all centred within 1-2
  cells, no touch (section 9). The wall-end kick (4-12 deg, once
  `MOVE_SLIPPED`, section 10) fixed in the simulator 10-22: a side reading
  that jumps 8 mm away stops counting until steady over 20 mm (section
  11; `TUNE STEER_HOLD 0` = the old centring). Next: the robot.

- Issue 19, work plan of the review (`docs/review.md`): done 10-06 items 2
  (flash defences removed), 3 (`DEV_TOOLS`: competition build lean) and 7
  (LEDs); 5 (split `motion.c`) declined. Left: 1 (large stretch), 4
  (HC-05 at 115200: the module in AT mode, on the robot), 6 (hardware, the
  user's call), 8 (monitor draws the robot's route: the user's call). Ideas
  from issue 15: a SysTick stop if the following error runs away while the
  main loop blocks (no such failure seen: only with one); the SysTick
  load: 424 us worst after a search (10-08); read it after a race (the
  return's curves are capped at 300 since 10-06).

- Issue 11, 16x16 competition readiness (`docs/testing.md`): tests 2-5
  done 09-27 (results there). Left: test 1 on a large stretch (review item
  1), and the oscillation check along a baseboard at 300/450/600/800.

## Closed

- 2026-11-11 Lighting check (issue 21; `docs/lighting.md`): `CHECK`
  removed. On the robot (10-08) it gave FAIL at home under good light:
  the shifts it resolved (a few mm) are the size of the placement's and
  the maze's tolerances. Flash -1.7 KB. The competition checklist reads
  the light with `IR` (`docs/testing.md`); the IR still have no ambient
  subtraction (hardware), so a strong lamp can still make an open right
  side a wall: shade it.

- 2026-10-08 The IR delay (issue 20; `docs/measurements.md`): 8 wall
  approaches at 100-900 mm/s (two at each) gave FL 16-24 / FR 24-32 ms at 600-900 (the
  same ms at both speeds: a delay, no slide), in-place turns 0-8 ms on all
  four. `IR_DELAY_MS` 50 -> 22 (the user's call: stops alike, ~40 ms less
  crawl at the end), goes with the next flash; `SEARCH_LEG_SPEED_MAX`
  stays 450 (robust before fast).

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
