# Open issues

One entry per issue: state and next step, one or two lines each. Update the
entry before ending a session; move it to "Closed" with its result and
commit when done. Details go in the docs it points to, not here.

## Open

1. **Speed run: curves at 480 mm/s leave the robot off-centre; staircases
   add it up.** The -1.2 deg bias of the straights is fixed (the 180 at the
   start over-turned: `TURNTICKS` 403, 8025265). Layout E (`docs/mazes.md`,
   six curves in a row), last straight: CURVE 480 +19..+21 mm left (4 runs),
   CURVE 300 ~0; live `CURVE_PRE -8` and `CURVE_SLIP 0` crashed the robot
   (details in `docs/control.md`, curve slip). The robot keeps `CURVE 300`
   saved (safe); the code default is still 480. Ring layout F
   (`docs/mazes.md`; `docs/control.md`, "curve angle on the ring"),
   analysed offline: the centring's turns are real; the encoders run ~3
   deg/m ahead of the robot (to the right).
   `calib_analyze.py --chain <session csvs>` follows it. Loops at 400 and
   480 with centring (21:06-21:22, all clean): the curves' slip is ~1.5
   +- 0.7 deg at 300-480, not growing with speed; the asked angles (90 +
   2.0 (v/480)^2) are within ~0.7 of the ~91.5 needed: no change. So the
   curves' heading is calibrated; E's +20 mm at 480 is the sideways
   displacement. Wheel drift trimmed on layout D (21:41-21:56, three
   laps of `CAL STRAIGHT`/`CAL TURN` at 300, all OK): the wheels differ
   ~2.5 deg/m; `WHEEL_DIFF` default -0.0025 committed (`docs/control.md`,
   after "trimmed live"), not flashed yet: the robot runs with 0 until
   then. Next flash written (all TUNE, default off but WHEEL_DIFF; host
   tests OK, numbers in `--control`; RAM 86.9 %, flash 91.9 %; keeps map,
   goal and saved params): `TURN_CARRY` 1 (a turn's rest goes to the next
   move; then `TURNTICKS` ~401.5; sim: 90+90 = 180), `SIDE_LEVER` 55
   (sim: bias error of 1-2 cell straights -25..30 %), `CURVE_PRE_SLIP` k
   + `CURVE_PRE_V0` v0 (pre += k (v^2-v0^2)/(480^2-v0^2); sim on E: +7 at
   480 moves the exit +20 mm right; V0 300 leaves 300 alone, pure v^2
   would move it +7). Battery after the flash:
   Flashed 22:50: a) OK (stack 1712 free). On E (ERASE, search 31
   actions), slip 0, lateral at the last straight's first reading:
   300 -6.2, 400 +10.0, 480 +19.7 mm (left > 0); `wheel_diff_ppm=-2500`.
   V0 300 at 480: slip 3 +6.8, 5 +1.1 and +2.0, 7 +17.4 (outlier?
   stopped); slip 5 at 400: 0.0. Defaults now 5 / 300 (not flashed;
   live they need TUNE after every reset). Next: b), c) on layout D.
   b) done 23:02-23:12 (D, facing the south wall; FL-FR change per 4
   quarter turns, right/left, 1.12 mm/deg from 401->402): 403 carry 0
   -1.5/+0.8; carry 1: 401 -0.8/+1.9, 402 -2.4/+2.3, 399 -0.3/-1.5.
   In-place turns: real = encoder x TURNTICKS/398.2 (every set
   397.9-398.6); 403 carry 0 turns each 90 ~+0.3 deg real.
   c) done 09-27 00:19-00:30, D speed runs with `CAL RUN` at CURVE 480
   (PRE_SLIP 5/V0 300), the start yaw after the previous run's final 180,
   carry executed (from the first straight's "paralelo" heading):
   carry 0/403 +1.8 (A3; before +0.8, -0.5); carry 1/398 +0.3, -1.3,
   -0.5. No clear gain, and TURNTICKS is also the curves' scale: at 398
   the curve turned ~2 deg less (straight 2 +2..+4.7 deg, exit +15..+23 mm
   left, 403: -3..+12). Carry 1 needs its own in-place scale (TURNTICKS
   398 for turns, 403 for rolling): next flash, then retest; until then
   carry 0/403. `SIDE_LEVER` 55 (L2, L3) vs 0 (A2, A3): straight 1 ends
   -5 mm vs -8, straight 2 the same (+14/+15 at the curve exit either
   way), end bias vs parallel no better: inconclusive, keep 0.
   !! The robot's last good flash record (B1's save) has TURNTICKS 398:
   after the next power-on send `TURNTICKS 403` + `SAVE`.
   Next: d) below; the curve exit at 480 on D is +12..+17 mm left too.
   d) layout E (from D: remove east of (1,1); add east of (1,0), (1,2)),
      `ERASE`, search, speed runs with `CAL RUN` at CURVE 300, 400 (new
      point), 480, slip 0: E@400 ~+14 left = v^2 (V0 0), ~+10 = V0 300.
      Then `CURVE_PRE_SLIP` 3, 5, 7 at 480 (stop at the first anomaly),
      recheck 300. Validated values into robot_config.h.
2. **Bluetooth drops** (2026-09-26 ~11:00, battery freshly charged): the
   link fell 6 times in 8 min, 3 of them during `@D` dumps (lost; `CAL DUMP`
   resends), with the robot at rest; no kernel errors on the PC. Hours of
   the same work before without drops. 20:32-21:00: 237 drops with the
   robot at rest on layout F after the ring session (low battery?).
   21:04-21:22, charged: no drops in 30 dumps; 21:41-21:56, none in 47.
   Next:
   charge it; if it goes on, note where the robot and the PC are, try a
   dump next to the PC, and watch `btmon`.
7. **Flash wedge again** (09-27 00:26, `docs/freezes.md`): L1's run-end
   save (the 2nd since the power-on) wedged: "1a 344891 us", cut, store
   blocked; RCC_CR 030b4d83 before and after; "tras reiniciar el HSI, 2
   bytes en 56 us (normal)": the designed test says the HSI was the
   culprit (n=1). Robot and runs fine. Next: write it into
   `docs/freezes.md`; consider (next flash) restarting the HSI and going
   on instead of blocking the store.
8. **Minor: `!! el programa estuvo parado 3000 ms`** while mode 4 waited
   for the ERASE confirmation (09-26 23:13, PC in `leds_set_mask` from
   `main`): the stall check flags the UI's own 3 s wait. Silence it there.
6. **Roadmap, not started:** side-sensor calibration; longitudinal
   correction on wall edges (posts); short diagonals (low priority).

## Closed

- 2026-09-26 Search legs on long straights (layout D, 450 mm/s): two
  searches (16:16, 16:23), both identical: 13 actions, the legs 2 N, 3 E,
  3 W, 2 S decided each next cell in time and stopped on the front wall
  (`fin=IR`) 0.3-0.5 mm short, err <= 1.7 mm / 2.7 deg; map right, no "!!".

- 2026-09-26 Freezes, the flash wedging (`docs/freezes.md`): fix 452756d
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
