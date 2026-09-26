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
   saved (safe); the code default is still 480. Next: calibrate the curve
   angle on single curves (ask the user for a layout with one curve then 2+
   cells walled on both sides, left and right; `CAL CURVE` at 300, 400, 480,
   the heading after each from `calib_analyze.py`'s "paralelo a las
   paredes" line; E hints left curves turn ~0.6 deg more than right ones),
   then set `CURVE_SLIP`/`CURVE_ANGLE` and re-check E starting at 300. The
   root fix of the turns (carry each turn's ~0.78 deg shortfall into the
   next move, `TURNTICKS` to ~401.5) needs a flash: batch it with the curves.
2. **Bluetooth drops** (2026-09-26 ~11:00, battery freshly charged): the
   link fell 6 times in 8 min, 3 of them during `@D` dumps (lost; `CAL DUMP`
   resends), with the robot at rest; no kernel errors on the PC. Hours of
   the same work before without drops. Next: if it goes on, note where the
   robot and the PC are, try a dump next to the PC, and watch `btmon`.
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
