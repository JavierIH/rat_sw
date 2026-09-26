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
   then. Next flash, all in one: that default, `wheel_diff_ppm` in the
   dumps (done), and still to write: a `CURVE_PRE` growing as (v/480)^2
   (E's kinematic model: ~+7 at 480 cancels its +20 mm), the turns'
   shortfall carry-over (`TURNTICKS` ~401.5), the nose lever in the
   centring's observer, each behind TUNE. After it: a dump shows
   `wheel_diff_ppm=-2500`; `CURVE_PRE` on single curves at 400, then E.
2. **Bluetooth drops** (2026-09-26 ~11:00, battery freshly charged): the
   link fell 6 times in 8 min, 3 of them during `@D` dumps (lost; `CAL DUMP`
   resends), with the robot at rest; no kernel errors on the PC. Hours of
   the same work before without drops. 20:32-21:00: 237 drops with the
   robot at rest on layout F after the ring session (low battery?).
   21:04-21:22, charged: no drops in 30 dumps; 21:41-21:56, none in 47.
   Next:
   charge it; if it goes on, note where the robot and the PC are, try a
   dump next to the PC, and watch `btmon`.
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
