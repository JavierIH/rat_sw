# Open issues

One entry per issue: state and next step, one or two lines each. Update the
entry before ending a session; move it to "Closed" with its result and
commit when done. Details go in the docs it points to, not here.

## Open

1. **Speed run: heading error after a staircase of curves.** Measured
   on layout C at 478 mm/s: best `CURVE_ANGLE` ~92 (+-2 per run). On the
   robot since the 14:30 build of 09-26 (e9bd652): 90 + `CURVE_SLIP`
   (v/480)^2, 2.0 deg, `TUNE CURVE_SLIP`; not validated yet (issue 3 first).
   Next: speed runs with `CAL RUN` on layout D (one curve then 3 cells): the
   centring's heading on the last straight should average ~0.
2. **Bluetooth drops** (2026-09-26 ~11:00, battery freshly charged): the
   link fell 6 times in 8 min, 3 of them during `@D` dumps (lost; `CAL DUMP`
   resends), with the robot at rest; no kernel errors on the PC. Hours of
   the same work before without drops. Next: if it goes on, note where the
   robot and the PC are, try a dump next to the PC, and watch `btmon`.
3. **Freezes: the flash wedges** (`docs/freezes.md`, summary at the top).
   The chip is a clone whose flash sometimes runs ~9000x slow until a power
   cycle; the cause is unknown. Fix in 452756d (not flashed yet): no run
   erases; saves program a slot a halfword at a time and stop at the first
   slow one (~0.5 s, store blocked, map in RAM); only the boot erases.
   Next: flash (the saved map is discarded once, STORE_VERSION 7), power
   cycle, `STATUS` (5 free slots, "1a ~56 us"), then searches and speed
   runs as usual; any "!! flash" line: copy it into freezes.md.
5. **Search legs on long straights.** Layout D (`docs/mazes.md`, set up
   now): route N 2 cells, E 3 to the goal.
   Check the decision point and the front wall at 450 mm/s.
6. **Roadmap, not started:** side-sensor calibration; longitudinal
   correction on wall edges (posts); short diagonals (low priority).

## Closed

- 2026-09-26 Robot checks of the 10:48 firmware: 1-cell search legs leave
  no doubtful sides (only rest readings do, by design); step mode does not
  pause before a speed run; `CLOCK HSI` (64 MHz), `CAL TURN 4`, `RESET`
  (boots, loads the map, back on the crystal) all work.
- 2026-09-26 Layout C scrape: the centring's integral learned an
  off-centre start as heading; replaced by an observer (1472d96). Squaring
  limited to 12 deg (93370fc). Stall reports only printed in mode 5
  (0221cd6). `CAL RUN` records a speed run (04c0caf, a6d7fac).
