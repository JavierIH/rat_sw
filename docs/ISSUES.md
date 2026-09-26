# Open issues

One entry per issue: state and next step, one or two lines each. Update the
entry before ending a session; move it to "Closed" with its result and
commit when done. Details go in the docs it points to, not here.

## Open

1. **Speed run: heading error after a staircase of curves.** Measured
   on layout C at 478 mm/s: best `CURVE_ANGLE` ~92 (+-2 per run). In
   firmware (e9bd652, not flashed): 90 + `CURVE_SLIP` (v/480)^2, 2.0 deg,
   `TUNE CURVE_SLIP`. Next: flash, then speed runs with `CAL RUN` on layout
   D (one curve then 3 cells): the centring's heading on the last straight
   should average ~0.
2. **Bluetooth drops** (2026-09-26 ~11:00, battery freshly charged): the
   link fell 6 times in 8 min, 3 of them during `@D` dumps (lost; `CAL DUMP`
   resends), with the robot at rest; no kernel errors on the PC. Hours of
   the same work before without drops. Next: if it goes on, note where the
   robot and the PC are, try a dump next to the PC, and watch `btmon`.
3. **Freezes in flash writes** (`docs/freezes.md`). Cause unknown; every
   stuck write began within ms of a move (92 writes at rest or 1-2 s after
   hard moves were fine). Protection in firmware (490c9d5, 5595853, not
   flashed): one save per run and only if changed, 1 s settle, a probe
   halfword before erasing, lock after a failure; `CAL FLASH [n] [ms]`
   reproducer. Next: flash, `CAL FLASH 100 0` (immediate writes), then
   `CAL FLASH 100 1000`; power-cycle after any lock.
5. **Search legs on long straights.** Layout D = C with the wall
   (0,1)/(0,2) moved to (0,1)|(1,1): route N 2 cells, E 3 to the goal.
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
