# Open issues

One entry per issue: state and next step, one or two lines each. Update the
entry before ending a session; move it to "Closed" with its result and
commit when done. Details go in the docs it points to, not here.

## Open

1. **Speed run: heading error after a staircase of curves.** Measured
   2026-09-26 on layout C (1D1I1D2, 478 mm/s), heading the centring holds on
   the last straight: `CURVE_ANGLE` 90 -> +2.4..+5 deg, 92.5 -> -3.1..-0.2;
   best ~92, +-2 per run (`tools/calib_data/2026-09-26_1[12]-*_run.csv`).
   Next (batched flash): angle 90 + `CURVE_SLIP` (v/480)^2, 2.0 deg,
   `TUNE CURVE_SLIP`; validate with 3-4 speed runs on layout C.
2. **Bluetooth drops** (2026-09-26 ~11:00, battery freshly charged): the
   link fell 6 times in 8 min, 3 of them during `@D` dumps (lost; `CAL DUMP`
   resends), with the robot at rest; no kernel errors on the PC. Hours of
   the same work before without drops. Next: if it goes on, note where the
   robot and the PC are, try a dump next to the PC, and watch `btmon`.
3. **Freezes in flash writes** (`docs/freezes.md`). 2026-09-26 11:50: a
   run's final save froze 213 s and failed, then `SAVE` at rest froze 200 s
   and failed; HSI on and FLASH_SR clean before and after (HSI hypothesis
   refuted), the CPU stalled on the flash bus. Next (batched flash): skip
   unchanged writes, full diagnostic line, chip identity in STATUS.
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
