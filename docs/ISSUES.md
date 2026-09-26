# Open issues

One entry per issue: state and next step, one or two lines each. Update the
entry before ending a session; move it to "Closed" with its result and
commit when done. Details go in the docs it points to, not here.

## Open

1. **Speed run: heading error after a staircase of curves.** The false bias
   that scraped layout C is fixed (observer, 1472d96): on the robot the run
   now reaches the goal square (was 5.5 deg off and scraping). Left: after
   1D1I1D2 the robot comes out ~4-5 deg left (at 478 mm/s each curve turns
   ~2.5 deg less than the encoders say) and drifts 11 mm off on the last
   straight (`tools/calib_data/2026-09-26_11-04-15_run.csv`).
   Next: layout C, `TUNE CURVE_ANGLE 92.5`, `CAL RUN`, `MODE 2`, `START`;
   compare "recta 2" of `calib_analyze.py` with that file (the first try's
   dump was lost to issue 2). If it helps: an angle compensation growing
   with the curve speed (90 + k v^2) in firmware; check at CURVE 400 too.
2. **Bluetooth drops** (2026-09-26 ~11:00, battery freshly charged): the
   link fell 6 times in 8 min, 3 of them during `@D` dumps (lost; `CAL DUMP`
   resends), with the robot at rest; no kernel errors on the PC. Hours of
   the same work before without drops. Next: if it goes on, note where the
   robot and the PC are, try a dump next to the PC, and watch `btmon`.
3. **Freezes in flash writes** (`docs/freezes.md`). The diagnostics are on
   the robot since 2026-09-26 (a normal write takes 38 ms; the robot was
   power-cycled after that flash). Next: nothing until a freeze or a
   "!! flash" / "!! osciladores" line shows up; then read its registers.
4. **Check on the robot** (firmware of 2026-09-26): search legs of one cell
   no longer leave doubtful sides (`SEARCH_SIDE_FROM_MM` 20); step mode no
   longer pauses before a speed run; `CLOCK HSI`, `CAL TURN 4`, `RESET`.
   One search + speed run cycle on layout C covers the first two.
5. **Search legs on long straights.** Layout D: remove walls (1,2)|(2,2),
   (2,2)|(3,2), (0,1)|(1,1); add (1,2)/(1,1), (2,2)/(2,1). Check the
   decision point and the front wall at 450 mm/s.
6. **Roadmap, not started:** side-sensor calibration; longitudinal
   correction on wall edges (posts); short diagonals (low priority).

## Closed

- 2026-09-26 Layout C scrape: the centring's integral learned an
  off-centre start as heading; replaced by an observer (1472d96). Squaring
  limited to 12 deg (93370fc). Stall reports only printed in mode 5
  (0221cd6). `CAL RUN` records a speed run (04c0caf, a6d7fac).
