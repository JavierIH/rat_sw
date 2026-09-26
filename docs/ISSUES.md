# Open issues

One entry per issue: state and next step, one or two lines each. Update the
entry before ending a session; move it to "Closed" with its result and
commit when done. Details go in the docs it points to, not here.

## Open

9. **Flashed 2026-09-27, battery pending** (map, goal and params kept):
   `CURVE_PRE_SLIP` 7, modes 1 search, 2/3 wall follower left/right, 4
   FAST 800 CURVE 300, 5 FAST 900 CURVE 480, 6 erase (LED n = mode n), 5 s
   countdown (CAL 2 s), erase wait reports health. Battery on layout E:
   a) SELECT through the modes (LED n): OK; b) mode 2 to the goal: OK (9
   actions, no "!!"); mode 3 waits for issue 10's flash (user: no follower
   runs until then); c)
   mode 4 with `CAL RUN`, then mode 5 with `CAL RUN` (expect the last
   straight within ~5 mm). Also pending, low priority: `TURN_CARRY` 1 would
   need its own in-place scale (TURNTICKS 398 for turns, 403 for curves);
   no clear gain on D (`docs/control.md`), so carry stays 0.
10. **Next flash** (built, not flashed; keeps map, goal and params): the
   wall followers (modes 2/3) drive straights without stopping like the
   search (`drive_leg()` shared with it; `CONT OFF` stops in every cell);
   host: same cells, 36 % fewer stops. After it: modes 2 and 3 on E.
6. **Roadmap, not started:** side-sensor calibration. Dropped by the user
   (2026-09-27): longitudinal correction on wall edges (too fine for our
   precision) and diagonals.

## Closed

- 2026-09-27 `!! el programa estuvo parado 3000 ms` while mode 4 (now 6)
  waited for the erase confirmation: the wait now reports health (next
  flash).

- 2026-09-27 Speed-run curves at 480 off-centre on staircases (issue 1):
  the 180's over-turn fixed (`TURNTICKS` 403), wheel drift trimmed
  (`WHEEL_DIFF` -0.0025), and the curves' sideways slip compensated with
  `CURVE_PRE_SLIP` 7 above `CURVE_PRE_V0` 300: layout E's last straight
  went from +21 mm left to +4.6/0.0 at 480, 0.0 at 400, +0.4 at 300
  (`docs/control.md`, curve slip). `TURN_CARRY`, `SIDE_LEVER`: no clear
  gain, left off.

- 2026-09-27 Flash wedge again (09-27 00:26, `docs/freezes.md`): a restart
  of the HSI cleared it (n=1), so a slow halfword now restarts the HSI and
  the write goes on (a bad record is rewritten in the next slot); only a
  second slow one blocks the store. Goes with the next flash (keeps map and
  params); if a "!! flash: 2 bytes en ..." line appears, copy it into
  `docs/freezes.md`.

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
