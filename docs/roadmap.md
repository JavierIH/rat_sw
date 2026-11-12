# Roadmap (2026-11-11)

Where the firmware stands after the audit of 11-11 and what to do next,
in order, to make the robot more robust, the code simpler and the runs
faster. Every number is from the simulator on the 523 classic 16x16 mazes
of `micromouseonline/mazefiles` (perfect sensing unless it says noise)
or from the robot session it names. The history is in `review.md`; the
state of each issue in `ISSUES.md`.

## Where it stands

- Simulator: 523 of 523 searches end with the optimal path verified,
  182 s on average; 0 searches fail in 4184 with 3 % noise (4 before
  11-11); the race's route to the goal takes 17.5 s at 900/400.
  `make -C test/host`: 3963 checks; tools: 34 tests.
- Robot: the last flash is 10-08. Everything below is in the code and in
  the simulator only:
  - the wall-end hold of the centring (GitHub issue #1), `IR_DELAY_MS` 22;
  - races only read the map (2.3-2.6 on 11-11, the followers 2.1/2.2 too);
  - a leg blocked in its cell keeps "just turned" (A3), the sides on the
    way in sampled every 8 ms (A4), map repairs that escalate (A5);
  - a front only one sensor sees at rest is doubtful;
  - `CHECK` removed.
- Sizes: `competition` RAM 52.7 %, flash 82.4 % (11.2 KB free);
  `practice`/`dev` RAM 84.4 %, flash 91.0 % (5.7 KB free).

## Rules that hold (do not break them)

1. No entry of the race menu changes the map or writes the flash
   (`test_races_only_read`). A race that finds a wall on its verified
   route ends there: the pose is wrong, not the map. Only the search
   records walls, and only the search and the console save.
2. A reading that is not sure is not recorded, neither as a wall nor as
   open (sides and front).
3. The pose moves only with a confirmed move; anything else ends the run.
4. Nothing erases the flash during a run; saves happen at rest.
5. A new constant or knob needs a measurement; one that stays neutral
   after its test is deleted, and so is a path that was superseded.

## 1. Robustness

In order. The first two are worth more than any new code.

1. **Validate on the robot what only the simulator has seen.** Eight
   changes wait behind one flash (`practice`), and flashing means taking
   the robot apart: one flash, then the session of `testing.md` ("Next
   robot session", steps 1-6). The code is already in `develop` (merged
   11-12), so `develop` is ahead of the robot until that flash.
   Until then do not stack more behaviour changes on it.
2. **A large maze.** The robot has never run more than the 4x3
   (issue 11). Two things only a long run shows:
   - the decisions on the way. On the practice maze the worst took 4.3 ms
     for 1024 pops (10-08); the worst on the real mazes is 5144 pops,
     ~12 ms estimated (139 cycles a pop on the emulated chip, plus the
     2.3 ms that decision took beyond its plan): 5.5 mm at 450 mm/s of
     the 8 mm `SEARCH_LATE_MARGIN_MM`. `STATUS` after a search must say `late 0`;
     if not, raise `TUNE LATE_MARGIN` before touching the planner;
   - the heading over long straights and many turns (no gyro): `LOG 2`,
     the `err=` of every stop.
3. **Record every competition run.** The race 2.4 crash at OSHWDEM
   (issue 17) has no log, so it cannot be explained. `bt_logger.py`
   running and `LOG 2` at every run; a crash with a log is a fix.
4. **The light.** The IR have no ambient subtraction and the hardware is
   fixed: ~200 counts of extra light make an open right side a wall.
   The automatic check failed (`lighting.md`); the defence is the
   checklist's `IR` reading and shade. If a venue bites, the one piece
   worth trying is the fronts' open reading at START (no turns, the
   placement does not move it), with a lamp measurement first.
5. **What noise still does** (3 % of readings wrong, far more than the
   robot's): 1 race in ~3000 stops at a wall on a passage two wrong
   sightings verified, and 7 in 6000 end at the start for a wall "seen"
   twice in front. Both end the race with the map untouched; a new
   search heals the first. Tried: races that trust only passages driven
   or seen three times (`EV_VERIFIED` 3) cost 0.03 s a race and leave 14
   of 523 routes not optimal; in 2092 noisy cycles neither setting had a
   failure, so there is nothing to show for it. Only if the robot shows
   the failure.
6. **The stack with a follower.** Races 2.1/2.2 keep the map's copy on
   the stack (292 B): read "stack never used" in `STATUS` after one
   (2240 B after a search on 10-08; about 1.9 KB expected).

Not planned: a SysTick stop when the following error runs away while the
main loop blocks (never seen), a lock on the map (the test covers it).

## 2. Simplicity

Removed on 11-11: the races' exploring and saving (-170 B), `CHECK`
(103 lines, -1.7 KB), the pre-speed-control analyses of
`calib_analyze.py` (127 lines), the Spanish left. What is left to
remove, each with what decides it:

1. **One way to search.** There are two: straight legs decided on the
   way (the default) and `CONT OFF`, a stop in every cell. In the
   simulator the legs are 4-7 % faster (30 random mazes of each kind:
   106.6 against 111.1 s on perfect ones, 49.3 against 52.8 on open
   ones): that is all they buy. After
   the large maze, keep one:
   - legs clean (`late 0`, the map right): delete `CONT OFF`, `forward()`
     in `search.c`, the sides' sampling in `run_path()` and the mode's
     tests (~60 lines, and the robot check of A4 with them);
   - legs not clean: delete the legs (`motion_explore()`, `explore_step()`,
     `path_grow()`, the leg code of `search.c`: ~200 lines) and take the
     5 %.
2. **The race menu.** Race 2.6 (900/480) was not repeatable (issue 12).
   If it still is not after the wall-end fix, delete it: the menu and
   `app_mode_t` (which keeps old numbers for the telemetry) get shorter.
3. **Knobs at their neutral value.** `TUNE` has 33 entries. To delete
   once their test is done: `STEER_HOLD` 0 (the old centring) when the
   wall ends pass on the robot; `CURVE_PRE`, `CURVE_POST`, `CURVE_ANGLE`
   (0, 0, 90: kept on 09-30 for `calib_analyze.py`'s suggestions) unless
   a `CAL CURVE` session moves them.
4. **The flash stress tools** of the `virtual` build (`TUNE STRESS`,
   `REVERSAL`, `HSI`): they reproduce the wedge fixed on 10-05. Delete
   them after a competition without a "!! flash" line.
5. `motion.c` (905 lines) stays in one file (the user's call, 10-06);
   item 1 is what shortens it.

## 3. Efficiency

Where the time goes: a search is 182 s, of which reaching the goal and
coming back is 132 s and verifying the best path (phase OPTIM) 50 s; the
race is 17.5 s to the goal.

1. **The OPTIM phase is a choice, by the competition's rules.**

   | `OPTIMIZE_MAX_STEPS` | search | route at 900/400 | optimal |
   |---|---|---|---|
   | 0 | 131.7 s | 18.21 s | 251 of 523 |
   | 100 | 143.2 s | 18.07 s | 305 |
   | 200 | 166.2 s | 17.76 s | 420 |
   | 400 | 181.4 s | 17.49 s | 520 |
   | 800 (now) | 182.1 s | 17.49 s | 523 |

   If only the best run counts, 800 is right. If the search time counts
   (the classic rule adds 1/30 of it to the run), the 50 s cost 1.7 s to
   win 0.7 s: no OPTIM is better at every row. It is a compile-time
   constant today; make it a console setting (`OPTIM n`, not saved, as
   `CONT`) once the rules of the next competition are known.
2. **The legs' speed.** `SEARCH_LEG_SPEED_MAX` 450 -> 600 takes the search
   from 182 to 161 s (300: 230 s). It was kept at 450 on 10-08 (robust
   before fast). To raise it: the decisions must fit (~7.3 of the 8 mm
   margin at 600, so `LATE_MARGIN` up first) and the stops on a front
   wall must stay `end=IR` within 1 mm on layout D at 600.
3. **The races are at the hardware's limits.** Straights: the motors
   hold ~937 mm/s (`FAST` 900). Curves: ~480 by the motors, less by the
   grip. `ACCEL` 3000: 5000 slipped. The planner's costs are measured
   (`review.md`, step 2): 2/1 picks the best route in 198 of 200. What
   would still gain is diagonals, which the user dropped.
4. **CPU.** A plan is ~2 ms (1024 pops), `curve_setup()` 2.1 ms at rest,
   SysTick 424 us at worst (unexplained: read `STATUS` after a search
   with no `CAL` test since the boot; the recorder halves its buffer in
   SysTick). Nothing to do unless the large maze says `late`.

## Order

1. The robot session and the merge (robustness 1).
2. The large maze (robustness 2), then one way to search (simplicity 1)
   and the legs' speed (efficiency 2).
3. The rules of the next competition, then `OPTIM` (efficiency 1).
4. After the next competition: the logs (robustness 3), race 2.6, the
   stress tools, the neutral knobs.
