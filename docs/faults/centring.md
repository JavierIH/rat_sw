# Centring far from the centre line (GitHub issue #1, 2026-10-05)

The investigation of GitHub issue #1 ("Slow centering after turns: replace
the bias observer with a PD wall follower"): what was measured on the robot,
how the faults were found and reproduced, what was changed and what is
still open. The centring as it is now: `docs/design.md`.

## 1. Summary

- The issue's proposal (a PD on the lateral error, no bias) was worse in the
  simulator: without the bias a robot whose encoders a turn left θ off the
  walls settles θ/KP off-centre (8 mm at 4 deg). Section 2.
- The user's test was another one: the robot placed by hand crooked or
  against a wall at the start of a straight, expected back in the centre
  within a cell. On the robot it ran a whole straight 30-40 mm off, along
  the wall, and into the wall from 35 deg even at 100 mm/s. Section 3.
- The causes were not the observer's speed but six faults that only show
  far from the centre line or with a big yaw (section 4): the 25 mm error
  clamp took a real offset for a sensor transient; at a big yaw one beam
  loses its wall and the other was clamped the same way; one 15 deg clamp
  for bias and pull together; a turn limit of 0.2 deg/mm at any speed; a
  false bias at every change of walls; a swing back in the recovery at
  600-800.
- Fixed in `steer_step()` (section 5), every constant live with `TUNE`.
  Flashed and seen on the robot: the first part (21:02 build: back from
  ~50 mm off and ~20 deg at 450) and the rest (22:09 build: 35 deg at 100
  and 300 back without touching). A D term (`STEER_KD`) was tried and
  removed (the user's call). The recordings then put the side IR delay at
  4-8 ms, not the 66 ms the controller compensated (section 7).
- 10-06: the whole controller replaced by the classic PD wall follower on
  the turn rate (section 8, branch `pd-steering`, not flashed yet).

## 2. The issue's proposal, in the simulator

`host_tests --control`, "straight 720 mm after a turn": yaw +-4 deg (what an
in-place turn leaves), centred, walls +-2 mm, mean of 16 seeds; |y| at the
next cell's centre (180 mm) at 300 / 600 / 900 mm/s, and the sum of the mean
|y| (mm) + |yaw| (deg) at 180-630 mm over 300-900 (lower is better). With
the firmware of the time (KP 0.5, KI 20, observer 40 mm, `STEER_MAX_DEG` 15,
`STEER_CURVE` 0.2):

| Centring | \|y\| at 180 mm | Sum |
|---|---|---|
| firmware | 3.7 / 4.7 / 6.3 | 91 |
| PD, no bias (the proposal): KP 0.5, D 0.26 deg per deg of slope over 60 mm | 5.9 / 7.1 / 8.3, then 9-17 for good | 333 |
| PD, no bias: KP 1.0, D 0.52 over 30 mm | 4.2 / 4.8 / 6.1, then 5-11, yaw 2-5 deg | 283 |
| PD on the turning rate (= the integral of the lateral error, removed 09-30), KI 8 | 2.3 / 3.0 / 3.8, weaves at 900 (4-5 mm, 2-3.6 deg) | 92 |
| the same, KI 20 | 1.0 / 1.4 / 1.4, diverges at 900 (13-18 mm, 9-13 deg) | 226 |
| firmware + D 0.26 over 60 mm | 3.7 / 4.4 / 6.0 | 91 |
| firmware + D 0.52 over 30 mm | 3.9 / 4.4 / 5.4, yaw 2-3 deg, weaves at 300-450 | 115 |
| observer 4x faster over the first 180 mm | 3.0 / 3.0 / 5.0, yaw 2.6-2.7 deg at 600-800 | 85 |
| KP 0.7 | 3.1 / 3.6 / 5.8 | 85 |
| KP 1.0 | 2.3 / 2.3 / 4.9, weaves (6-7 crossings, yaw 2-4 deg) | 123 |
| the true bias from the start (a bound) | 2.1 / 3.3 / 4.9 | 78 |

- Without the bias the robot cannot centre: the centring asks for an
  encoder heading, and driving parallel needs the heading the turn left,
  which P only asks for at yaw/KP off-centre. A D term does not change that
  (parallel to the walls the slope is 0). Read as a turning rate, the PD is
  a PI on the heading: the integral of the lateral error removed on 09-30.
- What sets the pace at small errors is the sensors: the side IR report 50
  ms late plus 16 ms of averaging, and the P loop's distance constant, 1 /
  (KP pi/180) = 115 mm at KP 0.5, which cannot shrink without the weave KP
  1.0 shows.
- The robot agreed (issue 14, 09-27 13:20-13:36 and 10-05 20:23-20:26,
  `CAL STRAIGHT 3` after a 180 at 300/450): 5-8 mm at the worst in the
  first cell, within +-2 mm from 270-315 mm.
- The mistake of this first pass: only the yaw a turn leaves was simulated,
  never a robot far off the centre line or crooked by more than 5 deg,
  where every fault of section 4 lives.

## 3. On the robot

Layout I (`docs/mazes.md`), row 0 (4 cells, 540 mm between the corner cells'
centres; the corner cells have one side wall), `CAL STRAIGHT 3 <mm/s>`, KP
0.5, KI 20; recordings in `tools/calib_data/2026-10-05_*_straight.csv`,
summarised with `tools/calib_analyze.py` (error every 45 mm, positive = left
of the centre line, as the side IR read it: ~1 mm per deg of yaw too, the
sensors are at the nose).

Firmware of the time (20:22-20:31):

- 20-30-15, placed ~13-15 deg crooked, 300: `+6 +10 +11 +11 +9 +7 +5 +5
  +2 +1 +0 -0`; the heading offset asked sat on the 15 deg clamp from 227
  to 407 mm: ~450 mm to centre.
- 20-31-33, ~11 deg crooked and ~27 mm off, 300: `+36 +32 +32 +32 +33 +33
  +33 +33 +35 +35 +36 +42`, the whole straight along the left wall, ending
  42 mm off (SL 47: ~6 mm from it). The heading asked settled at +10.6 deg,
  just the robot's yaw: parallel, never back.

After the first fix (both walls agreeing trusted, stronger pull far off,
clamps apart at 25; build 21:02:07):

- 21-05-00, ~35 mm off against the right wall, ~5 deg, 300: `-33 -24 -20
  -14 -10 -7 -3 -2 -1 +0 +2 +1`: 11-13 mm at the first cell's end, 0-2
  from 360 mm (before: 34 mm the whole straight).
- 21-06-49, against the left wall (~50 mm off) and ~20 deg into it, 450:
  56-62 mm at 47 mm, 25 at 180, 12 at 225, 4-5 at 315, centred from 405;
  heading asked up to 37 deg (bias ~21 + pull ~16).
- 21-10-53 .. 21-13-53, 180s (`CAL TURN 2`, 179 deg by the encoders) and
  straights at 600 and 800: clean stops, no weave, but straights starting
  9-21 deg crooked though the previous one ended square to the front wall
  (open, section 7).
- 21-20-16, ~30 deg crooked, tail by the right wall, 900: crossed the lane
  in 120 mm, touched the left wall at 180 mm (SL 11, FL 50), backed up to
  the start (`BLOCKED`, as designed). The heading rose at the 0.2 deg/mm
  limit; at 865 mm/s the tyres allow ~0.25 deg/mm anyway (3.3 m/s^2).
- 21-26-28, ~35 deg crooked, 100: at rest SL and SR disagreed by 32 mm;
  SR lost its wall at 31 mm (its beam runs along it); the heading rose
  exactly 0.2 deg/mm (+2 at 60 mm, +8 at 91, +17 at 135) and stopped at +17
  (one wall: clamped); into the wall at 150 mm (`LOST`).

All but the D term (build 22:09:56):

- 22-10-53, 22-11-11, 22-11-32, ~35 deg at 100: peaks 23-29 mm, centred at
  the end (`-14 -23 -21 -18 -14 -11 -8 -5 -3 -2 -1`), heading asked up to
  41-43 deg. 22-11-56, the same at 300: peak 24, centred from 450 mm.
- 22-12-18 (700), 22-13-13 and 22-13-31 (900), placed ~30 mm off: smooth
  returns (`-29 -26 -28 -20 -13 -11 -9 -3 -1 ...`).
- 22-12-37 and 22-12-55, ~33 mm off and ~8 deg, 800: a swing. The heading
  asked went +10, +19, +13, +4, +1, +9, +15, +17, +13, +10 (period ~135-180
  mm, ~5-6 Hz), the encoders 20-25 mm behind (+-5 deg), the lateral 34, 23,
  28, 25, 12, 6, 10, 4, -2...

## 4. Diagnosis

1. **The error clamp took a real offset for a transient.** Past
   `STEER_ERROR_MAX_MM` (25) the error was clamped and the bias observer
   stopped. The IR-delay prediction (the encoders' sideways motion since
   the reading, with the bias still 0) then said the robot was moving to
   the centre, and the clamped reading could not contradict it: a stable
   equilibrium parallel to the wall at the heading the yaw needed (20-31-33,
   reproduced in the simulator: `+38 +35 +32 +32 +33 ...`, the robot's
   `+36 +32 +32 +32 +33 ...`). The clamp exists because the angled beams
   catch posts and walls ahead; but those fool one sensor, not both the
   same way.
2. **At a big yaw only one wall is seen.** The beams point 15 deg forward:
   yawed 35 deg, one runs at 50 deg to its wall's normal, reads long, then
   no wall (> `SIDE_WALL_TRACK_MM`). The other was then clamped as in 1,
   so the pull never passed ~12 deg + bias (21-26-28).
3. **One clamp for bias and pull.** `STEER_MAX_DEG` 15 bounded both
   together: a robot crooked by 15 deg had no pull left (20-30-15); 25 still
   left a 30 deg yaw ~13 mm off to the end (simulator).
4. **The turn limit.** `STEER_CURVE_DEG_PER_MM` 0.2, per mm at any speed:
   straightening 35 deg takes 175 mm, the lane is crossed in ~100 (21-26-28,
   at 100 mm/s, where the tyres would allow ~8 deg/mm).
5. **A false bias at every change of walls.** On a change (two walls to
   one, or another wall) the reading followed the new reference at the slew
   rate (`STEER_SLEW_MM_PER_MS`) and through the average, and the observer,
   restarted at the change, learned that ramp as motion: +2.4 deg in
   `test_steering_bias`'s scenario, within 25 mm too, so the old code had
   it (the test only passed because its scenario sat beyond the clamp).
6. **The recovery swings at 600-800.** While the bias is still being
   learned, the IR-delay prediction (53 mm at 800) overestimates the motion
   to the centre, the pull eases early, the robot drifts back and pulls
   again (22-12-37, 22-12-55; the simulator shows the same shape, weaker:
   `+19 +27 +1 +9 -4`).

Physical limit: ~3.3 m/s^2 sideways (the curves' grip): at 865 mm/s ~0.25
deg/mm. A robot placed 30 deg or more crooked at 900 may touch the wall
whatever the control does (it backs up: `BLOCKED`).

## 5. Changes

In `steer_step()` (`src/control.c`); constants in `src/robot_config.h`, each
live with `TUNE` (name in brackets):

- Trusted readings, used however far off, the observer learning from them:
  both walls agreeing within `STEER_AGREE_MM` 12 (`STEER_AGREE`), or one wall
  alone nearer than on the centre line (a beam reads a wall too far as it
  leaves it, never too near: nothing stands in between). Otherwise the old
  rules: the implausible one of two dropped, the error clamped at 25.
- A stronger pull beyond `STEER_FAR_MM` 10 (`STEER_FAR`) by trusted
  readings: `STEER_KP_FAR` 1.0 deg per mm (`STEER_KP_FAR`), fading as
  (`STEER_VREF_MM_S`/v)^2 (2.0 weaved at 450 in the simulator; plain 1/v
  overshot at 600-900).
- Pull and bias clamped apart, each at `STEER_MAX_DEG` 40 (`STEER_MAX`).
- `STEER_CURVE_DEG_PER_MM` 0.4 (`STEER_CURVE`). 0.5 was worse in the
  simulator at 450-600: big recoveries ended 11-22 mm off (0.4: 1-8).
- The reading filter restarts on a change of walls with the observer: the
  false bias of 5 went from 2.44 to 0.14 deg.
Tried and left out: the PD without bias, and the rest of section 2; a cap
on the turn rate in deg/s at speed (200 deg/s: more swing at 800-900); the
far pull fading as 1/v^3 (no change); `STEER_CURVE` 0.5; a D term on the
encoders' yaw to the corridor (heading minus bias), `STEER_KD`: in the
simulator 0.15 took a 40 mm recovery at 600 from 12.8 to 2 mm off and
thinned the zigzag, 0.3 slowed big recoveries (while the bias is wrong it
pushes against the pull: 30 deg at 450 ~30 mm off for a cell), and neither
removed the swing of fault 6; removed before flashing, at the user's call.

## 6. The simulator

- `test/host/control_sim.c`: the side readings are geometry, the beams 15
  deg forward from the nose, scaled to read 1:1 with the lateral position:
  the same `side_lever_mm` (55 mm per radian) for small yaws as before, and
  at a big yaw one beam reads long, then no wall, and the two disagree. The
  linear model of before could not reproduce 21-26-28.
- `sim_result_t`: `y_at[]` (y and yaw every 90 mm) and `y_max` (48: touching
  a wall).
- `host_tests --control`: "straight 720 mm after a turn" (the table of 2).
- `test_speed_control()`: the robot's runs as cases, which fail on the old
  code: 20-31-33 (27 mm off, 11 deg, 450), 20 deg centred (450), 30 deg with
  the tail 38 mm off (450) and 35 deg 30 mm off (100, 21-26-28): no wall
  touched, back within 3-4 mm.
- To reproduce a run: `plant_nominal()` with `y0` (mm left of centre),
  `yaw0` (deg right of the corridor), `wall_error_mm` 2, then
  `sim_straight(&p, 540, speed, 3000, PARAM_KP, PARAM_KI)`; the start of a
  run gives y0/yaw0 from SL/SR at rest and the encoder heading at its end.

Results with the final code (max |y| / |y| at the end of 540 mm, mean of 8
seeds; 48 touches the wall):

| Case | 100 | 300 | 450 | 600 | 900 mm/s |
|---|---|---|---|---|---|
| 35 deg, 30 mm off (21-26-28) | 30 / 2.2 | 30 / 1.8 | 32 / 0.8 | 35 / 1.8 | 39 / 6.8 |
| 30 deg, 38 mm off (21-20-16) | 38 / 2.3 | 38 / 2.2 | 38 / 1.0 | 38 / 2.6 | 38 / 8.5 |
| 11 deg, 27 mm off (20-31-33) | 30 / 0.7 | 31 / 0.4 | 31 / 0.8 | 33 / 4.6 | 35 / 4.6 |
| 40 mm off, square | 40 / 0.4 | 40 / 0.5 | 40 / 1.1 | 40 / 2.3 | 40 / 4.3 |
| after a turn (4 deg) | 4 / 0.8 | 5 / 0.7 | 5 / 0.6 | 6 / 1.0 | 7 / 1.1 |
| 15 mm off | 15 / 0.5 | 15 / 0.5 | 15 / 0.5 | 15 / 0.6 | 15 / 1.5 |

Before the changes the first two touched the wall at every speed and the
third ran the whole straight 30-35 mm off. Small errors are as before
(after a turn: 3.7-6.2 mm at 180 mm, ~1 from 360).

## 7. The IR delay

The firmware assumed 50 ms (`IR_DELAY_MS`) plus half the side average (16):
the centring predicted 66 ms of motion, and the simulator's robot had 50 ms
too. The only measurement was a front wall approached at 400 mm/s
(`docs/measurements.md`: "18 mm farther"); the side sensors were never
measured. From the recordings (10-05, no robot needed):

- In-place turns (127 `CAL TURN` recordings at <= 4 ms), the reading fitted
  against the encoder angle delayed d, first 25 deg (the readings move only
  with the angle, ~1 mm/deg): SL median 8 ms (52 turns, quartiles 4-8), SR
  4 ms (60, 0-4), FL 8 ms (13), FR 4 ms (42, 0-12).
- Approaches to the end wall of the 10-05 straights (300-900 mm/s), the
  reading against the final reading plus the encoder distance still to go,
  delayed d: FL ~10-28 ms (median ~16), FR ~26-34 ms (median ~28). This
  method also absorbs any scale error of the calibration.

So the side delay is far shorter than 66 ms: the old prediction counted
motion the readings already showed, overestimating the motion to the centre
(the swing of fault 6). The motors short-brake at 0 (TB6612FNG: PWM low with
one input high), and the encoders count any coasting, so neither explains
the front's 50 ms; a slide with the wheels stopped would.

## 8. The PD wall follower (10-06, branch `pd-steering`)

The user's call: the classic wall follower of strong micromice (UKMARS
mazerunner and most others) with the side IR 5-10 ms late, and every piece
of the old controller removed.

- `steer_step()`: the heading offset turns at `KP` deg/s per mm off-centre
  plus `KD` deg per mm the error changes; both walls averaged, one wall that
  one, none: heading held; 16 ms average, error clamp 25 mm, no derivative
  across a change of walls. Removed: the observer (`KI`,
  `STEER_OBSERVER_MM`), the 66 ms prediction, the slew limit, the agreement
  and far-pull rules, the heading clamps and curvature limit, `STEER_VREF`
  and their `TUNE` entries (RAM -392 B, flash -1.2 KB). `KI` became `KD` in
  the parameters, same layout: a flash keeps the map, the parameters start
  from the new defaults.
- Simulator: side IR 8 ms late (`SIDE_IR_DELAY_MS` in `control_sim.c`).
  Grid over KP 3-10 and KD 0.3-1.5 on a turn's yaw, 15 mm off, a curve exit
  (15 mm, 4 deg), the robot's 11 deg / 27 mm, 30 and 35 deg and the 800
  swing case, at 300-900 mm/s: KP 8, KD 0.6 balanced (ends 0.5-0.9 mm off,
  second half 1.4-2.0 mm, 35 deg 4.1; ~2-3 zero crossings a straight;
  the same with the delay at 5, 10 or 20 ms). KP 3 KD 0.3 touched the wall
  at 35 deg; KD 1.5 weaved.
- Against the old controller on the same robot model (mean of 300-900 mm/s,
  largest offset / largest in the second half, mm):

| Case | Old | PD |
|---|---|---|
| after a turn (4 deg) | 6 / 2.0 | 3 / 1.4 |
| 15 mm off | 15 / 5.2 | 15 / 1.4 |
| curve exit (15 mm, 4 deg) | 17 / 8.1 | 15 / 1.4 |
| 35 deg crooked | 40 / 24.6 | 30 / 4.1 |
| the 800 swing case | 29 / 12.5 | 25 / 2.0 |

  The zero crossings rise from ~1 to ~2-3 a straight: +-0.5 mm wobbles at
  the centre, now that it gets there.
- Curves are untouched: the centring holds its offset through them and
  resumes in the new corridor. `sim_path_walls()` (10-06) runs whole paths
  between walls with the centring as `motion.c` does (blind from each
  curve until the IR read the new corridor, fading 40 mm before it). Mean
  of 8 seeds at 300-480 mm/s, largest distance off the route:

| Path | No walls, centred start | PD, centred start | No walls, 10 mm 3 deg | PD, 10 mm 3 deg |
|---|---|---|---|---|
| staircase | ~1.2 mm | 2.4 | 37-38 | 10.1 (the start) |
| layout E | ~1.1 | 2.7 | ~24 | 10.2 |
| 6 cells, curve, 6 cells | ~2.5 | 1.1 | ~79 | 10.0 (the start) |

  From a perfect start the PD costs ~1-1.5 mm in staircases: the side
  readings step ~2 mm, KD turns each into ~1 deg of heading, and the
  offset held at a curve's start carries through the next curves (no
  straight between them to correct). Lowering KD made everything worse
  (it also damps). From any real start the PD is what keeps the robot on
  the route. `test_path_centring` and `host_tests --control` keep it.

## 9. Open

- Flash the branch and check on the robot: crooked starts of 30-35 deg at
  100-900 mm/s, 180s at 600-900 (no weave), `CAL CURVE`, races 2.4/2.5;
  `KP`/`KD` are live parameters.
- The front delay: wall approaches at 100 and 600 mm/s; a mismatch that
  grows with the speed is delay, a constant one calibration or slip.
- The straights at 800 that started 9-21 deg crooked after a 179 deg turn
  from a square stop (21-10-53 .. 21-13-53): repeat the 180 series.
