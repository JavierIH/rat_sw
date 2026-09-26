# Robot validation history

What has been verified on the robot and in the simulator, with numbers.
Newest results at the end. Open problems live in docs/ISSUES.md.

## Status up to 2026-09-26
Verified on the PC simulator (hundreds of random 16x16 and 4x3 mazes, with and
without sensor noise): searches always complete, the verified speed-run path
is optimal with perfect sensing, and nothing crashes (also with 20% of side
readings doubtful). The speed control is tested on the simulated robot
(exact distances and angles, centring from bad starts, +-20% model errors).
On the practice maze with the speed control: search + return in 19.4 s at
SPD 600 (22.3 s at 400), the same map every time and no doubtful wall;
3-cell straights at 700 mm/s stop within ~2 mm of the front-wall reference
and centre (900 works but runs out of PWM at the end of the acceleration);
turns within 0.2 deg each (TURNTICKS 405); FRONT_SQUARE_OFFSET_MM confirmed
with CAL NOISE (-15.4).

Smooth curves on the robot (practice maze): the first speed run at FAST 700
/ CURVE 400 drove the 9-cell route with 4 curves (a U-turn included) in one
move, 3.14 s to the goal, tracking within 2.7 mm / 2.6 deg, and the IR stop
at the goal landed 0.5 mm from the encoders' plan after 1.49 m; the return
at 600/400 took 3.27 s. Six CAL CURVE at 300 mm/s: the curve itself turns
89-90 deg on the encoders and, with the maze's lateral bias taken out
(two left and two right between (2,0) and (3,1)), ends 1.6 +- 3 mm early:
CURVE_PRE/POST/ANGLE stay at 0/0/90. The heading offset the centring holds
through a curve (up to 3.2 deg in these tests) is its learned bias, the
misalignment of the encoders' frame: in the one test where it could be
checked (start heading measured on a wall, 3.35 deg off) it put the robot
into the curve within 0.1 deg of the corridor, and the exit came out centred
once the maze's lateral bias was taken out. Keep holding it. Raising the
speeds (practice maze, 9 cells, 4 curves): CURVE 400 / 450 / 480 (478, the
motor cap) reached the goal in 3.28 / 2.98 / 2.79 s, FAST 900 in 2.74-2.92 s
(3 of 3 clean). Motors that cannot keep up (a low battery) used to fall
24-42 mm behind the reference at FAST 800-900 in the simulator, and as the
curves follow the reference's distance they started early and cut inside.
The reference now slows down while the robot lags (`PATH_LAG_*`): on the
robot at FAST 900 with `TUNE MOTOR_SCALE 0.8` (the motors get 80 % of the
PWM, as with a LiPo at its cutoff) the run slowed to 81 % where needed,
stayed within 4.1 mm of the reference and took 2.84 s instead of 2.76, as
the simulator predicted (4.3 mm, 78 %). Defaults FAST 900, CURVE 480.

Search legs (straight on without stopping, the default): on the robot on
the practice maze and on layouts B and C (the 4x3 rearranged; goal (3,2)):
maps right; layout B 7.4 s against 8.5 s stopping in every cell, its speed
runs 1.48-1.69 s and clean, IR stops within 3.5 mm of the plan. 1-cell
legs left some sides doubtful (fixed by `SEARCH_SIDE_FROM_MM` 20, not yet
on the robot). Still to validate: long straights.

Layout C (2026-09-26; route 1D1I1D2, a staircase of three curves in
consecutive cells, then 2 cells along the north border): the speed run
drifted onto the north wall on the last straight, scraped it, and wedged
turning at the goal. Reproduced in step mode (IR at the goal): it arrived
yawed 5.5 deg and 24 mm off-centre. Cause: the start was 8-12 mm off-centre
(where the search's final 180 turn left it) and the centring's integral
learned that as ~5 deg of bias in the first cell (the simulator reproduces
3.3 deg for 10 mm), then aimed the last straight at the wall. With
`TUNE BIAS_WIN 0` (no integral) the same run arrived square, 17 mm off.
Fixed by the bias observer (not yet on the robot). Also measured: single
curves at 478 mm/s turn ~2.4 deg less than the encoders say (4.9 for a
right plus a left; single measurements scatter +-3 deg), and one exited
12 mm wide; at 300 mm/s they matched. Not compensated yet: re-measure with
`CAL RUN` on the staircase once the observer is on the robot.

Layout D (2026-09-26; route N 2 cells, E 3 to the goal): two searches with
`CONT ON` at 450 mm/s made the same 13 actions and the same map; every leg
(2 N, 3 E, 3 W, 2 S) decided its next cell in time and ended on the front
wall 0.3-0.5 mm short of its target, tracking error <= 1.7 mm / 2.7 deg.
