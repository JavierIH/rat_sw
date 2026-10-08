# Measurements behind robot_config.h

The numbers behind the constants that docs/design.md does not already give
(robot_config.h keeps one line per constant).

- Odometry: `WHEEL_TICKS_PER_MM` 9.05 from 1-, 2- and 3-cell straights
  measured with a ruler (4885 ticks ~ 540 mm). `TICKS_PER_TURN`: with the
  speed control 400 turned 355.3 deg per 360, 405 turned 360.8 right /
  359.4 left and the side readings did not move after 8 turns; at 403 the
  90s end ~0.4 deg short.
- IR delay: at 400 mm/s the front wall read 18 mm farther than it was;
  the position 50 ms earlier plus the reading matched it within 0.6 mm
  over a whole approach (`CAL STRAIGHT 3 400`). 22 since 10-08: 8
  approaches of `CAL STRAIGHT 3`, two at each of 100, 300, 600 and 900
  mm/s (`calib_analyze.py --delay`), gave FL 16-24 / FR 24-32 ms at 600-900, the same ms at both
  speeds (a delay, not a slide or an offset; at 100-300 a 2 mm calibration
  error scatters them to 8-58); in-place turns 0-8 ms on all four. With
  `TUNE IR_DELAY 22` the stops read 92-93 (91.5-94.5 at 50) and the
  reference ends ~40 ms sooner. `FRONT_TRACK_MM` 170:
  tracking from 140, at 500 mm/s the robot learnt about the wall with ~20
  mm left, too late to brake if it was not where planned; beyond 150 the
  readings err short (the wall looks closer), so it brakes a bit early and
  the target recedes as they improve. `FRONT_TRACK_REF_MM` 92: aiming at
  94 (`FRONT_WALL_REF_MM`) itself ended 2-4 mm long (a front realignment
  in 5 of a search's stops); at 92 the stops read 93-94.
- Side centre readings, before the 09-27 rounds: 180 deg pairs gave SR
  87/65 and SL 76/101 (twice alike); centring on 84 kept the robot ~8 mm
  towards the left wall; hand-centred readings SR 76-80, SL 84-89.
- Search legs (set with the old 50 ms delay; at 22 the margin is wider,
  the cap stays: robust before fast): the front reads reliably under ~170
  mm, which with the 50 ms delay comes ~14 mm + 50 ms of travel into the cell (a wall reads ~135
  at the decision, none over `SEARCH_FRONT_OPEN_MM`). Up to ~500 mm/s that
  is before the decision; 450 leaves ~12 mm (25 ms) to spare. Faster, the
  robot would decide blind and brake harder at walls (at 600, ~4100
  mm/s^2; the wheels slipped at 5000).
- `FRONT_SQUARE_OFFSET_MM` -15: median of 44 IR stops on the practice maze
  (sd 5.7 mm), confirmed with `CAL NOISE` (-15.4, -16.2). `SIDE_YAW_DOUBT_MM`
  10 is ~8 deg of yaw.
- Motor model (`CAL STEP` 200/400/600, open loop): PWM = KV v + KS, first
  order, `TAU` 34-79 ms (shorter at higher PWM), no dead time; both wheels
  alike (393/393 mm/s at PWM 400). The battery changes KV; the loops absorb
  it. KS: 100 PWM ran at 73 mm/s, turns cruise 25 PWM above KV v.
- Loop gains, chosen on the simulated robot: ~5 Hz bandwidth, damping
  ~0.7, tolerant of +-20 % in KV and TAU; without the D terms it
  oscillated. `ROT_KP` 20 let the heading stick until 2-4 deg of error and
  then jump (S-curves); 40/0.6 on the robot: heading oscillation 0.83 ->
  0.35 deg rms, tracking error 3.9 -> 1.5 deg max; `ROT_KD` 0.8 took the
  lateral noise at 700 mm/s from +-2.0 to +-0.8 mm. Settling: 70 PWM never
  moved the wheels, 100 did; the integrals build that push within ~50 ms.
  `SETTLE_DEG` 0.5 waited 200 ms more per turn.
- Centring: `STEER_MAX_DEG` 8 could not even get parallel. The side IR
  step ~2.4 mm every ~16 ms and the heading follows its target ~50 ms
  late: KP 1 with 0.35 deg/mm (`STEER_CURVE_DEG_PER_MM`) and a 16 ms
  average made fast small S-curves (~5.5 Hz, +-2 deg); 0.2 and 32 ms are
  gentler (the simulator with a slower heading response agrees: heading
  error 3.8 -> 1.5 deg); 0.4 since 10-05 (with 0.2 a robot 35 deg crooked
  reached the wall: `docs/faults/centring.md`). `STEER_VREF_MM_S`: at 700 mm/s KP 0.7 weaved (1.0
  deg rms, 4 Hz), 0.5 did not (0.67), the same as KP 0.7 at 400. KP 1.0
  also weaved with `ROT_KP` 20.
- Curves at 400 mm/s: 2.3 m/s^2 sideways (braking at 5 slipped, 3 was near
  the grip limit), 4400 deg/s^2 at the ramps (in-place turns use 5000),
  outer wheel at 560 mm/s. The robot follows the reference a few ms late,
  so a curve may need to start earlier (`CURVE_PRE` < 0: exits displaced to
  the outside). `CURVE_PWM_SHARE`: the outer wheel's feedforward peaks where
  a ramp meets the arc (fastest and still accelerating); in the simulator
  500 mm/s still tracked within 1 mm, 600 saturated for a moment.
  `SQUARE_MM_PER_DEG` 1.2: 1.07-1.16 mm per tick over 4 turns in the
  `TURNTICKS` calibration.
- Planner costs over 600 random mazes on the path reference (`host_tests
  --costs`). `OPTIMIZE_MAX_STEPS`: one simulated 16x16 maze in 100 needed
  more than 300 (docs/testing.md for the 520 real ones).
- Parameter defaults. SPD: practice-maze search + return 22.3 s at 400
  mm/s, 20.3 s at 500 (a 10 ms pause before sensing), 19.4 s at 600 (no
  pause), same map every time (1-cell moves
  barely cruise at 3000 mm/s^2). FAST: on 3-cell straights 700 stopped
  within ~2 mm of the front-wall reference and centred (0.98 s), 900 too
  (0.92 s) but out of PWM at the end of the acceleration (the motor model
  holds ~937 mm/s); practice-maze speed runs at 900 with CURVE 480: 3 of 3
  clean, within 2.4 mm / 3.1 deg. With motors 20 % weaker (a low battery)
  900 is out of PWM: the path's reference slows to the robot's pace
  (`PATH_LAG_*`), within 4.3 mm of it in the simulator (24 mm, and curves
  cut 47 mm inside, without). CURVE: practice-maze speed runs (9 cells, 4
  curves, a U-turn) 400 in 3.28 s, 450 in 2.98 s, 480 (478) in 2.79 s,
  within 2.5 mm and 1.9 / 2.7 / 3.1 deg.
- Lighting check (`LIGHT_*`, `docs/lighting.md`), from the 74 `CAL NOISE`
  recordings at home: stopped at a wall the raw readings' standard
  deviation is 4-8 counts (median), 16 at worst, i.e. 1.9 mm at worst on
  the centre line: `LIGHT_NOISE_MAX_MM` 4, twice that. Open space reads
  20-30 counts: SR then reads ~182 mm, 42 beyond `WALL_DETECT_MM`; the
  margin `LIGHT_OPEN_MARGIN_MM` 20 leaves room for ~100 counts of extra
  light on SR before it calls for a correction. `LIGHT_OK_MM` 3: a first
  guess (the cubics and centres agree within ~2 mm at home), to confirm
  with validation 1.
