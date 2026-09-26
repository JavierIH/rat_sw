#!/usr/bin/env python3
"""Tests for calib_analyze.py with synthetic recordings of known physics.

The recordings go through robot_monitor.CalibrationCapture exactly as a
real '@D' dump would, so the whole CSV path is covered too.
"""
import math
import os
import random
import re
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import calib_analyze as ca  # noqa: E402
import robot_monitor as rm  # noqa: E402

FL_CAL = (-0.00000002278, 0.000132, -0.2627, 237.7)
INFO = [
    "@D INFO ticks_per_mm=9 cell_ticks=1620 move_extra_ticks=140 ticks_per_turn=430 turn_still_ms=20",
    "@D INFO spd=150 fast=220 turn=110 kp=2.00 kd=30.00 ke=0.00 pd_max=150 accel_step_per_ms=4",
    "@D INFO wall_detect_mm=140 front_ref_mm=94 front_emergency_mm=60 side_track_mm=130 lane_mm=168",
    '@D INFO ir_cal_fl="-0.00000002278f, 0.000132f,  -0.2627f, 237.7f"',
    '@D INFO ir_cal_fr="-0.00000003535f, 0.0001995f, -0.3834f, 317.6f"',
    '@D INFO ir_cal_sl="-0.00000005219f, 0.0002629f, -0.4566f, 325.6f"',
    '@D INFO ir_cal_sr="-0.00000003241f, 0.0001505f, -0.25f,   189.0f"',
]


def fl_raw_for(mm):
    """Raw ADC reading that the FL calibration maps to `mm` (bisection)."""
    lo, hi = 0.0, 4095.0
    for _ in range(60):
        mid = (lo + hi) / 2
        a, b, c, d = FL_CAL
        if ((a * mid + b) * mid + c) * mid + d > mm:
            lo = mid
        else:
            hi = mid
    return int(round((lo + hi) / 2))


def save(directory, description, period, rows, notes=()):
    """Feeds a firmware-style dump through the monitor's capture."""
    capture = rm.CalibrationCapture(directory)
    lines = ["@D BEGIN " + description,
             '@D INFO period_ms=%d samples=%d capacity=384 result=OK build="test"' % (period, len(rows))]
    lines += INFO + ["@D COLS t_ms,enc_l,enc_r,pwm_l,pwm_r,raw_fl,raw_fr,raw_sl,raw_sr"]
    lines += ["@D %d,%d,%d,%d,%d,%d,%d,%d,%d" % ((i * period,) + tuple(int(round(v)) for v in row))
              for i, row in enumerate(rows)]
    lines += ["@D END result=OK samples=%d" % len(rows)]
    for line in lines:
        capture.feed(line)
    for note in notes:
        capture.note(note)
    return capture.last_path


NEW_INFO = [
    "@D INFO ticks_per_mm=9.05 cell_mm=180 turn_ticks=400 kv_l=0.92 kv_r=0.90 tau_ms=50.00 ks=20",
    "@D INFO spd=400 fast=500 accel=3000 turn=500 turn_accel=5000 kp=1.00 ki=4.00",
] + INFO[2:]


def save_new(directory, description, period, rows, notes=()):
    """A dump from the speed-control firmware: two more columns, the reference."""
    capture = rm.CalibrationCapture(directory)
    lines = ["@D BEGIN " + description,
             '@D INFO period_ms=%d samples=%d capacity=320 result=OK build="test"' % (period, len(rows))]
    lines += NEW_INFO + ["@D COLS t_ms,enc_l,enc_r,pwm_l,pwm_r,raw_fl,raw_fr,raw_sl,raw_sr,ref_fwd,ref_rot"]
    lines += ["@D %d,%s" % (i * period, ",".join(str(int(round(v))) for v in row)) for i, row in enumerate(rows)]
    lines += ["@D END result=OK samples=%d" % len(rows)]
    for line in lines:
        capture.feed(line)
    for note in notes:
        capture.note(note)
    return capture.last_path


def number(pattern, text):
    m = re.search(pattern, text)
    if not m:
        raise AssertionError("%r not found in:\n%s" % (pattern, text))
    return float(m.group(1))


class TestCalibAnalyze(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()

    def tearDown(self):
        self.tmp.cleanup()

    def test_step_response(self):
        # 20 ms dead time, first order tau = 60 ms to 400 mm/s, brakes with tau 30 ms.
        period, pwm, on_ms, rows, pos, v = 2, 200, 500, [], 0.0, 0.0
        for i in range(400):
            t = i * period
            if t < on_ms:
                target = 400.0 if t >= 20 else 0.0
                v += (target - v) * (1 - math.exp(-period / 60.0)) if t >= 20 else 0.0
            else:
                v *= math.exp(-period / 30.0)
            pos += v * period / 1000.0 * 9
            p = pwm if t < on_ms else 0
            rows.append((pos, pos * 0.99, p, p, 0, 0, 0, 0))
        text = ca.report([save(self.tmp.name, "step %d %d" % (pwm, on_ms), period, rows)])
        self.assertAlmostEqual(number(r"velocidad estable (\d+) mm/s", text), 400, delta=15)
        self.assertAlmostEqual(number(r"ganancia ([\d.]+) mm/s", text), 2.0, delta=0.08)
        self.assertAlmostEqual(number(r"tiempo muerto (\d+) ms", text), 20, delta=6)
        self.assertAlmostEqual(number(r"constante de tiempo ~(\d+) ms", text), 60, delta=12)
        self.assertAlmostEqual(number(r"frenada: ([\d.]+) mm", text), 400 * 0.030, delta=3)
        self.assertAlmostEqual(number(r"\(([-+\d.]+)%\)", text), 1.0, delta=0.2)

    def test_motor_model_from_steps(self):
        # PWM = KV * v + KS per wheel: left KV 0.95 KS 30, right KV 0.88 KS 30, tau 50 ms.
        paths = []
        for pwm in (200, 400, 600):
            period, on_ms, rows, pl, pr, vl, vr = 2, 400, [], 0.0, 0.0, 0.0, 0.0
            for i in range(350):
                t = i * period
                on = t < on_ms
                tl = (pwm - 30) / 0.95 if on else 0.0
                tr = (pwm - 30) / 0.88 if on else 0.0
                k = 1 - math.exp(-period / 50.0)
                vl += (tl - vl) * k
                vr += (tr - vr) * k
                pl += vl * period / 1000.0 * 9.05
                pr += vr * period / 1000.0 * 9.05
                p = pwm if on else 0
                rows.append((pl, pr, p, p, 0, 0, 0, 0))
            paths.append(save(self.tmp.name, "step %d %d" % (pwm, on_ms), period, rows))
        text = ca.report(paths)
        self.assertAlmostEqual(number(r"MOTOR_KV_L +([\d.]+)f", text), 0.95, delta=0.03)
        self.assertAlmostEqual(number(r"MOTOR_KV_R +([\d.]+)f", text), 0.88, delta=0.03)
        self.assertAlmostEqual(number(r"MOTOR_KS_PWM +([\d.]+)f", text), 30, delta=8)
        self.assertAlmostEqual(number(r"MOTOR_TAU_S +([\d.]+)f", text), 0.050, delta=0.008)

    def test_controlled_straight(self):
        # 3 cells at 500 mm/s, 0.5 mm behind the reference, 12 mm left of centre
        # at the start and centred by the end; 543 mm measured.
        rows, ref, tpm = [], 0.0, 9.05
        n = 600
        for i in range(n):
            ref = min(540.0, ref + 500 * 0.002)
            pos = max(0.0, ref - 0.5)
            y = 12.0 * max(0.0, 1 - i / 150.0)
            sr = fl_raw_for(84 + y)      # the FL curve stands in for SR: only mm matter here
            on = ref < 540.0 or i < n - 20
            rows.append((pos * tpm, pos * tpm, 500 if on else 0, 500 if on else 0, 0, 0, 0, sr, ref * 10, 0))
        path = save_new(self.tmp.name, "straight 3 500", 2, rows, ["medido 543 mm"])
        with open(path) as f:
            text = f.read().replace('ir_cal_sr="-0.00000003241f, 0.0001505f, -0.25f,   189.0f"',
                                    'ir_cal_sr="-0.00000002278f, 0.000132f,  -0.2627f, 237.7f"')
        with open(path, "w") as f:
            f.write(text)
        text = ca.report([path])
        self.assertIn("con control de velocidad", text)
        self.assertAlmostEqual(number(r"error de avance max ([\d.]+) mm", text), 0.5, delta=0.1)
        self.assertAlmostEqual(number(r"inicio ([-+\d.]+) mm", text), 12, delta=1.5)
        self.assertAlmostEqual(number(r"2a mitad ([-+\d.]+) \+-", text), 0, delta=1.0)
        self.assertAlmostEqual(number(r"WHEEL_TICKS_PER_MM ([\d.]+)", text), 539.5 * tpm / 543, delta=0.02)

    def test_controlled_turns(self):
        # Four 90 deg turns, the encoders exactly on the reference; 352 deg measured.
        rows, tpm, mpd = [], 9.05, 400 / 90 / 9.05
        angle = 0.0
        for k in range(4):
            for i in range(60):
                angle = 90.0 * k + 90.0 * (i + 1) / 60
                half = angle * mpd * tpm
                rows.append((half, -half, 300, -300, 0, 0, 0, 0, 0, angle * 100))
            for i in range(20):
                half = angle * mpd * tpm
                rows.append((half, -half, 0, 0, 0, 0, 0, 0, 0, angle * 100))
        text = ca.report([save_new(self.tmp.name, "turn 4", 5, rows, ["angulo 352"])])
        self.assertEqual(len(re.findall(r"\+90\.00 grados de encoder", text)), 4, text)
        self.assertAlmostEqual(number(r"TURNTICKS (\d+)", text), 400 * 352 / 360, delta=2)

    def test_controlled_curve(self):
        # CAL CURVE 1 400: right curve, encoders on the reference. The robot
        # leaves the curve 5 mm left of centre (out wide), the wall at the end
        # stops it 3 mm before the plan, and it is 2 deg short of square.
        period, v, tpm, mpd = 2, 400.0, 9.05, 400 / 90 / 9.05
        radius, ramp, pre, post = 70.0, 30.0, 4.5, 4.5
        length = radius * math.pi / 2 + ramp
        start = 90 + pre
        planned = 180 + pre + length + post
        stop = planned - 3.0

        def progress(u):       # the clothoid-arc-clothoid of path.c, 0..1
            k = 1 / radius
            if u <= 0:
                return 0.0
            if u >= length:
                return 1.0
            if u < ramp:
                h = 0.5 * k * u * u / ramp
            elif u < length - ramp:
                h = k * (u - 0.5 * ramp)
            else:
                h = math.pi / 2 - 0.5 * k * (length - u) ** 2 / ramp
            return h / (math.pi / 2)

        rows, s, t = [], 0.0, 0
        while True:
            s = min(stop, v * t / 1000.0)
            heading = 90.0 * progress(s - start)
            half_rot = heading * mpd * tpm
            lateral = 0.0 if s < start + length else 5.0
            sr = fl_raw_for(84 + lateral)
            done = s >= stop
            front = (fl_raw_for(96.4), fl_raw_for(94.0)) if done else (fl_raw_for(300), fl_raw_for(300))
            p = 0 if done else 400
            rows.append((s * tpm + half_rot, s * tpm - half_rot, p, p) + front + (0, sr, s * 10, heading * 100))
            if done:
                break
            t += period
        rows += [rows[-1]] * 20
        info = ["@D INFO curve_r=70.0 curve_ramp=30.0 curve_angle=90.00 curve_len=%.1f curve_vmax=478" % length,
                "@D INFO curve_pre=4.5 curve_post=4.5 curve_pre_adj=0.0 curve_post_adj=0.0"]
        capture = rm.CalibrationCapture(self.tmp.name)
        lines = ["@D BEGIN curve 1 400", '@D INFO period_ms=2 samples=%d capacity=320 result=OK build="t"' % len(rows)]
        lines += NEW_INFO + info
        lines += ["@D COLS t_ms,enc_l,enc_r,pwm_l,pwm_r,raw_fl,raw_fr,raw_sl,raw_sr,ref_fwd,ref_rot"]
        lines += ["@D %d,%s" % (i * period, ",".join(str(int(round(x))) for x in row)) for i, row in enumerate(rows)]
        lines += ["@D END result=OK samples=%d" % len(rows)]
        for line in lines:
            capture.feed(line)
        with open(capture.last_path) as f:
            text = f.read()
        for sensor in ("fr", "sr"):     # the FL curve stands in for them: only mm matter here
            text = re.sub(r'ir_cal_%s="[^"]*"' % sensor, 'ir_cal_%s="-0.00000002278f, 0.000132f,  -0.2627f, 237.7f"'
                          % sensor, text)
        with open(capture.last_path, "w") as f:
            f.write(text)
        text = ca.report([capture.last_path])
        self.assertIn("Curva a la derecha a 400 mm/s", text)
        self.assertAlmostEqual(number(r"de rumbo en la curva max ([\d.]+)", text), 0.0, delta=0.2)   # one tick
        self.assertAlmostEqual(number(r"curva en (\d+) ms", text), 1000 * length / v, delta=2 * period)
        self.assertAlmostEqual(number(r"encoders: ([-+\d.]+) grados en la curva", text), 90.0, delta=0.2)
        self.assertAlmostEqual(number(r"([-+\d.]+) en todo el movimiento", text), 90.0, delta=0.1)
        self.assertAlmostEqual(number(r"lateral al salir: ([-+\d.]+) mm", text), 5.0, delta=0.6)
        self.assertIn("por fuera", text)
        self.assertAlmostEqual(number(r"TUNE CURVE_PRE ([-\d.]+)", text), -5.0, delta=0.6)
        self.assertAlmostEqual(number(r"TUNE CURVE_POST ([-\d.]+)", text), -3.0, delta=0.2)
        self.assertAlmostEqual(number(r"TUNE CURVE_ANGLE ([\d.]+)", text), 92.0, delta=0.4)

    def test_continuous_run(self):
        # CAL RUN over a cell, a right curve and 270 mm of straight at 500
        # mm/s. The robot leaves the curve 12 mm left of centre and ends 2 mm
        # off; the side IR report where it was 50 ms before.
        period, v, tpm, mpd = 4, 500.0, 9.05, 400 / 90 / 9.05
        radius, ramp = 70.0, 30.0
        length = radius * math.pi / 2 + ramp
        c0, c1 = 94.5, 94.5 + length
        stop = c1 + 4.5 + 270.0

        def heading_at(s):
            u = s - c0
            if u <= 0:
                return 0.0
            if u >= length:
                return 90.0
            k = 1 / radius
            if u < ramp:
                h = 0.5 * k * u * u / ramp
            elif u < length - ramp:
                h = k * (u - 0.5 * ramp)
            else:
                h = math.pi / 2 - 0.5 * k * (length - u) ** 2 / ramp
            return math.degrees(h)

        def lateral_at(s):
            return 0.0 if s < c1 else 12.0 - 10.0 * (s - c1) / (stop - c1)

        rows, t = [], 0
        while True:
            s = min(stop, v * t / 1000.0)
            seen = max(0.0, s - v * 0.05)
            walls = seen < c0 - 10 or seen > c1
            sl, sr = (fl_raw_for(84 - lateral_at(seen)), fl_raw_for(84 + lateral_at(seen))) if walls else (0, 0)
            half_rot = heading_at(s) * mpd * tpm
            done = s >= stop
            p = 0 if done else 400
            rows.append((s * tpm + half_rot, s * tpm - half_rot, p, p, fl_raw_for(300), fl_raw_for(300), sl, sr,
                         s * 10, heading_at(s) * 100))
            if done:
                break
            t += period
        capture = rm.CalibrationCapture(self.tmp.name)
        lines = ["@D BEGIN run", '@D INFO period_ms=%d samples=%d capacity=320 result=OK build="t"' % (period, len(rows))]
        lines += NEW_INFO + ["@D INFO curve_angle=90.00"]
        lines += ["@D COLS t_ms,enc_l,enc_r,pwm_l,pwm_r,raw_fl,raw_fr,raw_sl,raw_sr,ref_fwd,ref_rot"]
        lines += ["@D %d,%s" % (i * period, ",".join(str(int(round(x))) for x in row)) for i, row in enumerate(rows)]
        lines += ["@D END result=OK samples=%d" % len(rows)]
        for line in lines:
            capture.feed(line)
        with open(capture.last_path) as f:
            text = f.read()
        for sensor in ("fr", "sl", "sr"):
            text = re.sub(r'ir_cal_%s="[^"]*"' % sensor, 'ir_cal_%s="-0.00000002278f, 0.000132f,  -0.2627f, 237.7f"'
                          % sensor, text)
        with open(capture.last_path, "w") as f:
            f.write(text)
        text = ca.report([capture.last_path])
        self.assertIn("Movimiento continuo (OK)", text)
        self.assertAlmostEqual(number(r"(\d+) mm en \d+ ms", text), stop, delta=2)
        self.assertIn("recta 1", text)
        second = text[text.index("recta 2"):]
        self.assertIn("rumbo +90", second)
        self.assertAlmostEqual(number(r"lateral ([-+\d.]+) mm al leer", second), 12.0, delta=1.0)
        self.assertAlmostEqual(number(r"([-+\d.]+) al final, peor", second), 2.0, delta=1.0)
        self.assertAlmostEqual(number(r"peor ([-+\d.]+)", second), 12.0, delta=1.0)
        self.assertNotIn("recta 3", text)
        # The curve's ramps belong to the curve, not to the straights.
        self.assertAlmostEqual(number(r"recta 1: 0-(\d+) mm", text), c0, delta=3)
        self.assertAlmostEqual(number(r"recta 2: (\d+)-", text), c1, delta=3)
        self.assertIn("pidio +0.0..+0.0 grados", text[:text.index("recta 2")])

    def straight(self, cells, true_ticks_per_mm, coast):
        target = cells * 1620 + 140
        rows, pos = [], 0.0
        while pos < target:
            pos = min(target, pos + 300 * 9 * 5 / 1000.0)     # 300 mm/s, 5 ms samples
            rows.append((pos, pos, 150, 150, 0, 0, 0, 0))
        for k in range(1, 30):
            rows.append((pos + coast * (1 - math.exp(-k / 3.0)),) * 2 + (0, 0, 0, 0, 0, 0))
        final = pos + coast * (1 - math.exp(-29 / 3.0))
        measured = final / true_ticks_per_mm
        return save(self.tmp.name, "straight %d 150" % cells, 5, rows, ["medido %.1f mm" % measured])

    def test_straights_split_cell_and_move_calibration(self):
        # Wheels really do 9.5 ticks/mm and coast 60 ticks after braking: to
        # stop centred, N cells need N*1710 - 60 ticks at the brake point.
        one = self.straight(1, 9.5, 60)
        three = self.straight(3, 9.5, 60)
        text = ca.report([one, three])
        self.assertAlmostEqual(number(r"CELL_TICKS ~ (\d+)", text), 1710, delta=3)
        self.assertAlmostEqual(number(r"MOVE_EXTRA_TICKS ~ (-?\d+)", text), -60, delta=6)
        self.assertAlmostEqual(number(r"\((\d+) ticks de inercia", text), 60, delta=2)

    def test_turn_overshoot_and_angle(self):
        rows, rot = [], 0.0
        for _ in range(4):
            while rot % 455 < 430 or not rows:
                rot += 2
                rows.append((rot, -rot, 110, -110, 0, 0, 0, 0))
                if rot % 455 >= 430:
                    break
            for k in range(1, 31):
                rows.append((rot + 25 * min(1, k / 5.0),) + (-(rot + 25 * min(1, k / 5.0)),) + (0,) * 6)
            rot += 25
        text = ca.report([save(self.tmp.name, "turn 4", 5, rows, ["angulo 380"])])
        self.assertEqual(len(re.findall(r"sobregiro 25\)", text)), 4, text)
        self.assertAlmostEqual(number(r"TICKS_PER_TURN sugerido (\d+)", text), 90 * 4 * 455 / 380 - 25, delta=3)

    def test_ir_sweep_recovers_the_curve(self):
        start, rows = 40, []
        for i in range(200):
            back = min(200, i * 1.2)
            raw = fl_raw_for(start + back)
            rows.append((-back * 9, -back * 9, -80, -80, raw, raw, 0, 0))
        text = ca.report([save(self.tmp.name, "ir 200", 10, rows, ["inicio 40 mm"])])
        self.assertLess(number(r"FL ajuste nuevo: error medio ([\d.]+) mm", text), 1.0)
        self.assertIn("#define CAL_FL", text)

    def test_noise_statistics(self):
        rng = random.Random(1)
        rows = [(0, 0, 0, 0, 1000 + rng.gauss(0, 5), 800 + rng.gauss(0, 2), 900, 60) for _ in range(200)]
        text = ca.report([save(self.tmp.name, "noise 2000", 10, rows)])
        self.assertAlmostEqual(number(r"FL raw\s+([\d.]+)", text), 1000, delta=1.5)
        self.assertAlmostEqual(number(r"FL raw\s+[\d.]+ \+-\s+([\d.]+)", text), 5, delta=1)
        self.assertIn("encoders: quietos", text)

    def test_square_offset_from_a_wall(self):
        # Facing a wall square: FL reads ~100 mm, FR ~112 mm.
        rows = [(0, 0, 0, 0, fl_raw_for(100), 700, 900, 60) for _ in range(50)]
        path = save(self.tmp.name, "noise 500", 10, rows)
        fr_mm = ca.load(path).ir_mm("fr", 700)
        text = ca.report([path])
        self.assertAlmostEqual(number(r"FL-FR = ([-+\d.]+) mm", text), 100 - fr_mm, delta=0.6)
        self.assertIn("#define FRONT_SQUARE_OFFSET_MM", text)

    def test_missing_measurements_ask_for_notes(self):
        rows = [(i * 10, i * 10, 150, 150, 0, 0, 0, 0) for i in range(50)] + [(500, 500) + (0,) * 6]
        text = ca.report([save(self.tmp.name, "straight 1 150", 5, rows)])
        self.assertIn("/nota medido", text)


if __name__ == "__main__":
    unittest.main()
