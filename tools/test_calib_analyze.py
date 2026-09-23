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
    "@D INFO ticks_per_mm=9 cell_ticks=1620 move_extra_ticks=140 ticks_per_turn=430 turn_settle_ms=150",
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
