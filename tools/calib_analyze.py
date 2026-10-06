#!/usr/bin/env python3
"""Summarise calibration recordings from the robot (CAL commands).

    python3 tools/calib_analyze.py tools/calib_data/*.csv

Each CSV (saved by robot_monitor.py) carries the test, every relevant
firmware constant and the samples. Physical measurements added with
/note in the monitor turn the report into concrete suggestions:

    straight:  /note measured 176 mm      distance actually travelled
    turn:      /note angle 352            total angle actually turned (degrees)
    ir:        /note start 40 mm          front sensors to wall when the sweep starts

CAL CURVE (a cell, a smooth curve, a cell) needs no notes: the side walls
after the curve, the front wall at the end and FL-FR there suggest
CURVE_PRE, CURVE_POST and CURVE_ANGLE.

Recordings from the speed-control firmware carry the profile reference
(ref_fwd, ref_rot): straights and turns then report how closely the wheels
followed it, the centring and the real distance/angle. Open-loop steps
(CAL STEP) at two or more PWMs give the motor model constants
(MOTOR_KV_L/R, MOTOR_KS_PWM, MOTOR_TAU_S). Older recordings (no reference)
are still analysed as before.

    python3 tools/calib_analyze.py --chain tools/calib_data/2026-09-26_20-*.csv

follows the heading over a whole session of moves (the ring, layout F):
how far the encoders drift from the robot's real heading, move by move.
"""
import argparse
import math
import os
import re
import sys

CELL_MM = 180
SENSORS = ("fl", "fr", "sl", "sr")
IR_DELAY_MS = 50            # as in robot_config.h: the IR report the robot's past
SQUARE_MM_PER_DEG = 1.2     # as in robot_config.h: FL - FR per degree of yaw
# Side readings move ~1 mm per degree of yaw (sensors at the nose); fitted on the ring, 2026-09-26.
SIDE_LEVER_MM = 55.0        # per radian


# ---- Loading ----------------------------------------------------------------------------

class Recording:
    def __init__(self, path, test, meta, notes, data):
        self.path, self.test, self.meta, self.notes, self.data = path, test, meta, notes, data
        words = (test or "").split()
        self.kind = words[0] if words else "?"
        self.args = [int(w) for w in words[1:] if re.match(r"^-?\d+$", w)]
        self.period = self.number("period_ms", 1)
        self.n = len(data.get("t_ms", []))

    def number(self, key, default=None):
        try:
            return float(self.meta[key])
        except (KeyError, ValueError):
            return default

    def note_value(self, *words):
        """First number after any of `words` in the notes (e.g. 'measured 176 mm')."""
        for note in self.notes:
            for word in words:
                m = re.search(word + r"\D*?(-?\d+(?:[.,]\d+)?)", note, re.IGNORECASE)
                if m:
                    return float(m.group(1).replace(",", "."))
        return None

    def ir_mm(self, sensor, raw):
        coefficients = self.meta.get("ir_cal_" + sensor)
        if not coefficients:
            return None
        a, b, c, d = (float(v.strip().rstrip("fF")) for v in coefficients.split(","))
        mm = ((a * raw + b) * raw + c) * raw + d
        return min(400.0, max(0.0, mm))


def parse_tokens(text):
    """key=value pairs; a value may be "quoted, with spaces"."""
    return {key: quoted if raw.startswith('"') else raw
            for key, raw, quoted in re.findall(r'(\w+)=("([^"]*)"|\S+)', text)}


def load(path):
    meta, notes, test, columns, rows = {}, [], None, None, []
    with open(path, encoding="utf-8") as f:
        for line in f:
            line = line.rstrip("\n")
            if line.startswith("#"):
                body = line[1:].strip()
                if body.startswith("note:"):
                    notes.append(body[5:].strip())
                elif body.startswith("test:"):
                    test = body[5:].strip()
                else:
                    meta.update(parse_tokens(body))
            elif columns is None:
                columns = line.split(",")
            elif line.strip():
                rows.append([int(v) for v in line.split(",")])
    data = {c: [r[i] for r in rows] for i, c in enumerate(columns or [])}
    return Recording(path, test, meta, notes, data)


# ---- Helpers ------------------------------------------------------------------------------

def mean(values):
    return sum(values) / len(values) if values else float("nan")


def stdev(values):
    if len(values) < 2:
        return 0.0
    m = mean(values)
    return math.sqrt(sum((v - m) ** 2 for v in values) / (len(values) - 1))


def average_ticks(rec):
    return [(l + r) / 2 for l, r in zip(rec.data["enc_l"], rec.data["enc_r"])]


def speed_mm_s(rec, ticks, smooth=3):
    """Central-difference speed, smoothed over `smooth` samples each side."""
    tpm = rec.number("ticks_per_mm", 9)
    n = len(ticks)
    out = []
    for i in range(n):
        a, b = max(0, i - smooth), min(n - 1, i + smooth)
        dt = (b - a) * rec.period
        out.append((ticks[b] - ticks[a]) / dt * 1000.0 / tpm if dt else 0.0)
    return out


def motor_on(rec, i):
    return rec.data["pwm_l"][i] != 0 or rec.data["pwm_r"][i] != 0


def stop_index(rec):
    """First sample with both motors off after they had been on. The cut
    happened between it and the previous sample, the last one driving."""
    started = False
    for i in range(rec.n):
        if motor_on(rec, i):
            started = True
        elif started:
            return i
    return rec.n - 1


def fit_cubic(xs, ys):
    """Least-squares y = a x^3 + b x^2 + c x + d (normal equations, no numpy)."""
    rows = [[x ** 3, x ** 2, x, 1.0] for x in xs]
    ata = [[sum(r[i] * r[j] for r in rows) for j in range(4)] for i in range(4)]
    aty = [sum(r[i] * y for r, y in zip(rows, ys)) for i in range(4)]
    m = [ata[i] + [aty[i]] for i in range(4)]
    for col in range(4):
        pivot = max(range(col, 4), key=lambda r: abs(m[r][col]))
        m[col], m[pivot] = m[pivot], m[col]
        if abs(m[col][col]) < 1e-30:
            raise ValueError("not enough data to fit")
        for r in range(4):
            if r != col:
                f = m[r][col] / m[col][col]
                m[r] = [a - f * b for a, b in zip(m[r], m[col])]
    return [m[i][4] / m[i][i] for i in range(4)]


# ---- Analyses -----------------------------------------------------------------------------

def analyze_noise(rec, out):
    out.append("Sensor noise with the robot still (%d samples)" % rec.n)
    for s in SENSORS:
        raw = rec.data["raw_" + s]
        mm = [rec.ir_mm(s, r) for r in raw]
        line = "  %s raw %7.1f +- %5.1f (min %4d max %4d)" % (s.upper(), mean(raw), stdev(raw), min(raw), max(raw))
        if mm[0] is not None:
            line += "  ->  %6.1f +- %4.2f mm" % (mean(mm), stdev(mm))
        out.append(line)
    moved = max(max(abs(v) for v in rec.data["enc_l"]), max(abs(v) for v in rec.data["enc_r"]))
    out.append("  encoders: %s" % ("still" if moved == 0 else "moved %d ticks (vibration?)" % moved))
    worst = max(stdev([rec.ir_mm(s, r) or 0 for r in rec.data["raw_" + s]]) for s in SENSORS)
    if worst > 3:
        out.append("  ! a sensor swings more than 3 mm: check the wiring or raise the averaging (IR_OVERSAMPLE)")
    fl = [rec.ir_mm("fl", r) for r in rec.data["raw_fl"]]
    fr = [rec.ir_mm("fr", r) for r in rec.data["raw_fr"]]
    if fl[0] is not None and fr[0] is not None and max(fl) < 140 and max(fr) < 140:
        # Only meaningful if the robot was square to a wall, centred in its cell.
        offset = mean(fl) - mean(fr)
        out.append("  FL-FR = %+.1f mm. If the robot was square to a wall and centred in the cell:" % offset)
        out.append("     #define FRONT_SQUARE_OFFSET_MM  %d   (now %s)"
                   % (round(offset), rec.meta.get("front_square_offset_mm", "?")))


def rest_means(rec):
    """Mean mm of every sensor over a recording at rest, or None without calibration."""
    means = {}
    for s in SENSORS:
        mm = [rec.ir_mm(s, r) for r in rec.data["raw_" + s]]
        if None in mm:
            return None
        means[s] = mean(mm)
    return means


def front_mean(means):
    """FL and FR averaged when both see a wall, else None."""
    return (means["fl"] + means["fr"]) / 2 if max(means["fl"], means["fr"]) < 140 else None


def mirror_centres(rec, before, after, out):
    """A 180 deg turn in place mirrors the robot across the lane's centre line,
    so each side sensor's readings before and after it average to what it
    reads with the robot centred (SIDE_CENTER_L/R_MM), wherever it stood."""
    slb, srb, sla, sra = before["sl"], before["sr"], after["sl"], after["sr"]
    out.append("  180 deg turn from the previous CAL NOISE: SL %.1f -> %.1f, SR %.1f -> %.1f mm"
               % (slb, sla, srb, sra))
    track = rec.number("side_track_mm", 130)
    if max(slb, srb, sla, sra) >= track:
        out.append("  ! a side wall is missing (> %.0f mm): repeat it in a corridor with walls on both sides" % track)
        return
    ref = rec.number("front_ref_mm", 94)
    for means in (before, after):
        front = front_mean(means)
        if front is not None and abs(front - ref) > 5:
            out.append("  ! the robot is %+.0f mm off the cell centre front to back: facing a side the side beams land"
                       " by the post and the pair is no good; centre it or use the round of 4 quarter turns"
                       % (front - ref))
    off_r, off_l = (srb - sra) / 2, (sla - slb) / 2
    out.append("  off-centre before the turn: %+.1f mm by SR, %+.1f by SL (> 0: to the left)"
               % (off_r, off_l))
    if abs(off_r - off_l) > 3:
        out.append("  ! they disagree: the turn was not in place, a beam hit a post or a sensor's slope is off")
    cl, cr = (slb + sla) / 2, (srb + sra) / 2
    out.append("  centring: SL %.1f, SR %.1f mm -> TUNE CENTER_L %.1f, TUNE CENTER_R %.1f (now %s / %s;"
               % (cl, cr, cl, cr, rec.meta.get("center_l", "?"), rec.meta.get("center_r", "?")))
    out.append("     a turn back, CAL TURN -2 + CAL NOISE, averages out the turns' angle error)")


def side_round(rec, stations, out, points):
    """CAL NOISE at four headings a quarter turn apart (CAL TURN 1 between) in
    a cell closed on three sides: facing the side walls, the front sensors
    (calibrated against the encoders) give the robot's offset to the right of
    the first heading, x = (F_left - F_right) / 2, whatever their offset; at
    the first heading and its opposite the side sensors read at +x and -x.
    A heading whose beams land by a post (the robot off the cell's centre
    front to back, seen by the front sensors) is left out."""
    right, left = front_mean(stations[1]), front_mean(stations[3])
    if right is None or left is None:
        out.append("  Round of 4 quarter turns: ! a wall is missing facing a side; do it in a cell closed on 3 sides")
        return
    x = (left - right) / 2
    out.append("  Round of 4 quarter turns: robot %+.1f mm right of the centre (front sensors: right %.1f, left %.1f)"
               % (x, right, left))
    ref, keep = rec.number("front_ref_mm", 94), {0: True, 2: True}
    for q in (0, 2):
        front = front_mean(stations[q])
        if front is not None and abs(front - ref) > 5:
            keep[q if front < ref else 2 - q] = False
    for q, offset in ((0, x), (2, -x)):
        m = stations[q]
        if not keep[q]:
            out.append("   heading %3d: SL %.1f, SR %.1f mm, dropped (the robot is not centred front to back:"
                       " the beams land by the post)" % (q * 90, m["sl"], m["sr"]))
        elif max(m["sl"], m["sr"]) >= rec.number("side_track_mm", 130):
            out.append("   heading %3d: a side wall is missing" % (q * 90))
        else:
            out.append("   heading %3d: SL %.1f, SR %.1f mm with the robot at %+.1f mm" % (q * 90, m["sl"], m["sr"], offset))
            points.append((offset, m["sl"], m["sr"]))


def side_fit(rec, points, out):
    """Side readings against the offset measured by the front sensors: the
    slope says whether each sensor's curve follows the robot's sideways
    motion (1), the value at x = 0 is its centred reading."""
    xs = [p[0] for p in points]
    if len(points) < 2 or max(xs) - min(xs) < 10:
        out.append("Side sensors: points at least 10 mm apart are needed (move the robot sideways between rounds)")
        return
    kl, cl = fit_line(xs, [p[1] for p in points])
    kr, cr = fit_line(xs, [p[2] for p in points])
    worst = max(max(abs(sl - (cl + kl * x)), abs(sr - (cr + kr * x))) for x, sl, sr in points)
    old_l, old_r = rec.number("center_l", cl), rec.number("center_r", cr)
    out.append("Side sensors from %d points, robot from %+.1f to %+.1f mm: SL = %.1f %+.3f x, SR = %.1f %+.3f x"
               " (residuo max %.1f mm)" % (len(points), min(xs), max(xs), cl, kl, cr, kr, worst))
    out.append("  slope SL %.2f, SR %.2f (1: the curve follows the motion; more than 10 %% off,"
               " recalibrate that sensor's curve)" % (kl, -kr))
    out.append("  centring: TUNE CENTER_L %.1f, TUNE CENTER_R %.1f (now %.1f / %.1f): with both walls the"
               " robot would centre %.1f mm more to the right" % (cl, cr, old_l, old_r, ((cl - old_l) - (cr - old_r)) / 2))


def controlled(rec):
    """Recorded by the speed-control firmware (profile reference present)."""
    return "ref_fwd" in rec.data


def side_errors(rec, upto):
    """Lateral error (mm, > 0 = left of centre) as the centring sees it: both
    walls averaged when both are in range, else the one there is."""
    lane, track = rec.number("lane_mm", 168), rec.number("side_track_mm", 130)
    center_l, center_r = rec.number("center_l", lane / 2), rec.number("center_r", lane / 2)
    out = []
    for i in range(upto):
        sl = rec.ir_mm("sl", rec.data["raw_sl"][i])
        sr = rec.ir_mm("sr", rec.data["raw_sr"][i])
        errors = []
        if sr is not None and sr < track:
            errors.append(sr - center_r)
        if sl is not None and sl < track:
            errors.append(center_l - sl)
        if errors:
            out.append((i, mean(errors)))
    return out


def centring_profile(rec, errors, fwd, step_mm=45, band_mm=1.5):
    """The lateral error by distance (the mean in every step_mm, placed where
    the robot was IR_DELAY_MS before the reading) and the zero crossings with
    a +-band_mm hysteresis: a weave crosses, sensor noise around zero does not."""
    lag = int(round(IR_DELAY_MS / rec.period))
    bins = {}
    for i, e in errors:
        bins.setdefault(int(fwd[max(i - lag, 0)] // step_mm), []).append(e)
    crossings, side = 0, 0
    for _, e in errors:
        s = 1 if e > band_mm else -1 if e < -band_mm else 0
        if s and side and s != side:
            crossings += 1
        side = s or side
    return ("per %d mm: %s | crossings >%.1f mm: %d"
            % (step_mm, " ".join("%+.0f" % mean(bins[k]) for k in sorted(bins)), band_mm, crossings))


def analyze_straight_controlled(rec, out):
    cells = rec.args[0] if rec.args else 1
    speed = rec.args[1] if len(rec.args) > 1 else rec.number("spd")
    tpm = rec.number("ticks_per_mm", 9.05)
    mpd = rec.number("turn_ticks", 400) / 90 / tpm     # wheel mm per degree
    ticks = average_ticks(rec)
    fwd = [t / tpm for t in ticks]
    rot = [(l - r) / 2 / tpm / mpd for l, r in zip(rec.data["enc_l"], rec.data["enc_r"])]
    ref_f = [v / 10.0 for v in rec.data["ref_fwd"]]
    ref_r = [v / 100.0 for v in rec.data["ref_rot"]]
    on = [i for i in range(rec.n) if motor_on(rec, i)]
    end = (on[-1] + 1) if on else rec.n
    out.append("Straight of %d cell(s) at %s mm/s with speed control (%s)"
               % (cells, int(speed) if speed is not None else "?", rec.meta.get("result", "?")))
    out.append("  travelled %.1f mm (reference %.1f, plan %d) in %.0f ms"
               % (fwd[-1], ref_f[end - 1], cells * CELL_MM, (end - (on[0] if on else 0)) * rec.period))
    v = speed_mm_s(rec, ticks, smooth=2)
    if on:
        out.append("  top speed %.0f mm/s" % max(v[i] for i in on))
    ef = [ref_f[i] - fwd[i] for i in on]
    er = [ref_r[i] - rot[i] for i in on]
    if ef:
        out.append("  tracking: forward error max %.2f mm (mean %+.2f), heading max %.2f deg"
                   % (max(abs(e) for e in ef), mean(ef), max(abs(e) for e in er)))
    pwm = [max(abs(rec.data["pwm_l"][i]), abs(rec.data["pwm_r"][i])) for i in on]
    saturated = sum(1 for p in pwm if p >= 1000)
    if saturated:
        out.append("  ! PWM at the limit in %d samples of %d: no margin, lower the speed or ACCEL" % (saturated, len(pwm)))
    errors = side_errors(rec, end)
    if len(errors) > 5:
        values = [e for _, e in errors]
        half = values[len(values) // 2:]
        crossings = sum(1 for a, b in zip(values, values[1:]) if (a < 0) != (b < 0))
        seconds = len(values) * rec.period / 1000.0
        out.append("  centring: start %+.1f mm, 2nd half %+.1f +- %.1f mm, %.1f crossings/s"
                   % (values[0], mean(half), stdev(half), crossings / seconds if seconds else 0))
        out.append("  " + centring_profile(rec, errors, fwd))
        heading = [ref_r[i] for i, _ in errors]
        out.append("  heading asked by the centring: %+.1f .. %+.1f deg" % (min(heading), max(heading)))
    out.append("  turn the encoders measured at the end: %+.2f deg" % rot[-1])
    measured = rec.note_value("measured")
    if measured:
        out.append("  measured %.0f mm -> WHEEL_TICKS_PER_MM %.3f (now %.2f)" % (measured, ticks[-1] / measured, tpm))
    else:
        out.append("  (note the real distance with: /note measured <mm> mm)")


def analyze_turn_controlled(rec, out):
    quarters = rec.args[0] if rec.args else 4
    tpm = rec.number("ticks_per_mm", 9.05)
    tt = rec.number("turn_ticks", 400)
    mpd = tt / 90 / tpm
    rot = [(l - r) / 2 / tpm / mpd for l, r in zip(rec.data["enc_l"], rec.data["enc_r"])]
    ref_r = [v / 100.0 for v in rec.data["ref_rot"]]
    fwd = [t / tpm for t in average_ticks(rec)]
    segments, start = [], None
    for i in range(rec.n):
        on = motor_on(rec, i)
        if on and start is None:
            start = i
        elif not on and start is not None:
            segments.append((start, i))
            start = None
    if start is not None:
        segments.append((start, rec.n))
    out.append("Turn of %d quarters with speed control (%s), %d turns found"
               % (abs(quarters), rec.meta.get("result", "?"), len(segments)))
    for k, (a, b) in enumerate(segments):
        rest = rot[a - 1] if a > 0 else 0.0
        ref_rest = ref_r[a - 1] if a > 0 else 0.0
        err = max(abs((ref_r[i] - ref_rest) - (rot[i] - rest)) for i in range(a, b))
        out.append("  turn %d: %+.2f encoder deg in %.0f ms, tracking error max %.2f deg"
                   % (k + 1, rot[b - 1] - rest, (b - a) * rec.period, err))
    out.append("  displacement of the centre: %.1f mm" % max(abs(f) for f in fwd))
    angle = rec.note_value("angle")
    if angle:
        total = abs(rot[-1])
        out.append("  measured %.0f real deg for %.0f encoder deg: TURNTICKS %.0f (now %.0f)"
                   % (angle, total, tt * angle / total if total else tt, tt))
    else:
        out.append("  (note the real angle turned with: /note angle <deg>, or compare FL-FR with CAL NOISE)")


def curve_encoder_angle(rec, speed):
    """Encoder degrees of each curve: CURVE_ANGLE plus the slip compensation
    at the curve's speed (curve_slip at 480 mm/s, growing as v^2)."""
    speed = min(speed, rec.number("curve_vmax", speed))
    return rec.number("curve_angle", 90.0) + rec.number("curve_slip", 0.0) * (speed / 480.0) ** 2


def analyze_curve_controlled(rec, out):
    """CAL CURVE: straight into the next cell, a smooth curve in it, stop at
    the centre of the cell after it."""
    direction = 1 if not rec.args or rec.args[0] >= 0 else -1
    speed = rec.args[1] if len(rec.args) > 1 else rec.number("curve")
    tpm = rec.number("ticks_per_mm", 9.05)
    mpd = rec.number("turn_ticks", 400) / 90 / tpm
    angle = curve_encoder_angle(rec, speed)
    fwd = [t / tpm for t in average_ticks(rec)]
    rot = [(l - r) / 2 / tpm / mpd for l, r in zip(rec.data["enc_l"], rec.data["enc_r"])]
    ref_f = [v / 10.0 for v in rec.data["ref_fwd"]]
    ref_r = [v / 100.0 for v in rec.data["ref_rot"]]
    on = [i for i in range(rec.n) if motor_on(rec, i)]
    end = (on[-1] + 1) if on else rec.n
    out.append("Curve to the %s at %s mm/s with speed control (%s)"
               % ("right" if direction > 0 else "left", int(speed) if speed is not None else "?",
                  rec.meta.get("result", "?")))
    if "curve_len" in rec.meta:
        out.append("  shape: radius %s, ramps %s mm: %s mm of curve, straight before %s and after %s mm, up to %s mm/s"
                   % tuple(rec.meta.get(k, "?") for k in ("curve_r", "curve_ramp", "curve_len", "curve_pre",
                                                          "curve_post", "curve_vmax")))
    # The curve from the distance travelled (the rotation reference also carries the centring's offset).
    s0 = CELL_MM / 2 + rec.number("curve_pre", 0.0)
    s1 = s0 + rec.number("curve_len", 0.0)
    turning = [i for i in range(end) if s0 < ref_f[i] < s1]
    if not turning or "curve_len" not in rec.meta:
        out.append("  the reference never curved")
        return
    a, b = turning[0], turning[-1] + 1
    v = speed_mm_s(rec, average_ticks(rec), smooth=2)
    out.append("  curve in %.0f ms at %.0f-%.0f mm/s" % ((b - a) * rec.period, min(v[a:b]), max(v[a:b])))
    ef = [ref_f[i] - fwd[i] for i in on]
    er = [ref_r[i] - rot[i] for i in range(a, b)]
    if ef:
        out.append("  tracking: forward error max %.2f mm, heading in the curve max %.2f deg"
                   % (max(abs(e) for e in ef), max(abs(e) for e in er)))
    pwm = [max(abs(rec.data["pwm_l"][i]), abs(rec.data["pwm_r"][i])) for i in on]
    saturated = sum(1 for p in pwm if p >= 1000)
    if saturated:
        out.append("  ! PWM at the limit in %d samples of %d: lower CURVE" % (saturated, len(pwm)))
    out.append("  encoders: %+.2f deg in the curve, %+.2f over the whole move (asked %+.2f); centring"
               " held in the curve %+.2f" % (rot[b] - rot[a], rot[end - 1], direction * angle, ref_r[a]))

    # Sideways, once the side readings come from the exit corridor (IR_DELAY_MS after the curve).
    errors = dict(side_errors(rec, end))
    first = b + int(math.ceil(IR_DELAY_MS / rec.period))
    before = [errors[i] for i in range(max(0, a - 10), a) if i in errors]
    after = [errors[i] for i in range(first, min(end, first + max(3, int(40 / rec.period)))) if i in errors]
    pre_adj = rec.number("curve_pre_adj", 0.0)
    if before:
        out.append("  lateral on entry: %+.1f mm (> 0: left of the centre)" % mean(before))
    if after:
        lateral = mean(after)
        outside = lateral * direction       # right curve: its outside is the left
        out.append("  lateral on exit: %+.1f mm, %.1f mm %s the curve"
                   % (lateral, abs(outside), "outside" if outside > 0 else "inside"))
        # Out wide: the curve started late (or the robot turned late): start it earlier.
        out.append("  -> TUNE CURVE_PRE %.1f (now %.1f)" % (pre_adj - outside, pre_adj))
    else:
        out.append("  (no side walls on exit: repeat it with walls on both sides of the last cell)")

    # The front wall at the end: where the IR put the stop against the plan.
    planned = CELL_MM + sum(rec.number(k, 0.0) for k in ("curve_pre", "curve_len", "curve_post"))
    stop = ref_f[-1]                                # the reference stays at the end once there
    still = list(range(end, rec.n)) or [end - 1]    # recorded at rest after the stop
    fl = [rec.ir_mm("fl", rec.data["raw_fl"][i]) for i in still]
    fr = [rec.ir_mm("fr", rec.data["raw_fr"][i]) for i in still]
    fl = mean(fl) if None not in fl else None
    fr = mean(fr) if None not in fr else None
    walled = fl is not None and fr is not None and fl < rec.number("wall_detect_mm", 140) \
        and fr < rec.number("wall_detect_mm", 140)
    if walled and "curve_len" in rec.meta:
        post_adj = rec.number("curve_post_adj", 0.0)
        out.append("  wall at the end: stopped %+.1f mm from the plan (%.1f mm)" % (stop - planned, planned))
        out.append("  -> TUNE CURVE_POST %.1f (now %.1f)" % (post_adj + stop - planned, post_adj))
        skew = fl - fr - rec.number("front_square_offset_mm", 0.0)
        yaw_right = -skew / SQUARE_MM_PER_DEG   # FL closer: turned right of square
        out.append("  real heading at the end (FL-FR): %.1f deg to the %s" % (abs(yaw_right),
                                                                         "right" if yaw_right > 0 else "left"))
        front = 0.5 * (fl + fr) - rec.number("front_ref_mm", 94)
        if abs(front) > 6:
            out.append("  (FL-FR %+.0f mm from the reference distance: the heading is not reliable)" % front)
        elif abs(rot[end - 1]) > 45:
            # Real rotation over the encoders', assuming the robot started square.
            k = (direction * 90.0 + yaw_right) / rot[end - 1]
            out.append("  real turn / encoders: %.3f, assuming it came out straight" % k)
            out.append("  -> TUNE CURVE_ANGLE %.2f (now %.2f; the exit heading and FL-FR are noisy:"
                       " average several)" % (90.0 / k, angle))
    else:
        out.append("  (no wall ahead at the end: with one, CURVE_POST and CURVE_ANGLE get calibrated)")


def analyze_straight(rec, out, measured_pairs):
    cells = rec.args[0] if rec.args else 1
    pwm = rec.args[1] if len(rec.args) > 1 else rec.number("spd")
    tpm = rec.number("ticks_per_mm", 9)
    extra = rec.number("move_extra_ticks", 140)
    if extra >= 2 ** 31:    # dumps before Sep 24 printed it unsigned
        extra -= 2 ** 32
    target = cells * rec.number("cell_ticks", 1620) + extra
    ticks = average_ticks(rec)
    stop = stop_index(rec)
    at_stop, final = ticks[max(0, stop - 1)], ticks[-1]
    coast = final - at_stop
    out.append("Straight of %d cell(s) at PWM %s (%s)" % (cells, int(pwm) if pwm is not None else "?", rec.meta.get("result", "?")))
    out.append("  target %d ticks | on braking %d | final %d -> %.1f mm (%d ticks coasting = %.1f mm)"
               % (target, at_stop, final, final / tpm, coast, coast / tpm))
    diff = rec.data["enc_l"][-1] - rec.data["enc_r"][-1]
    out.append("  final left-right difference: %d ticks (%.1f mm)" % (diff, diff / tpm))
    v = speed_mm_s(rec, ticks)
    moving = [v[i] for i in range(stop) if motor_on(rec, i)]
    if moving:
        cruise = sorted(moving)[len(moving) * 3 // 4:]
        out.append("  top speed %.0f mm/s, cruise ~%.0f mm/s" % (max(moving), mean(cruise)))
    steer = [(l - r) / 2 for i, (l, r) in enumerate(zip(rec.data["pwm_l"], rec.data["pwm_r"])) if motor_on(rec, i)]
    if steer:
        out.append("  steering correction (PWM): mean %+.1f, deviation %.1f" % (mean(steer), stdev(steer)))
        if abs(mean(steer)) > 15:
            out.append("  ! it always corrects to the same side: the motors differ (compensate it)")
    lane = rec.number("lane_mm", 168)
    errors = []
    for raw in rec.data["raw_sr"][:stop]:
        mm = rec.ir_mm("sr", raw)
        if mm is not None and mm < rec.number("side_track_mm", 130):
            errors.append(mm - lane / 2)
    if len(errors) > 5:
        crossings = sum(1 for a, b in zip(errors, errors[1:]) if (a < 0) != (b < 0))
        seconds = len(errors) * rec.period / 1000.0
        out.append("  centring on the right wall: error %+.1f +- %.1f mm, %.1f crossings/s"
                   % (mean(errors), stdev(errors), crossings / seconds if seconds else 0))
        if crossings / max(seconds, 1e-9) > 4:
            out.append("  ! it oscillates: lower KP or raise KD")
    measured = rec.note_value("measured")
    if measured:
        per_mm = final / measured
        needed = at_stop + (cells * CELL_MM - measured) * per_mm
        out.append("  measured %.0f mm -> %.2f real ticks/mm (config %.0f)" % (measured, per_mm, tpm))
        out.append("  for %d mm it would have to brake at %.0f ticks (now %d): %+.0f ticks"
                   % (cells * CELL_MM, needed, target, needed - target))
        measured_pairs.append((cells, needed))
    else:
        out.append("  (note the real distance with: /note measured <mm> mm)")


def analyze_turn(rec, out):
    quarters = rec.args[0] if rec.args else 4
    direction = 1 if quarters > 0 else -1
    tpt = rec.number("ticks_per_turn", 422)
    rot = [direction * (l - r) / 2 for l, r in zip(rec.data["enc_l"], rec.data["enc_r"])]
    segments, start = [], None
    for i in range(rec.n):
        on = motor_on(rec, i)
        if on and start is None:
            start = i
        elif not on and start is not None:
            segments.append((start, i))
            start = None
    out.append("Turn of %d quarters (%s), %d stretches found" % (abs(quarters), rec.meta.get("result", "?"), len(segments)))
    overshoots = []
    for k, (a, b) in enumerate(segments):
        # From rest before the motors start to rest again before the next turn (or the end).
        rest = rot[a - 1] if a > 0 else rot[a]
        end = segments[k + 1][0] - 1 if k + 1 < len(segments) else rec.n - 1
        turned_at_stop = rot[b - 1] - rest
        turned = rot[end] - rest
        overshoots.append(turned - turned_at_stop)
        out.append("  turn %d: on braking %.0f ticks, settled %.0f (overshoot %.0f)"
                   % (k + 1, turned_at_stop, turned, turned - turned_at_stop))
    if overshoots:
        out.append("  mean overshoot %.1f +- %.1f ticks (current threshold %d)" % (mean(overshoots), stdev(overshoots), tpt))
    angle = rec.note_value("angle")
    if angle and segments:
        first = segments[0][0]
        total = rot[-1] - (rot[first - 1] if first > 0 else rot[first])
        per_degree = total / angle
        suggested = 90 * per_degree - mean(overshoots)
        out.append("  measured %.0f deg -> %.2f ticks/deg; TICKS_PER_TURN suggested %.0f (now %d):"
                   " try it with TURNTICKS %.0f" % (angle, per_degree, suggested, tpt, suggested))
    else:
        out.append("  (note the real angle turned with: /note angle <deg>)")


def analyze_step(rec, out, motor_points=None):
    pwm = rec.args[0] if rec.args else rec.number("spd")
    tpm = rec.number("ticks_per_mm", 9)
    ticks = average_ticks(rec)
    v = speed_mm_s(rec, ticks, smooth=1)
    on = [i for i in range(rec.n) if motor_on(rec, i)]
    out.append("Open-loop PWM step %s (%s)" % (int(pwm) if pwm is not None else "?", rec.meta.get("result", "?")))
    if not on:
        out.append("  no samples with the motors on")
        return
    t0, off = on[0], on[-1] + 1
    last = off - t0
    steady = v[t0 + int(last * 0.7):off] or v[t0:off]
    v_ss = mean(steady)
    out.append("  steady speed %.0f mm/s -> gain %.2f mm/s per PWM unit" % (v_ss, v_ss / pwm if pwm else 0))
    wheel = {}
    for side in ("l", "r"):
        vw = speed_mm_s(rec, rec.data["enc_" + side], smooth=1)
        wheel[side] = mean(vw[t0 + int(last * 0.7):off] or vw[t0:off])
    out.append("  per wheel: left %.0f mm/s, right %.0f mm/s" % (wheel["l"], wheel["r"]))

    def crossing(fraction):     # ms after the step when the speed crosses fraction * v_ss
        level = fraction * v_ss
        for i in range(t0 + 1, off):
            if v[i] >= level > v[i - 1]:
                return (i - 1 - t0 + (level - v[i - 1]) / (v[i] - v[i - 1])) * rec.period
        return None

    # First order plus dead time, Smith's two-point method.
    t28, t63 = crossing(0.283), crossing(0.632)
    if t28 is not None and t63 is not None and v_ss > 0:
        tau = 1.5 * (t63 - t28)
        out.append("  dead time %.0f ms" % max(0.0, t63 - tau))
        out.append("  time constant ~%.0f ms (first-order model)" % tau)
        if motor_points is not None and pwm:
            motor_points.append((pwm, wheel["l"], wheel["r"], tau, max(0.0, t63 - tau)))
    rest = next((i for i in range(off, rec.n - 1) if all(ticks[j] == ticks[i] for j in range(i, min(rec.n, i + 5)))), rec.n - 1)
    out.append("  braking: %.1f mm in %.0f ms from the cut" % ((ticks[rest] - ticks[off]) / tpm, (rest - off) * rec.period))
    drift = rec.data["enc_l"][off] - rec.data["enc_r"][off]
    run = max(1.0, ticks[off] - ticks[t0])
    out.append("  left-right imbalance: %+d ticks in %.0f mm (%+.1f%%)" % (drift, run / tpm, 100.0 * drift / run))


def analyze_ir(rec, out):
    tpm = rec.number("ticks_per_mm", 9)
    ticks = average_ticks(rec)
    back = [abs(t) / tpm for t in ticks]
    start = rec.note_value("start")
    out.append("Front IR sweep: %.0f mm backwards (%s)" % (max(back), rec.meta.get("result", "?")))
    if start is None:
        out.append("  (note the starting sensors-to-wall distance with: /note start <mm> mm)")
    table = []
    step = 10.0
    next_at = 0.0
    for i, d in enumerate(back):
        if d >= next_at:
            table.append((d, rec.data["raw_fl"][i], rec.data["raw_fr"][i]))
            next_at += step
    out.append("  %9s  %7s  %7s" % ("distancia", "raw FL", "raw FR"))
    for d, fl, fr in table:
        out.append("  %7.0f mm  %7d  %7d" % (d + (start or 0), fl, fr))
    if start is None or len(back) < 8:
        return
    for s in ("fl", "fr"):
        xs = rec.data["raw_" + s]
        ys = [start + d for d in back]
        try:
            a, b, c, d = fit_cubic(xs, ys)
        except ValueError as exc:
            out.append("  %s: %s" % (s.upper(), exc))
            continue
        new_err = [abs(((a * x + b) * x + c) * x + d - y) for x, y in zip(xs, ys)]
        old_err = [abs((rec.ir_mm(s, x) or 0) - y) for x, y in zip(xs, ys)]
        out.append("  %s new fit: mean error %.1f mm (current calibration %.1f mm)"
                   % (s.upper(), mean(new_err), mean(old_err)))
        out.append("     #define CAL_%s  %.5gf, %.5gf, %.5gf, %.5gf" % (s.upper(), a, b, c, d))


def wall_parallel(seen, fwd, rot, base, min_span=100):
    """Encoder heading (relative to `base`, degrees, > 0 right) along which a
    straight ran parallel to its walls: the side readings' drift beyond what
    the encoder heading explains, fitted as a constant (a yawed start, a turn
    or a curve that turned more or less than the encoders say). `seen`: (sample
    where the robot was, lateral mm). None if the readings span too little."""
    if len(seen) < 8 or fwd[seen[-1][0]] - fwd[seen[0][0]] < min_span:
        return None
    k0 = seen[0][0]
    integral, acc = {k0: 0.0}, 0.0
    for k in range(k0 + 1, seen[-1][0] + 1):
        acc += (fwd[k] - fwd[k - 1]) * math.radians(0.5 * (rot[k] + rot[k - 1]) - base)
        integral[k] = acc
    # lateral = L0 - integral - rad(yaw) * s - lever * rad(heading), and parallel = -yaw.
    slope, _ = fit_line([fwd[k] - fwd[k0] for k, _ in seen],
                        [lat + integral[k] + SIDE_LEVER_MM * math.radians(rot[k] - base) for k, lat in seen])
    return math.degrees(slope)


def analyze_run(rec, out):
    """CAL RUN: a whole continuous move of a run (a speed run to the goal, a
    search leg). Per straight between curves: how far off the centre line
    the side walls put the robot once they read that straight, at its end
    and at worst, and the heading offset the centring asked for."""
    tpm = rec.number("ticks_per_mm", 9.05)
    mpd = rec.number("turn_ticks", 400) / 90 / tpm
    angle = curve_encoder_angle(rec, min(rec.number("curve", 480.0), rec.number("fast", 900.0)))
    fwd = [t / tpm for t in average_ticks(rec)]
    rot = [(l - r) / 2 / tpm / mpd for l, r in zip(rec.data["enc_l"], rec.data["enc_r"])]
    ref_f = [v / 10.0 for v in rec.data["ref_fwd"]]
    ref_r = [v / 100.0 for v in rec.data["ref_rot"]]
    on = [i for i in range(rec.n) if motor_on(rec, i)]
    if not on:
        out.append("  the robot did not move")
        return
    first, end = on[0], on[-1] + 1
    # The recording goes on after the stop: what follows is another move.
    top = max(ref_f)
    end = min(end, next(i for i in range(rec.n) if ref_f[i] >= top - 0.05) + 1)
    v = speed_mm_s(rec, average_ticks(rec), smooth=2)
    out.append("Continuous move (%s): %.0f mm in %.0f ms, up to %.0f mm/s"
               % (rec.meta.get("result", "?"), fwd[end - 1] - fwd[first], (end - first) * rec.period,
                  max(v[first:end])))
    out.append("  tracking: forward error max %.2f mm, heading max %.2f deg"
               % (max(abs(ref_f[i] - fwd[i]) for i in on), max(abs(ref_r[i] - rot[i]) for i in on)))
    saturated = sum(1 for i in on if max(abs(rec.data["pwm_l"][i]), abs(rec.data["pwm_r"][i])) >= 1000)
    if saturated:
        out.append("  ! PWM at the limit in %d samples of %d" % (saturated, len(on)))
    # Straights: where the reference turns under 0.4 deg/mm (curves ~0.65; their ramps widen each by 15 mm).
    fast = []
    for i in range(rec.n):
        a, b = max(0, i - 2), min(rec.n - 1, i + 2)
        ds = ref_f[b] - ref_f[a]
        fast.append(ds > 0.5 and abs(ref_r[b] - ref_r[a]) / ds > 0.4)
    marks = [ref_f[i] for i in range(rec.n) if fast[i]]
    curving = [any(abs(ref_f[i] - m) <= 15.0 for m in marks) for i in range(rec.n)]
    delay = int(round(IR_DELAY_MS / rec.period))
    lateral = dict(side_errors(rec, rec.n))
    straights, i = [], first
    while i < end:
        if curving[i]:
            i += 1
            continue
        j = i
        while j < end and not curving[j]:
            j += 1
        if fwd[j - 1] - fwd[i] > 20:
            straights.append((i, j))
        i = j
    for n, (a, b) in enumerate(straights, 1):
        base = angle * round(ref_r[a] / angle)
        # A reading at sample k is where the robot was IR_DELAY_MS before.
        seen = [(k - delay, lateral[k]) for k in range(a + delay, min(b + delay, rec.n)) if k in lateral]
        head = "  straight %d: %.0f-%.0f mm, heading %+.0f" % (n, fwd[a] - fwd[first], fwd[b - 1] - fwd[first], base)
        offsets = [ref_r[k] - base for k in range(a, b)]
        if not seen:
            out.append(head + ": no side walls")
            continue
        worst = max(seen, key=lambda e: abs(e[1]))
        out.append(head + ": lateral %+.1f mm reading the walls (at %.0f mm), %+.1f at the end, worst %+.1f (at %.0f mm);"
                   " centring asked %+.1f..%+.1f deg, %+.1f at the end"
                   % (seen[0][1], fwd[seen[0][0]] - fwd[a], seen[-1][1], worst[1], fwd[worst[0]] - fwd[a],
                      min(offsets), max(offsets), offsets[-1]))
        parallel = wall_parallel(seen, fwd, rot, base)
        if parallel is not None:
            out.append("    parallel to the walls at encoder heading %+.1f deg (%d readings over %.0f mm)"
                       % (parallel, len(seen), fwd[seen[-1][0]] - fwd[seen[0][0]]))
    out.append("  (lateral > 0: left of the centre; deg > 0: to the right)")
    out.append("       s mm   v mm/s  lateral        asked       encoders")
    last = None
    for k in range(first, end):
        if last is not None and fwd[k] - fwd[last] < 30:
            continue
        last = k
        base = angle * round(ref_r[k] / angle)
        lat = lateral.get(k + delay)
        if curving[k]:
            out.append("    %6.0f  %6.0f  %7s  %11s  %14s  curve" % (fwd[k] - fwd[first], v[k], "-", "-", "-"))
            continue
        out.append("    %6.0f  %6.0f  %7s  %+11.1f  %+14.1f"
                   % (fwd[k] - fwd[first], v[k], "%+.1f" % lat if lat is not None else "-", ref_r[k] - base,
                      rot[k] - base))


def chain(paths, wheel_diff=None):
    """--chain: the recordings of one session in order, every move recorded
    and the robot not moved by hand in between. The encoder heading summed
    over them against the robot's real one: per wall stretch (a straight, the
    cells before and after a CAL CURVE), how far the encoders are ahead of
    the robot, from the side walls; and from FL - FR where a move stopped
    square in front of a wall. The ticks are weighted by WHEEL_DIFF as the
    firmware weights them: the dump's (wheel_diff_ppm), or `wheel_diff` for
    the dumps without it (0 then: the firmware's default until 2026-09-26)."""
    out = ["Encoders minus the real heading, deg (> 0: the encoders run right of the robot; modulo 90)"]
    heading = travelled = 0.0
    for path in paths:
        rec = load(path)
        if rec.n == 0 or "ref_rot" not in rec.data:
            continue
        tpm = rec.number("ticks_per_mm", 9.05)
        mpd = rec.number("turn_ticks", 400) / 90 / tpm
        fwd = [t / tpm for t in average_ticks(rec)]
        wd = wheel_diff if wheel_diff is not None else rec.number("wheel_diff_ppm", 0.0) / 1e6
        rot = [(l * (1 + 0.5 * wd) - r * (1 - 0.5 * wd)) / 2 / tpm / mpd
               for l, r in zip(rec.data["enc_l"], rec.data["enc_r"])]
        ref_f = [v / 10.0 for v in rec.data["ref_fwd"]]
        on = [i for i in range(rec.n) if motor_on(rec, i)]
        end = on[-1] + 1 if on else rec.n
        stretches = []
        if rec.kind == "straight":
            stretches.append(("walls", 0, end, 0.0))
        elif rec.kind == "curve" and "curve_len" in rec.meta:
            s0 = CELL_MM / 2 + rec.number("curve_pre", 0.0)
            s1 = s0 + rec.number("curve_len", 0.0)
            a = next((i for i in range(end) if ref_f[i] > s0), end)
            b = next((i for i in range(end) if ref_f[i] >= s1), end)
            turn = 90.0 if not rec.args or rec.args[0] >= 0 else -90.0
            stretches += [("before", 0, a, 0.0), ("after", b, end, turn)]
        delay = int(round(IR_DELAY_MS / rec.period))
        lateral = dict(side_errors(rec, rec.n))
        found = []
        for name, a, b, base in stretches:
            seen = [(k - delay, lateral[k]) for k in range(a + delay, min(b + delay, rec.n)) if k in lateral]
            parallel = wall_parallel(seen, fwd, rot, base, min_span=60)
            if parallel is not None:
                found.append("%s %+5.1f" % (name, wrap90(heading + base + parallel)))
        heading += rot[-1]
        still = list(range(end, rec.n))
        front = [(rec.ir_mm("fl", rec.data["raw_fl"][i]), rec.ir_mm("fr", rec.data["raw_fr"][i])) for i in still]
        if stretches and front and None not in front[0]:
            fl, fr = mean([f for f, _ in front]), mean([r for _, r in front])
            if max(fl, fr) < rec.number("wall_detect_mm", 140) \
                    and abs(0.5 * (fl + fr) - rec.number("front_ref_mm", 94)) <= 6:
                yaw_right = -(fl - fr - rec.number("front_square_offset_mm", 0.0)) / SQUARE_MM_PER_DEG
                found.append("front %+5.1f" % wrap90(heading - yaw_right))
        travelled += fwd[end - 1] - fwd[0]
        name = os.path.basename(path)
        stamp = re.search(r"_(\d\d-\d\d-\d\d)_", name)
        out.append("  %s %-15s %5.2f m  encoders %+8.2f  %s"
                   % (stamp.group(1) if stamp else name, rec.test, travelled / 1000.0, heading, "  ".join(found)))
    return "\n".join(out)


def wrap90(deg):
    return (deg + 45.0) % 90.0 - 45.0


ANALYSES = {"noise": analyze_noise, "turn": analyze_turn, "ir": analyze_ir, "run": analyze_run}


def report(paths):
    out, measured_pairs, motor_points = [], [], []
    mirror = None   # readings of the last CAL NOISE, quarters turned since
    stations, turned = None, 0          # a round's readings by heading (quarters)
    side_points, side_rec = [], None    # (offset, SL, SR) from the rounds
    for path in paths:
        rec = load(path)
        out.append("=" * 72)
        out.append("%s  [%s, %d samples every %.0f ms, firmware %s]"
                   % (path, rec.test, rec.n, rec.period, rec.meta.get("build", "?")))
        if "accel" in rec.meta:
            gain2 = "kd" if "kd" in rec.meta else "ki"     # the centring's second gain (KI before 10-05)
            out.append("  SPD %s FAST %s CURVE %s ACCEL %s TURN %s TACCEL %s KP %s %s %s" % tuple(
                [rec.meta.get(k, "?") for k in ("spd", "fast", "curve", "accel", "turn", "turn_accel", "kp")]
                + [gain2.upper(), rec.meta.get(gain2, "?")]))
        else:
            out.append("  SPD %s FAST %s TURN %s KP %s KI %s KD %s KE %s" % tuple(
                rec.meta.get(k, "?") for k in ("spd", "fast", "turn", "kp", "ki", "kd", "ke")))
        for note in rec.notes:
            out.append("  note: " + note)
        if rec.n == 0:
            out.append("  no samples")
        elif rec.kind == "straight" and controlled(rec):
            analyze_straight_controlled(rec, out)
        elif rec.kind == "straight":
            analyze_straight(rec, out, measured_pairs)
        elif rec.kind == "turn" and controlled(rec):
            analyze_turn_controlled(rec, out)
        elif rec.kind == "curve" and controlled(rec):
            analyze_curve_controlled(rec, out)
        elif rec.kind == "step":
            analyze_step(rec, out, motor_points)
        elif rec.kind in ANALYSES:
            ANALYSES[rec.kind](rec, out)
        else:
            out.append("  prueba desconocida: %s" % rec.kind)
        # The side sensors: CAL NOISE, CAL TURN 2, CAL NOISE in a corridor, or rounds (side_round).
        if rec.kind == "noise" and rec.n:
            means = rest_means(rec)
            if mirror and means and mirror[1] % 4 == 2:
                mirror_centres(rec, mirror[0], means, out)
            mirror = (means, 0) if means else None
            if means is None:
                stations = None
            else:
                if stations is None:
                    stations, turned = {}, 0
                stations[turned % 4] = means
                if len(stations) == 4:
                    side_round(rec, stations, out, side_points)
                    stations, side_rec = None, rec
        elif rec.kind == "turn" and mirror:
            quarters = rec.args[0] if rec.args else 4
            mirror = (mirror[0], mirror[1] + quarters)
            turned += quarters
        else:
            mirror = stations = None
    if side_rec:
        out.append("=" * 72)
        side_fit(side_rec, side_points, out)
    lengths = {cells for cells, _ in measured_pairs}
    if len(lengths) >= 2:
        # needed(N) = N * CELL_TICKS + MOVE_EXTRA_TICKS, least squares over all measured straights.
        n = len(measured_pairs)
        sx = sum(c for c, _ in measured_pairs)
        sy = sum(t for _, t in measured_pairs)
        sxx = sum(c * c for c, _ in measured_pairs)
        sxy = sum(c * t for c, t in measured_pairs)
        cell = (n * sxy - sx * sy) / (n * sxx - sx * sx)
        extra = (sy - cell * sx) / n
        out.append("=" * 72)
        out.append("From %d measured straights: CELL_TICKS ~ %.0f, MOVE_EXTRA_TICKS ~ %.0f" % (n, cell, extra))
    if len({p for p, _, _, _, _ in motor_points}) >= 2:
        out.append("=" * 72)
        out.extend(motor_model(motor_points))
    return "\n".join(out)


def fit_line(xs, ys):
    """Least squares y = a x + b."""
    n = len(xs)
    sx, sy = sum(xs), sum(ys)
    sxx, sxy = sum(x * x for x in xs), sum(x * y for x, y in zip(xs, ys))
    a = (n * sxy - sx * sy) / (n * sxx - sx * sx)
    return a, (sy - a * sx) / n


def motor_model(points):
    """Feedforward constants from open-loop steps: PWM = KV * v + KS per wheel."""
    pwm = [p for p, _, _, _, _ in points]
    kv_l, ks_l = fit_line([l for _, l, _, _, _ in points], pwm)
    kv_r, ks_r = fit_line([r for _, _, r, _, _ in points], pwm)
    tau = mean([t for _, _, _, t, _ in points])
    dead = mean([d for _, _, _, _, d in points])
    return ["Motor model from %d steps (PWM = KV * speed + KS):" % len(points),
            "  left:  KV %.3f PWM per mm/s, KS %.0f PWM" % (kv_l, ks_l),
            "  right: KV %.3f PWM per mm/s, KS %.0f PWM" % (kv_r, ks_r),
            "  mean time constant %.0f ms, dead time %.0f ms" % (tau, dead),
            "     #define MOTOR_KV_L              %.3ff" % kv_l,
            "     #define MOTOR_KV_R              %.3ff" % kv_r,
            "     #define MOTOR_TAU_S             %.3ff" % (tau / 1000.0),
            "     #define MOTOR_KS_PWM            %.1ff" % max(0.0, (ks_l + ks_r) / 2)]


# ---- IR delay (--delay) ----------------------------------------------------------------

def best_delay(xs, ys, max_d):
    """The d (samples) that best fits ys[i + d] ~ a + b xs[i] (None: no reading): (d, b, rms), or None."""
    best = None
    for d in range(max_d + 1):
        y = ys[d:d + len(xs)]
        if len(y) < len(xs) or None in y:
            break
        mx, my = mean(xs), mean(y)
        sxx = sum((x - mx) ** 2 for x in xs)
        if sxx <= 0:
            return None
        b = sum((x - mx) * (v - my) for x, v in zip(xs, y)) / sxx
        rms = (sum((v - my - b * (x - mx)) ** 2 for x, v in zip(xs, y)) / len(xs)) ** 0.5
        if best is None or rms < best[2]:
            best = (d, b, rms)
    return best


def delay_turn(rec):
    """Each IR's delay on an in-place turn: its reading against the encoder angle over the first 25 deg,
    where it moves only with the angle. {sensor: (ms, mm per deg, rms)}, informative fits only."""
    tpm = rec.number("ticks_per_mm", 9.05)
    mpd = rec.number("turn_ticks", 400) / 90 / tpm
    ang = [(l - r) / 2 / tpm / mpd for l, r in zip(rec.data["enc_l"], rec.data["enc_r"])]
    start = next((i for i, a in enumerate(ang) if abs(a) > 0.5), None)
    end = next((i for i, a in enumerate(ang) if abs(a) > 25), None)
    if start is None or end is None:
        return {}
    first, max_d, out = max(0, start - 6), int(100 / rec.period), {}
    for s in SENSORS:
        limit = 130 if s in ("sl", "sr") else 220      # beyond: no wall to read
        r = [rec.ir_mm(s, v) for v in rec.data["raw_" + s]]
        r = [v if v is not None and v <= limit else None for v in r]
        fit = best_delay(ang[first:end + 1], r[first:], max_d)
        if fit and abs(fit[1]) >= 0.5 and fit[2] < 3.0:
            out[s] = (fit[0] * rec.period, fit[1], fit[2])
    return out


def delay_approach(rec):
    """The front IR's delay approaching the wall at the end of a straight: the reading against the true
    distance (the final reading plus the encoder distance still to go). {sensor: (ms, bias mm, rms)}."""
    if rec.meta.get("result") != "OK":
        return {}
    tpm = rec.number("ticks_per_mm", 9.05)
    pos = [t / tpm for t in average_ticks(rec)]
    out, last = {}, rec.n - 1
    for s in ("fl", "fr"):
        r = [rec.ir_mm(s, v) for v in rec.data["raw_" + s]]
        if r[last] is None:
            continue
        dist = [r[last] + pos[last] - p for p in pos]
        idx = [i for i in range(rec.n) if r[i] is not None and 60 < r[i] < 160 and dist[i] < 170]
        best = None
        for ms in range(0, 101, 2):
            k = ms / rec.period
            errs = []
            for i in idx:
                j = int(i - k)
                if 0 <= j < last:
                    errs.append(r[i] - (dist[j] + (dist[j + 1] - dist[j]) * (i - k - j)))
            if len(errs) < 5:
                continue
            m = mean(errs)
            rms = (sum((e - m) ** 2 for e in errs) / len(errs)) ** 0.5
            if best is None or rms < best[2]:
                best = (ms, m, rms)
        if best:
            out[s] = best
    return out


def delays(paths):
    """--delay: every IR's delay from in-place turns, the front ones' also from wall approaches."""
    out, turns, approaches = [], {s: [] for s in SENSORS}, {"fl": [], "fr": []}
    for path in paths:
        rec = load(path)
        if rec.kind == "turn" and rec.period <= 4:
            found = delay_turn(rec)
            for s, v in found.items():
                turns[s].append(v[0])
            text = "  ".join("%s %d ms (%+.2f mm/deg)" % (s.upper(), v[0], v[1]) for s, v in found.items())
        elif rec.kind == "straight" and "ref_fwd" in rec.data:
            found = delay_approach(rec)
            for s, v in found.items():
                approaches[s].append(v[0])
            text = "  ".join("%s %d ms (rms %.1f)" % (s.upper(), v[0], v[2]) for s, v in found.items())
        else:
            continue
        out.append("%s  %s" % (os.path.basename(path), text or "-"))
    def summary(name, table):
        for s, v in table.items():
            if v:
                v = sorted(v)
                out.append("%s %s: %d, median %d ms (quartiles %d-%d)"
                           % (name, s.upper(), len(v), v[len(v) // 2], v[len(v) // 4], v[3 * len(v) // 4]))
    summary("in-place turns", turns)
    summary("wall approaches", approaches)
    return "\n".join(out)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("files", nargs="+", help="CSVs from tools/calib_data/")
    parser.add_argument("--chain", action="store_true",
                        help="a session in order: encoder heading against the real one, move by move")
    parser.add_argument("--wheel-diff", type=float,
                        help="with --chain: the session's WHEEL_DIFF, if the dumps lack it (0)")
    parser.add_argument("--delay", action="store_true",
                        help="every IR's delay: in-place turns (CAL TURN), wall approaches (CAL STRAIGHT)")
    args = parser.parse_args(argv)
    if args.delay:
        print(delays(sorted(args.files)))
    else:
        print(chain(sorted(args.files), args.wheel_diff) if args.chain else report(args.files))
    return 0


if __name__ == "__main__":
    sys.exit(main())
