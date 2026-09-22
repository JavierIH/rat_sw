#!/usr/bin/env python3
"""Summarise calibration recordings from the robot (CAL commands).

    python3 tools/calib_analyze.py tools/calib_data/*.csv

Each CSV (saved by robot_monitor.py) carries the test, every relevant
firmware constant and the samples. Physical measurements added with
/nota in the monitor turn the report into concrete suggestions:

    straight:  /nota medido 176 mm        distance actually travelled
    turn:      /nota angulo 352           total angle actually turned (degrees)
    ir:        /nota inicio 40 mm         front sensors to wall when the sweep starts

Straight recordings of different lengths, all measured, are combined to
split the distance calibration into CELL_TICKS and MOVE_EXTRA_TICKS.
"""
import argparse
import math
import re
import sys

CELL_MM = 180
SENSORS = ("fl", "fr", "sl", "sr")


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
        """First number after any of `words` in the notes (e.g. 'medido 176 mm')."""
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
                if body.startswith("nota:"):
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
            raise ValueError("datos insuficientes para ajustar")
        for r in range(4):
            if r != col:
                f = m[r][col] / m[col][col]
                m[r] = [a - f * b for a, b in zip(m[r], m[col])]
    return [m[i][4] / m[i][i] for i in range(4)]


# ---- Analyses -----------------------------------------------------------------------------

def analyze_noise(rec, out):
    out.append("Ruido de sensores con el robot quieto (%d muestras)" % rec.n)
    for s in SENSORS:
        raw = rec.data["raw_" + s]
        mm = [rec.ir_mm(s, r) for r in raw]
        line = "  %s raw %7.1f +- %5.1f (min %4d max %4d)" % (s.upper(), mean(raw), stdev(raw), min(raw), max(raw))
        if mm[0] is not None:
            line += "  ->  %6.1f +- %4.2f mm" % (mean(mm), stdev(mm))
        out.append(line)
    moved = max(max(abs(v) for v in rec.data["enc_l"]), max(abs(v) for v in rec.data["enc_r"]))
    out.append("  encoders: %s" % ("quietos" if moved == 0 else "se movieron %d ticks (vibracion?)" % moved))
    worst = max(stdev([rec.ir_mm(s, r) or 0 for r in rec.data["raw_" + s]]) for s in SENSORS)
    if worst > 3:
        out.append("  ! un sensor oscila mas de 3 mm: revisa conexiones o sube el promediado (IR_OVERSAMPLE)")


def analyze_straight(rec, out, measured_pairs):
    cells = rec.args[0] if rec.args else 1
    pwm = rec.args[1] if len(rec.args) > 1 else rec.number("spd")
    tpm = rec.number("ticks_per_mm", 9)
    target = cells * rec.number("cell_ticks", 1620) + rec.number("move_extra_ticks", 140)
    ticks = average_ticks(rec)
    stop = stop_index(rec)
    at_stop, final = ticks[max(0, stop - 1)], ticks[-1]
    coast = final - at_stop
    out.append("Recta de %d celda(s) a PWM %s (%s)" % (cells, int(pwm) if pwm is not None else "?", rec.meta.get("result", "?")))
    out.append("  objetivo %d ticks | al frenar %d | final %d -> %.1f mm (%d ticks de inercia = %.1f mm)"
               % (target, at_stop, final, final / tpm, coast, coast / tpm))
    diff = rec.data["enc_l"][-1] - rec.data["enc_r"][-1]
    out.append("  diferencia final izq-der: %d ticks (%.1f mm)" % (diff, diff / tpm))
    v = speed_mm_s(rec, ticks)
    moving = [v[i] for i in range(stop) if motor_on(rec, i)]
    if moving:
        cruise = sorted(moving)[len(moving) * 3 // 4:]
        out.append("  velocidad maxima %.0f mm/s, crucero ~%.0f mm/s" % (max(moving), mean(cruise)))
    steer = [(l - r) / 2 for i, (l, r) in enumerate(zip(rec.data["pwm_l"], rec.data["pwm_r"])) if motor_on(rec, i)]
    if steer:
        out.append("  correccion de direccion (PWM): media %+.1f, desviacion %.1f" % (mean(steer), stdev(steer)))
        if abs(mean(steer)) > 15:
            out.append("  ! corrige siempre hacia el mismo lado: los motores no son iguales (compensalo)")
    lane = rec.number("lane_mm", 168)
    errors = []
    for raw in rec.data["raw_sr"][:stop]:
        mm = rec.ir_mm("sr", raw)
        if mm is not None and mm < rec.number("side_track_mm", 130):
            errors.append(mm - lane / 2)
    if len(errors) > 5:
        crossings = sum(1 for a, b in zip(errors, errors[1:]) if (a < 0) != (b < 0))
        seconds = len(errors) * rec.period / 1000.0
        out.append("  centrado con la pared derecha: error %+.1f +- %.1f mm, %.1f cruces/s"
                   % (mean(errors), stdev(errors), crossings / seconds if seconds else 0))
        if crossings / max(seconds, 1e-9) > 4:
            out.append("  ! oscila: baja KP o sube KD")
    measured = rec.note_value("medido", "real", "mide")
    if measured:
        per_mm = final / measured
        needed = at_stop + (cells * CELL_MM - measured) * per_mm
        out.append("  medido %.0f mm -> %.2f ticks/mm reales (config %.0f)" % (measured, per_mm, tpm))
        out.append("  para %d mm habria que frenar a %.0f ticks (ahora %d): %+.0f ticks"
                   % (cells * CELL_MM, needed, target, needed - target))
        measured_pairs.append((cells, needed))
    else:
        out.append("  (anota la distancia real con: /nota medido <mm> mm)")


def analyze_turn(rec, out):
    quarters = rec.args[0] if rec.args else 4
    direction = 1 if quarters > 0 else -1
    tpt = rec.number("ticks_per_turn", 430)
    rot = [direction * (l - r) / 2 for l, r in zip(rec.data["enc_l"], rec.data["enc_r"])]
    segments, start = [], None
    for i in range(rec.n):
        on = motor_on(rec, i)
        if on and start is None:
            start = i
        elif not on and start is not None:
            segments.append((start, i))
            start = None
    out.append("Giro de %d cuartos (%s), %d tramos detectados" % (abs(quarters), rec.meta.get("result", "?"), len(segments)))
    overshoots = []
    for k, (a, b) in enumerate(segments):
        # From rest before the motors start, to the last sample driving, to
        # rest again just before the next turn (or the end).
        rest = rot[a - 1] if a > 0 else rot[a]
        end = segments[k + 1][0] - 1 if k + 1 < len(segments) else rec.n - 1
        turned_at_stop = rot[b - 1] - rest
        turned = rot[end] - rest
        overshoots.append(turned - turned_at_stop)
        out.append("  giro %d: al frenar %.0f ticks, tras asentarse %.0f (sobregiro %.0f)"
                   % (k + 1, turned_at_stop, turned, turned - turned_at_stop))
    if overshoots:
        out.append("  sobregiro medio %.1f +- %.1f ticks (umbral actual %d)" % (mean(overshoots), stdev(overshoots), tpt))
    angle = rec.note_value("angulo", "grados")
    if angle and segments:
        first = segments[0][0]
        total = rot[-1] - (rot[first - 1] if first > 0 else rot[first])
        per_degree = total / angle
        suggested = 90 * per_degree - mean(overshoots)
        out.append("  medido %.0f grados -> %.2f ticks/grado; TICKS_PER_TURN sugerido %.0f (ahora %d)"
                   % (angle, per_degree, suggested, tpt))
    else:
        out.append("  (anota el angulo real girado con: /nota angulo <grados>)")


def analyze_step(rec, out):
    pwm = rec.args[0] if rec.args else rec.number("spd")
    tpm = rec.number("ticks_per_mm", 9)
    ticks = average_ticks(rec)
    v = speed_mm_s(rec, ticks, smooth=1)
    on = [i for i in range(rec.n) if motor_on(rec, i)]
    out.append("Escalon de PWM %s en lazo abierto (%s)" % (int(pwm) if pwm is not None else "?", rec.meta.get("result", "?")))
    if not on:
        out.append("  no hay muestras con motores encendidos")
        return
    t0, off = on[0], on[-1] + 1
    last = off - t0
    steady = v[t0 + int(last * 0.7):off] or v[t0:off]
    v_ss = mean(steady)
    out.append("  velocidad estable %.0f mm/s -> ganancia %.2f mm/s por unidad de PWM" % (v_ss, v_ss / pwm if pwm else 0))

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
        out.append("  tiempo muerto %.0f ms" % max(0.0, t63 - tau))
        out.append("  constante de tiempo ~%.0f ms (modelo de primer orden)" % tau)
    rest = next((i for i in range(off, rec.n - 1) if all(ticks[j] == ticks[i] for j in range(i, min(rec.n, i + 5)))), rec.n - 1)
    out.append("  frenada: %.1f mm en %.0f ms desde el corte" % ((ticks[rest] - ticks[off]) / tpm, (rest - off) * rec.period))
    drift = rec.data["enc_l"][off] - rec.data["enc_r"][off]
    run = max(1.0, ticks[off] - ticks[t0])
    out.append("  desequilibrio izq-der: %+d ticks en %.0f mm (%+.1f%%)" % (drift, run / tpm, 100.0 * drift / run))


def analyze_ir(rec, out):
    tpm = rec.number("ticks_per_mm", 9)
    ticks = average_ticks(rec)
    back = [abs(t) / tpm for t in ticks]
    start = rec.note_value("inicio", "empieza", "desde")
    out.append("Barrido IR frontal: %.0f mm hacia atras (%s)" % (max(back), rec.meta.get("result", "?")))
    if start is None:
        out.append("  (anota la distancia inicial sensores-pared con: /nota inicio <mm> mm)")
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
        out.append("  %s ajuste nuevo: error medio %.1f mm (calibracion actual %.1f mm)"
                   % (s.upper(), mean(new_err), mean(old_err)))
        out.append("     #define CAL_%s  %.5gf, %.5gf, %.5gf, %.5gf" % (s.upper(), a, b, c, d))


ANALYSES = {"noise": analyze_noise, "turn": analyze_turn, "step": analyze_step, "ir": analyze_ir}


def report(paths):
    out, measured_pairs = [], []
    for path in paths:
        rec = load(path)
        out.append("=" * 72)
        out.append("%s  [%s, %d muestras cada %.0f ms, firmware %s]"
                   % (path, rec.test, rec.n, rec.period, rec.meta.get("build", "?")))
        out.append("  SPD %s FAST %s TURN %s KP %s KD %s KE %s" % tuple(
            rec.meta.get(k, "?") for k in ("spd", "fast", "turn", "kp", "kd", "ke")))
        for note in rec.notes:
            out.append("  nota: " + note)
        if rec.n == 0:
            out.append("  sin muestras")
        elif rec.kind == "straight":
            analyze_straight(rec, out, measured_pairs)
        elif rec.kind in ANALYSES:
            ANALYSES[rec.kind](rec, out)
        else:
            out.append("  prueba desconocida: %s" % rec.kind)
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
        out.append("Con %d rectas medidas: CELL_TICKS ~ %.0f, MOVE_EXTRA_TICKS ~ %.0f" % (n, cell, extra))
    return "\n".join(out)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("files", nargs="+", help="CSV de tools/calib_data/")
    args = parser.parse_args(argv)
    print(report(args.files))
    return 0


if __name__ == "__main__":
    sys.exit(main())
