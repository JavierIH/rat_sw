#!/usr/bin/env python3
"""Live monitor and console for the Ratatron micromouse.

Draws the maze while the robot explores it (from the robot's '@' telemetry
lines, see src/telemetry.h) next to the run log, and sends typed commands.
The planned route is recomputed here with the robot's own planner and costs
(read from src/robot_config.h), so the drawing shows what the robot will do.

    python3 tools/robot_monitor.py [--port /dev/rfcomm0] [--baud 9600]
    python3 tools/robot_monitor.py --replay tools/logs/<session>.log [--speed 4]

Sessions are recorded to tools/logs/ (--no-record to disable) and can be
replayed. Calibration dumps (CAL commands) are saved as CSV files in
tools/calib_data/; add your own measurements to the last one with /note.
The monitor only talks to the robot when you type a command (plus one SYNC
on connect), so it does not slow the robot down.

Keys: Enter send | Up/Down history | PgUp/PgDn/Home/End log | Tab help |
      Ctrl+X STOP the run | Ctrl+L repaint | Esc quit

Needs a terminal with UTF-8 and 24-bit colour (256 colours work, coarser). The maze
is drawn with sextant characters (Unicode 13): where they show as boxes, --pixels half.
"""
import argparse
import collections
import heapq
import math
import os
import re
import select
import shutil
import sys
import termios
import threading
import time
import tty

try:
    import serial
except ImportError:     # only needed for a live connection, not for --replay or the tests
    serial = None

HERE = os.path.dirname(os.path.abspath(__file__))
CONFIG_H = os.path.join(HERE, "..", "src", "robot_config.h")
LOG_DIR = os.path.join(HERE, "logs")
CALIB_DIR = os.path.join(HERE, "calib_data")

MAZE = 16
STATES = MAZE * MAZE * 4
INF = 1 << 30
DX = (0, 1, 0, -1)
DY = (1, 0, -1, 0)
HEADINGS = "NESW"
HEX = "0123456789ABCDEF"
UNKNOWN, WALL, OPEN_ONCE, OPEN_VERIFIED = range(4)
CELL_CODE = "0123456789ABCDEFGHIJKLMNOPQRSTUV"
WALL_CHAR = {"?": UNKNOWN, "#": WALL, ".": OPEN_ONCE, "o": OPEN_VERIFIED}

MODE_NAME = {1: "SEARCH", 2: "LEFT FOLLOWER", 3: "RIGHT FOLLOWER", 4: "SAFE RACE",
             5: "RACE", 6: "ERASE MAP", 7: "NO CURVES", 8: "MID RACE"}
ACTIVITY_NAME = {"I": "stopped", "C": "countdown", "G": "exploring to the goal",
                 "O": "optimizing the route", "H": "exploring to the start",
                 "F": "speed run", "R": "returning to the start",
                 "W": "following a wall", "E": "erasing the map", "K": "calibrating"}
RUNNING = set("CGOHFRWK")
MIN_ROWS, MIN_COLS = 12, 44
MIN_LOG_COLS = 34


def read_config(path=CONFIG_H):
    """Planner costs and start cell from the firmware's robot_config.h."""
    values = {"SEARCH_COST_CELL": 2, "SEARCH_COST_TURN": 1, "FAST_COST_CELL": 2,
              "FAST_COST_TURN": 1, "START_X": 0, "START_Y": 0}
    try:
        with open(path, encoding="utf-8") as f:
            for line in f:
                m = re.match(r"\s*#define\s+(\w+)\s+(\d+)\b", line)
                if m and m.group(1) in values:
                    values[m.group(1)] = int(m.group(2))
    except OSError:
        pass
    return values


def state(x, y, h):
    return ((y * MAZE + x) << 2) | h


ROUTE_MAX_CELLS = 255   # PATH_MAX_CELLS in src/path.h
ROUTE_TEXT_MAX = 40     # as in src/search.c


# ---- Maze model built from the telemetry ---------------------------------------------

class MazeModel:
    def __init__(self):
        self.goal = (7, 7, 8, 8)
        self.mode = None
        self.activity = None
        self.pose = None            # (x, y, heading)
        self.trail = []             # cells visited during the current run
        self.version = 0            # bumped on every map change
        self.telemetry_lines = 0
        self.bad_lines = 0
        self.last_telemetry = None
        self.clear()

    def clear(self):
        self.north = [[UNKNOWN] * MAZE for _ in range(MAZE)]
        self.east = [[UNKNOWN] * MAZE for _ in range(MAZE)]
        self.visited = [[False] * MAZE for _ in range(MAZE)]
        self.version += 1

    def wall(self, x, y, h):
        if h == 0:
            return WALL if y == MAZE - 1 else self.north[x][y]
        if h == 1:
            return WALL if x == MAZE - 1 else self.east[x][y]
        if h == 2:
            return WALL if y == 0 else self.north[x][y - 1]
        return WALL if x == 0 else self.east[x - 1][y]

    def set_wall(self, x, y, h, value):
        if h == 0 and y < MAZE - 1:
            self.north[x][y] = value
        elif h == 1 and x < MAZE - 1:
            self.east[x][y] = value
        elif h == 2 and y > 0:
            self.north[x][y - 1] = value
        elif h == 3 and x > 0:
            self.east[x - 1][y] = value

    def is_goal(self, x, y):
        x0, y0, x1, y1 = self.goal
        return x0 <= x <= x1 and y0 <= y <= y1

    def goal_cells(self):
        x0, y0, x1, y1 = self.goal
        return {(x, y) for x in range(x0, x1 + 1) for y in range(y0, y1 + 1)}

    def visited_count(self):
        return sum(v for column in self.visited for v in column)

    def apply(self, line, now=None):
        """Consume a telemetry line. False if `line` is not telemetry."""
        if not line.startswith("@"):
            return False
        try:
            self._apply(line[1:2], line[2:])
            self.telemetry_lines += 1
            self.last_telemetry = time.monotonic() if now is None else now
        except (ValueError, IndexError, KeyError):
            self.bad_lines += 1     # corrupted on the radio link: ignore
        return True

    def _apply(self, kind, body):
        if kind == "P":
            _expect(body, 3)
            self._set_pose(_hex(body[0]), _hex(body[1]), _heading(body[2]))
        elif kind == "C":
            _expect(body, 7)
            x, y, h = _hex(body[0]), _hex(body[1]), _heading(body[2])
            walls = [WALL_CHAR[c] for c in body[3:7]]
            for d in range(4):
                self.set_wall(x, y, d, walls[d])
            self.visited[x][y] = True
            self.version += 1
            self._set_pose(x, y, h)
        elif kind == "R":
            _expect(body, 1 + MAZE)
            y = _hex(body[0])
            codes = [CELL_CODE.index(c) for c in body[1:]]
            for x, v in enumerate(codes):
                self.north[x][y] = v & 3
                self.east[x][y] = (v >> 2) & 3
                self.visited[x][y] = bool(v & 16)
            self.version += 1
        elif kind == "G":
            _expect(body, 4)
            x0, y0, x1, y1 = (_hex(c) for c in body)
            if x0 > x1 or y0 > y1:
                raise ValueError(body)
            self.goal = (x0, y0, x1, y1)
            self.version += 1
        elif kind == "M":
            mode = int(body)
            if mode not in MODE_NAME:
                raise ValueError(body)
            self.mode = mode
        elif kind == "A":
            _expect(body, 1)
            if body not in ACTIVITY_NAME:
                raise ValueError(body)
            if body == "C" or (body in RUNNING and self.activity not in RUNNING):
                self.trail = [self.pose[:2]] if self.pose else []
            self.activity = body
        elif kind == "Y":
            _expect(body, 1)
            _hex(body)
            self.clear()
        else:
            raise ValueError(kind)

    def _set_pose(self, x, y, h):
        self.pose = (x, y, h)
        if self.activity in RUNNING and (not self.trail or self.trail[-1] != (x, y)):
            self.trail.append((x, y))


def _expect(body, length):
    if len(body) != length:
        raise ValueError(body)


def _hex(c):
    return HEX.index(c)


def _heading(c):
    return HEADINGS.index(c)


# ---- Planner: a port of maze.c, same costs and tie-breaks -----------------------------

class Overlay:
    def __init__(self, fast_cost=None, fast_path=(), fast_turns=0, route=(), candidates=()):
        self.fast_cost = fast_cost
        self.fast_path = list(fast_path)
        self.fast_turns = fast_turns
        self.route = list(route)
        self.candidates = set(candidates)


class Planner:
    def __init__(self, config):
        self.search = (config["SEARCH_COST_CELL"], config["SEARCH_COST_TURN"])
        self.fast = (config["FAST_COST_CELL"], config["FAST_COST_TURN"])
        self.start = (config["START_X"], config["START_Y"])
        self._cache_key = None
        self._cache = Overlay()

    @staticmethod
    def passable(model, x, y, h, verified):
        w = model.wall(x, y, h)
        if w == WALL:
            return False
        return w == OPEN_VERIFIED if verified else True

    def plan_to(self, model, targets, verified, costs):
        """Cost to reach any of `targets` (cells), for every state."""
        cell_cost, turn_cost = costs
        cost = [INF] * STATES
        heap = []
        for x, y in targets:
            for h in range(4):
                s = state(x, y, h)
                cost[s] = 0
                heap.append((0, s))
        heapq.heapify(heap)
        while heap:
            c, s = heapq.heappop(heap)
            if c > cost[s]:
                continue
            h, x, y = s & 3, (s >> 2) % MAZE, (s >> 2) // MAZE
            steps = [(state(x, y, (h + 1) & 3), turn_cost), (state(x, y, (h + 3) & 3), turn_cost)]
            if self.passable(model, x, y, (h + 2) & 3, verified):
                steps.append((state(x - DX[h], y - DY[h], h), cell_cost))
            for p, w in steps:
                if c + w < cost[p]:
                    cost[p] = c + w
                    heapq.heappush(heap, (c + w, p))
        return cost

    def plan_from(self, model, x0, y0, h0, verified, costs):
        cell_cost, turn_cost = costs
        cost = [INF] * STATES
        cost[state(x0, y0, h0)] = 0
        heap = [(0, state(x0, y0, h0))]
        while heap:
            c, s = heapq.heappop(heap)
            if c > cost[s]:
                continue
            h, x, y = s & 3, (s >> 2) % MAZE, (s >> 2) // MAZE
            steps = [(state(x, y, (h + 1) & 3), turn_cost), (state(x, y, (h + 3) & 3), turn_cost)]
            if self.passable(model, x, y, h, verified):
                steps.append((state(x + DX[h], y + DY[h], h), cell_cost))
            for n, w in steps:
                if c + w < cost[n]:
                    cost[n] = c + w
                    heapq.heappush(heap, (c + w, n))
        return cost

    def best_action(self, model, cost, x, y, h, verified, costs):
        """'F', 'U' (around), 'R', 'L' or None: same order as maze_best_action()."""
        cell_cost, turn_cost = costs
        here = cost[state(x, y, h)]
        if here == 0 or here >= INF:
            return None
        best, action = INF, None
        if self.passable(model, x, y, h, verified):
            c = cell_cost + cost[state(x + DX[h], y + DY[h], h)]
            if c < best:
                best, action = c, "F"
        for act, turns, nh in (("U", 2, (h + 2) & 3), ("R", 1, (h + 1) & 3), ("L", 1, (h + 3) & 3)):
            c = turns * turn_cost + cost[state(x, y, nh)]
            if c < best:
                best, action = c, act
        return action if best < INF else None

    def follow(self, model, cost, x, y, h, verified, costs):
        """Cells along the optimal path from (x, y, h), and how many turns."""
        cells, turns = [], 0
        for _ in range(STATES):
            a = self.best_action(model, cost, x, y, h, verified, costs)
            if a is None:
                break
            if a == "F":
                x, y = x + DX[h], y + DY[h]
                cells.append((x, y))
            else:
                h = (h + {"R": 1, "L": 3, "U": 2}[a]) & 3
                turns += 2 if a == "U" else 1
        return cells, turns

    def route(self, model, cost, x, y, h, verified, costs, limit=ROUTE_MAX_CELLS):
        """(in-place quarter turns, turn inside each cell entered) like
        maze_route(): the speed run's route driven in one go, or None."""
        a = self.best_action(model, cost, x, y, h, verified, costs)
        if a is None:
            return None
        turn = {"F": 0, "L": -1, "R": 1, "U": 2}[a]
        h = (h + turn) & 3
        turns = []
        while len(turns) < limit:
            a = self.best_action(model, cost, x, y, h, verified, costs)
            if a == "F":
                x, y = x + DX[h], y + DY[h]
                turns.append(0)
            elif a in ("L", "R") and turns and not turns[-1]:
                turns[-1] = 1 if a == "R" else -1
                h = (h + turns[-1]) & 3
            else:
                break
        if turns:
            turns[-1] = 0
        return turn, turns


    def candidates(self, model):
        """Unvisited cells on an optimistic optimal speed-run path (search phase OPTIM)."""
        sx, sy = self.start
        fwd = self.plan_from(model, sx, sy, 0, False, self.fast)
        bwd = self.plan_to(model, model.goal_cells(), False, self.fast)
        best = bwd[state(sx, sy, 0)]
        found = set()
        if best >= INF:
            return found
        for y in range(MAZE):
            for x in range(MAZE):
                if model.visited[x][y]:
                    continue
                for h in range(4):
                    s = state(x, y, h)
                    if fwd[s] < INF and bwd[s] < INF and fwd[s] + bwd[s] == best:
                        found.add((x, y))
                        break
        return found

    def overlay(self, model):
        key = (model.version, model.pose, model.activity, model.goal)
        if key == self._cache_key:
            return self._cache
        sx, sy = self.start
        goal = model.goal_cells()
        fast = self.plan_to(model, goal, True, self.fast)
        fast_cost = fast[state(sx, sy, 0)]
        fast_path, fast_turns = [], 0
        if fast_cost < INF:
            fast_path, fast_turns = self.follow(model, fast, sx, sy, 0, True, self.fast)
        route, cands = [], set()
        act, pose = model.activity, model.pose
        if pose and act in ("G", "O", "H"):
            if act == "O":
                cands = self.candidates(model)
            targets = goal if act == "G" else cands if act == "O" else {self.start}
            if targets:
                cost = self.plan_to(model, targets, False, self.search)
                route = self.follow(model, cost, *pose, False, self.search)[0]
        elif pose and act in ("F", "R"):
            targets = goal if act == "F" else {self.start}
            cost = self.plan_to(model, targets, True, self.fast)
            route = self.follow(model, cost, *pose, True, self.fast)[0]
        self._cache_key = key
        self._cache = Overlay(fast_cost if fast_cost < INF else None, fast_path, fast_turns, route, cands)
        return self._cache


def route_text(turns, limit=ROUTE_TEXT_MAX):
    """The route as the firmware logs it: cells straight ahead, then R/L for a
    curve right/left in the last of them ("2R1L3"), cut with '+'."""
    text, run = "", 0
    for i, t in enumerate(turns):
        run += 1
        if not t and i + 1 < len(turns):
            continue
        if len(text) + 5 > limit:
            return text + "+"
        text += str(run) + ("R" if t > 0 else "L" if t < 0 else "")
        run = 0
    return text


# ---- Calibration data capture ----------------------------------------------------------

class CalibrationCapture:
    """Collects '@D' blocks (CAL commands) and saves them as CSV files."""

    def __init__(self, directory, save=True):
        self.directory = directory
        self.save_files = save
        self.active = None
        self.last_path = None

    def progress(self):
        return None if self.active is None else (self.active["test"], len(self.active["rows"]))

    def feed(self, line):
        """Returns a (style, message) for the log, or None."""
        body = line[2:].strip()
        word, _, rest = body.partition(" ")
        if word == "BEGIN":
            self.active = {"test": rest or "?", "info": [], "cols": None, "rows": []}
            return "info", "receiving calibration data: " + (rest or "?")
        if self.active is None:
            return None
        if word == "INFO":
            self.active["info"].append(rest)
        elif word == "COLS":
            self.active["cols"] = rest
        elif word == "END":
            capture, self.active = self.active, None
            if not self.save_files:
                return "info", "calibration data: %d samples (replay: not saved)" % len(capture["rows"])
            try:
                path = self._save(capture, rest)
            except OSError as exc:
                return "bad", "could not save the data: %s" % exc
            self.last_path = path
            return "ok", "data saved in %s (%d samples). Add measurements with /note <text>" % (
                os.path.relpath(path), len(capture["rows"]))
        else:
            capture = self.active
            capture["rows"].append(body)
        return None

    def _save(self, capture, end):
        os.makedirs(self.directory, exist_ok=True)
        test = re.sub(r"[^A-Za-z0-9]+", "_", capture["test"].split(" ")[0]).strip("_") or "cal"
        stamp = time.strftime("%Y-%m-%d_%H-%M-%S")
        path = os.path.join(self.directory, "%s_%s.csv" % (stamp, test.lower()))
        suffix = 2
        while os.path.exists(path):     # two dumps in the same second must not overwrite each other
            path = os.path.join(self.directory, "%s_%s_%d.csv" % (stamp, test.lower(), suffix))
            suffix += 1
        with open(path, "x", encoding="utf-8") as f:
            f.write("# rat_sw calibration data (tools/calib_analyze.py)\n")
            f.write("# captured: %s\n" % time.strftime("%Y-%m-%d %H:%M:%S"))
            f.write("# test: %s\n" % capture["test"])
            for info in capture["info"]:
                f.write("# %s\n" % info)
            f.write("# end: %s\n" % end)
            if capture["cols"]:
                f.write(capture["cols"] + "\n")
            for row in capture["rows"]:
                f.write(row + "\n")
        return path

    def note(self, text):
        if not self.last_path:
            return "warn", "no calibration data to add the note to"
        with open(self.last_path, "r+", encoding="utf-8") as f:
            lines = f.readlines()
            header = 0
            while header < len(lines) and lines[header].startswith("#"):
                header += 1
            lines.insert(header, "# note: %s\n" % text)
            f.seek(0)
            f.writelines(lines)
            f.truncate()
        return "ok", "note added to %s" % os.path.relpath(self.last_path)


# ---- Rendering: 24-bit colour, the maze as half-block pixels ------------------------------

BG, BAR = (6, 8, 14), (16, 20, 36)
TEXT, DIM, FAINT = (190, 200, 215), (92, 104, 128), (44, 54, 78)
WHITE, BLACK = (255, 255, 255), (0, 0, 0)
CYAN, VIOLET, MAGENTA, PINK = (70, 230, 255), (150, 90, 255), (255, 50, 200), (255, 130, 220)
ORANGE, GOLD, GREEN, RED = (255, 140, 30), (255, 220, 90), (90, 255, 140), (255, 60, 70)
WALL_COLOUR, WALL_GLOW, UNSEEN = CYAN, (0, 70, 95), (20, 30, 52)
FLOOR = ((74, 14, 70), (22, 22, 78), (6, 34, 46))   # cells to the goal: there, half way, the farthest
BRAND = (CYAN, (60, 120, 255), VIOLET, MAGENTA)     # the header block and the logo, left to right
LOG_COLOUR = {"": TEXT, "dim": DIM, "boot": DIM, "bad": RED, "ok": GREEN, "head": MAGENTA,
              "warn": GOLD, "info": CYAN, "sent": GOLD}
SPARK, SPIN, MOUSE = " ▁▂▃▄▅▆▇█", "◐◓◑◒", "~(__^·>"

FONT = {    # 3x5 pixels, rows top to bottom: the run's clock and the banners
    "0": "111 101 101 101 111", "1": "010 110 010 010 111", "2": "111 001 111 100 111",
    "3": "111 001 111 001 111", "4": "101 101 111 001 001", "5": "111 100 111 001 111",
    "6": "111 100 111 101 111", "7": "111 001 001 001 001", "8": "111 101 111 101 111",
    "9": "111 101 111 001 111", ".": "0 0 0 0 1", ":": "0 1 0 1 0", " ": "0 0 0 0 0",
    "A": "111 101 111 101 101", "B": "110 101 110 101 110", "D": "110 101 101 101 110",
    "E": "111 100 111 100 111", "F": "111 100 111 100 100", "G": "111 100 101 101 111",
    "I": "111 010 010 010 111", "L": "100 100 100 100 111", "O": "111 101 101 101 111",
    "R": "110 101 110 101 101", "T": "111 010 010 010 010", "Y": "101 101 010 010 010",
}
FONT = {c: rows.split() for c, rows in FONT.items()}
LOGOS = ((     # the largest that fits is drawn
    "██████╗  █████╗ ████████╗ █████╗ ████████╗██████╗  ██████╗ ███╗   ██╗",
    "██╔══██╗██╔══██╗╚══██╔══╝██╔══██╗╚══██╔══╝██╔══██╗██╔═══██╗████╗  ██║",
    "██████╔╝███████║   ██║   ███████║   ██║   ██████╔╝██║   ██║██╔██╗ ██║",
    "██╔══██╗██╔══██║   ██║   ██╔══██║   ██║   ██╔══██╗██║   ██║██║╚██╗██║",
    "██║  ██║██║  ██║   ██║   ██║  ██║   ██║   ██║  ██║╚██████╔╝██║ ╚████║",
    "╚═╝  ╚═╝╚═╝  ╚═╝   ╚═╝   ╚═╝  ╚═╝   ╚═╝   ╚═╝  ╚═╝ ╚═════╝ ╚═╝  ╚═══╝",
), (
    "█▀█ ▄▀█ ▀█▀ ▄▀█ ▀█▀ █▀█ █▀█ █▄ █",
    "█▀▄ █▀█  █  █▀█  █  █▀▄ █ █ █ ▀█",
    "▀ ▀ ▀ ▀  ▀  ▀ ▀  ▀  ▀ ▀ ▀▀▀ ▀  ▀",
))


def mix(a, b, t):
    t = 0.0 if t < 0 else 1.0 if t > 1 else t
    return (int(a[0] + (b[0] - a[0]) * t), int(a[1] + (b[1] - a[1]) * t), int(a[2] + (b[2] - a[2]) * t))


def add(a, b, k=1.0):
    return (min(255, int(a[0] + b[0] * k)), min(255, int(a[1] + b[1] * k)), min(255, int(a[2] + b[2] * k)))


def ramp(stops, t):
    t = min(max(t, 0.0), 1.0) * (len(stops) - 1)
    i = min(int(t), len(stops) - 2)
    return mix(stops[i], stops[i + 1], t - i)


def sanitize(text):
    """Printable ASCII only: control characters would break the screen layout."""
    return "".join(c if 32 <= ord(c) < 127 else "?" for c in text)


class Screen:
    """A rows x cols grid of (char, foreground, background). Writes are clipped, never raise."""

    def __init__(self, rows, cols):
        self.rows, self.cols = rows, cols
        self.ch = [[" "] * cols for _ in range(rows)]
        self.fg = [[TEXT] * cols for _ in range(rows)]
        self.bg = [[BG] * cols for _ in range(rows)]

    def put(self, y, x, text, fg=TEXT, bg=None, end=None):
        """Writes text up to the column `end`. Returns the column after it."""
        end = self.cols if end is None else min(self.cols, end)
        if 0 <= y < self.rows:
            for c in text:
                if x >= end:
                    break
                if x >= 0:
                    self.ch[y][x], self.fg[y][x] = c, fg
                    if bg is not None:
                        self.bg[y][x] = bg
                x += 1
        return min(x, end)

    def bar(self, y, colour):
        if 0 <= y < self.rows:
            self.bg[y] = [colour] * self.cols

    def row_text(self, y):
        return "".join(self.ch[y])


def draw_frame(scr, top, left, height, width, tag, title, note="", colour=None):
    """A box with its title in the top border: ╭─┤ 0x01 MAZE ├───╮, cyan to violet across the screen."""
    def edge(x):
        return colour or ramp((mix(CYAN, BG, 0.45), mix(VIOLET, BG, 0.35)), x / max(1, scr.cols - 1))
    right, bottom = left + width - 1, top + height - 1
    for x in range(left + 1, right):
        scr.put(top, x, "─", edge(x))
        scr.put(bottom, x, "─", edge(x))
    for y in range(top + 1, bottom):
        scr.put(y, left, "│", edge(left))
        scr.put(y, right, "│", edge(right))
    for y, x, c in ((top, left, "╭"), (top, right, "╮"), (bottom, left, "╰"), (bottom, right, "╯")):
        scr.put(y, x, c, edge(x))
    end = right - 1
    x = scr.put(top, left + 2, "┤ ", edge(left + 2), end=end)
    x = scr.put(top, x, tag + " ", MAGENTA, end=end)
    x = scr.put(top, x, title, colour or CYAN, end=end)
    x = scr.put(top, x, note, DIM, end=end)
    scr.put(top, x, " ├", edge(x), end=end)


def big_text(scr, y, x, text, top, bottom, end):
    """Text in the 3x5 font, each font pixel two columns by one row. Returns the column after it."""
    for c in text:
        glyph = FONT.get(c, FONT[" "])
        for r, bits in enumerate(glyph):
            for i, bit in enumerate(bits):
                if bit == "1":
                    scr.put(y + r, x + 2 * i, "██", mix(top, bottom, r / 4), end=end)
        x += 2 * len(glyph[0]) + 2
    return x


def telemetry_age(model, now):
    if model.last_telemetry is None:
        return "no telemetry"
    age = now - model.last_telemetry
    return "tlm %.1fs" % age if age < 99 else "tlm --"


def view_size(model, full):
    """Cells to draw: the practice area grows with what was explored; any
    maze reaching beyond 8 cells is drawn whole."""
    if full:
        return MAZE, MAZE
    nx, ny = model.goal[2] + 1, model.goal[3] + 1
    for x in range(MAZE):
        for y in range(MAZE):
            if model.visited[x][y]:
                nx, ny = max(nx, x + 1), max(ny, y + 1)
    if model.pose:
        nx, ny = max(nx, model.pose[0] + 1), max(ny, model.pose[1] + 1)
    if nx > 8 or ny > 8:
        return MAZE, MAZE
    return max(nx, 4), max(ny, 3)


class Effects:
    """What the telemetry does not carry: the drawn robot gliding from cell to cell, the trail
    it leaves, the run's clock, the banners. Times are the session's (a replay's own)."""

    RUN_LINE = re.compile(r"^route .* t=(\d+)ms")    # the robot's own time of a continuous move

    def __init__(self, rate=1.0):
        self.rate = rate                # session seconds per second: a replay's speed
        self.lively = False             # --effects: the map moves during a run too
        self.stamp = self.wall = self.moved = 0.0
        self.path, self.s = [], 0.0     # cells the drawn robot still has to go through, and how far it is
        self.ghost = None               # cells/s of a run drawn from the route: its pose only comes at the end
        self.cell = None                # last cell the robot reported
        self.ribbon = []                # cells behind the drawn robot in this run
        self.afterglow = collections.deque(maxlen=24)
        self.ripples = []               # (x, y, born, colour, reach)
        self.banner = None              # (text, colour, born)
        self.alert_until = 0.0
        self.run = None                 # (start, end or None)
        self.lap, self.laps, self.robot_ms = None, [], 0

    def line(self, model, planner, text, stamp, wall):
        """Feeds a received line to the model. False if it is not telemetry."""
        self.stamp, self.wall = stamp, wall
        was, fresh = model.activity, None
        if self.lively and text.startswith("@C") and len(text) > 3 and text[2] in HEX and text[3] in HEX:
            fresh = (HEX.index(text[2]), HEX.index(text[3]))
            fresh = None if model.visited[fresh[0]][fresh[1]] else fresh
        if not model.apply(text, wall):
            self.words(model, text, stamp, wall)
            return False
        if model.activity != was:
            self.on_activity(model, planner, was, stamp)
        if model.pose and model.pose[:2] != self.cell:
            self.on_pose(model.pose[:2])
        if fresh and model.visited[fresh[0]][fresh[1]]:     # a ripple where a cell is discovered
            self.ripples.append((fresh[0], fresh[1], wall, CYAN, 0.5))
        return True

    def words(self, model, text, stamp, wall):
        took = self.RUN_LINE.match(text)
        if took and model.activity == "F":
            self.robot_ms += int(took.group(1))
        elif "Goal reached" in text:
            if model.activity == "F" and self.run:     # the robot's own time, else when the line came
                self.lap = self.robot_ms / 1000 if self.robot_ms else stamp - self.run[0]
                self.laps.append(self.lap)
            x0, y0, x1, y1 = model.goal
            self.banner = ("GOAL", MAGENTA, wall)
            self.ripples.append(((x0 + x1) / 2, (y0 + y1) / 2, wall, MAGENTA, 1.6))
        elif text.startswith("End: "):
            word = text[5:].split(" ")[0]
            if word in ("OK", "FAILED", "ABORTED"):
                self.banner = {"OK": ("READY", GREEN, wall), "FAILED": ("FAILED", RED, wall),
                               "ABORTED": ("ABORTED", GOLD, wall)}[word]
        elif text.startswith("!!"):
            self.alert_until = wall + 3.0

    def on_activity(self, model, planner, was, stamp):
        act = model.activity
        if act in "GWFK" and (was not in RUNNING or was == "C"):
            self.run, self.lap, self.robot_ms = (stamp, None), None, 0
            self.ribbon = [self.pos()] if self.path else []
        elif act not in RUNNING and was in RUNNING and self.run:
            self.run = (self.run[0], stamp)
        if act in "FR" and self.path:
            cells = list(planner.overlay(model).route)
            if cells:
                self.path += cells
                self.ghost = len(cells) / ((0.6 if act == "F" else 1.1) + 0.28 * len(cells))

    def on_pose(self, new):
        self.cell = new
        ghost, self.ghost = self.ghost, None
        if not self.path:
            self.path, self.s = [new], 0.0
            return
        ahead = self.path[int(self.s):]
        if ghost and new in ahead:      # the run stopped here: at its end or before it
            del self.path[len(self.path) - ahead[::-1].index(new):]
            return
        last = self.path[-1]
        dx, dy = new[0] - last[0], new[1] - last[1]
        if not ghost and (dx == 0 or dy == 0):
            n = abs(dx) + abs(dy)
            self.path += [(last[0] + dx * i // n, last[1] + dy * i // n) for i in range(1, n + 1)]
        else:       # somewhere else: carried by hand, or a run that stopped behind its drawing
            self.path, self.s, self.ribbon = [new], 0.0, []

    def advance(self, wall):
        """Moves the drawn robot on: called once per frame."""
        dt, self.moved = min(0.5, max(0.0, wall - self.moved)), wall
        n = len(self.path) - 1
        if self.s < n:
            v = self.ghost * self.rate if self.ghost else max(7.0, (n - self.s) / 1.2)
            s = min(n, self.s + v * dt)
            self.ribbon += self.path[int(self.s) + 1:int(s) + 1]
            self.s = s
        k = min(int(self.s), max(n, 0))
        if k > 0:
            del self.path[:k]
            self.s -= k
        if self.path:
            self.afterglow.append(self.pos() + (wall,))
        self.ripples = [r for r in self.ripples if wall - r[2] < 0.9]

    def busy(self, wall):
        return self.s < len(self.path) - 1 or self.ripples or wall < self.alert_until \
            or (self.banner and wall - self.banner[2] < 1.8)

    def pos(self):
        k = int(self.s)
        if k + 1 >= len(self.path):
            return (float(self.path[-1][0]), float(self.path[-1][1]))
        f, a, b = self.s - k, self.path[k], self.path[k + 1]
        return (a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f)

    def heading(self, model):
        k = int(self.s)
        if k + 1 < len(self.path):
            step = (self.path[k + 1][0] - self.path[k][0], self.path[k + 1][1] - self.path[k][1])
            for h in range(4):
                if (DX[h], DY[h]) == step:
                    return h
        return model.pose[2] if model.pose else 0

    def timer(self, wall):
        """(seconds, state): idle, run, lap (a speed run's time to the goal) or done."""
        if self.run is None:
            return None, "idle"
        if self.lap is not None:
            return self.lap, "lap"
        if self.run[1] is not None:
            return self.run[1] - self.run[0], "done"
        return max(0.0, self.stamp + (wall - self.wall) * self.rate - self.run[0]), "run"


def flood(model):
    """Cells to the goal through every wall not seen: the classic flood fill, for the floor's colour."""
    dist = [[INF] * MAZE for _ in range(MAZE)]
    queue = collections.deque(model.goal_cells())
    for x, y in queue:
        dist[x][y] = 0
    while queue:
        x, y = queue.popleft()
        for h in range(4):
            nx, ny = x + DX[h], y + DY[h]
            if 0 <= nx < MAZE and 0 <= ny < MAZE and model.wall(x, y, h) != WALL and dist[nx][ny] > dist[x][y] + 1:
                dist[nx][ny] = dist[x][y] + 1
                queue.append((nx, ny))
    return dist


# Pixels across and down in a character: sextants (2x3, two colours a character), or half
# blocks (1x2, coarser) for the terminals that do not draw sextants.
GRAINS = {"sextant": (2, 3), "half": (1, 2)}
NEAR = ((-1, 0, 1), (1, 0, 1), (0, -1, 1), (0, 1, 1), (-1, -1, .5), (-1, 1, .5), (1, -1, .5), (1, 1, .5))


def luma(c):
    return c[0] * 3 + c[1] * 6 + c[2]


class MazePainter:
    """The maze as pixels, walls on the grid lines; a character holds a few of them (GRAINS)."""

    def __init__(self):
        self.key = self.base = None
        self.free_posts = []    # (row, column, phase) of the posts no wall touches
        self.robot = None       # (column, row) of the robot's pixel in the last picture
        self.glyphs, self.rows = {}, {}     # characters already worked out: by their pixels, by screen row

    def static(self, model, size, pitch, grain):
        """Floor, walls and their glow: redrawn only when the map changes."""
        key = (model.version, model.goal, size, pitch, grain)
        if key == self.key:
            return self.base
        (nx, ny), (pw, ph) = size, pitch
        w, h = nx * pw + 1, ny * ph + 1
        px = [[BG] * (-(-w // grain[0]) * grain[0]) for _ in range(-(-h // grain[1]) * grain[1])]   # whole characters
        walls = []
        dist = flood(model)
        seen = [(x, y) for x in range(nx) for y in range(ny) if model.visited[x][y]]
        far = max([dist[x][y] for x, y in seen if dist[x][y] < INF] or [1]) or 1
        floor = {}

        def row(py):    # pixel row of the horizontal grid line py
            return (ny - py) * ph

        def segment(state, cells, points):
            floors = [floor[c] for c in cells if c in floor]
            for k, (r, c) in enumerate(points, 1):
                if state == WALL:
                    walls.append((r, c))
                elif state == UNKNOWN:
                    if k % (2 if len(points) < 6 else 3) == 0:
                        px[r][c] = UNSEEN
                elif floors:    # an opening: the floor goes through
                    px[r][c] = mix(floors[0], floors[-1], 0.5) if len(floors) == 2 else mix(BG, floors[0], 0.5)

        for x, y in seen:
            floor[(x, y)] = ramp(FLOOR, dist[x][y] / far)
            for r in range(row(y + 1) + 1, row(y)):
                px[r][x * pw + 1:x * pw + pw] = [floor[(x, y)]] * (pw - 1)
        states = {}
        for py in range(ny + 1):
            for x in range(nx):
                states["h", x, py] = WALL if py == 0 else model.wall(x, py - 1, 0)
                segment(states["h", x, py], ((x, py - 1), (x, py)), [(row(py), x * pw + k) for k in range(1, pw)])
        for gx in range(nx + 1):
            for y in range(ny):
                states["v", gx, y] = WALL if gx == 0 else model.wall(gx - 1, y, 1)
                segment(states["v", gx, y], ((gx - 1, y), (gx, y)), [(row(y + 1) + k, gx * pw) for k in range(1, ph)])
        self.free_posts = []
        for gx in range(nx + 1):
            for py in range(ny + 1):
                if any(states.get(s) == WALL for s in (("h", gx - 1, py), ("h", gx, py), ("v", gx, py - 1), ("v", gx, py))):
                    walls.append((row(py), gx * pw))
                else:
                    px[row(py)][gx * pw] = UNSEEN
                    self.free_posts.append((row(py), gx * pw, gx + py))
        glow = collections.defaultdict(float)
        for r, c in walls:
            px[r][c] = WALL_COLOUR
            for dr, dc, k in NEAR:
                glow[r + dr, c + dc] += k
        for r, c in walls:
            glow.pop((r, c), None)
        for (r, c), n in glow.items():
            if 0 <= r < h and 0 <= c < w:
                px[r][c] = add(px[r][c], WALL_GLOW, min(1.0, n / 3))
        self.key, self.base = key, px
        return px

    def pixels(self, model, ov, fx, start, size, pitch, grain, now):
        """The whole picture: rows of colours, whole characters of them."""
        px = [r[:] for r in self.static(model, size, pitch, grain)]
        (nx, ny), (pw, ph) = size, pitch
        w, h, fine = nx * pw + 1, ny * ph + 1, min(pw, ph) > 4

        def centre(x, y):   # (column, row) of a cell's centre, fractions of a cell allowed
            return int(round(x * pw)) + pw // 2, int(round((ny - 1 - y) * ph)) + ph // 2

        def light(c, r, colour, k=1.0):
            if 0 <= r < h and 0 <= c < w:
                px[r][c] = add(px[r][c], colour, k)

        def paint(c, r, colour, k=1.0):
            if 0 <= r < h and 0 <= c < w:
                px[r][c] = mix(px[r][c], colour, k)

        def polyline(cells):
            points = [centre(*c) for c in cells]
            out = points[:1]
            for (c0, r0), (c1, r1) in zip(points, points[1:]):
                n = max(abs(c1 - c0), abs(r1 - r0))
                out += [(c0 + (c1 - c0) * i // n, r0 + (r1 - r0) * i // n) for i in range(1, n + 1)]
            return out

        # During a run nothing moves but the robot: the goal pulses and the path flows only at
        # rest. With --effects they always do, the candidates blink and a sweep crosses what is unseen.
        running = model.activity in RUNNING
        still = running and not fx.lively
        if fx.lively:
            for r, c, phase in self.free_posts:
                px[r][c] = mix(UNSEEN, (50, 110, 170), max(0.0, math.sin(phase * 0.55 - now * 2.5)) ** 6)
        pulse = 0.5 if still else 0.5 + 0.5 * math.sin(now * 4)
        for x, y in model.goal_cells():         # the goal glows
            c0, r0 = x * pw, (ny - 1 - y) * ph
            for r in range(r0 + 1, r0 + ph):
                for c in range(c0 + 1, c0 + pw):
                    edge = fine and (r in (r0 + 1, r0 + ph - 1) or c in (c0 + 1, c0 + pw - 1))
                    paint(c, r, MAGENTA, 0.30 + 0.35 * pulse if edge else 0.22 + 0.10 * pulse)

        line = []
        if running and ov.route and model.pose:     # where it goes now, else the verified fast path
            line, base, hot = polyline([model.pose[:2]] + list(ov.route)), (30, 120, 60), (190, 255, 190)
        elif not running and ov.fast_path:
            line, base, hot = polyline([start] + list(ov.fast_path)), (0, 110, 140), (210, 255, 255)
        for i, (c, r) in enumerate(line):
            flow = 0.0 if still else max(0.0, 1 - ((now * 22 - i * 6 / pw) % 26) / 7)
            paint(c, r, mix(base, hot, flow), 0.9)

        for x, y in ov.candidates if not fx.lively or int(now * 3) % 2 else ():
            c, r = centre(x, y)
            for dc, dr in ((0, 0), (1, 0), (-1, 0), (0, 1), (0, -1)) if fine else ((0, 0),):
                paint(c + dc, r + dr, PINK, 0.9)

        where = fx.pos() if fx.path else model.pose[:2] if model.pose else None
        self.robot = None
        if where:       # distances in cells: the pixels need not be square
            tint = GOLD if model.activity == "F" else ORANGE
            radius = 0.3 if min(pw, ph) >= 4 else 0.0
            halo = radius + 2.5 / max(pw, ph)
            reach = [(dc, dr, math.hypot(dc / pw, dr / ph)) for dr in range(-int(halo * ph), int(halo * ph) + 1)
                     for dc in range(-int(halo * pw), int(halo * pw) + 1)]
            glow = {}
            for x, y, born in fx.afterglow:         # phosphor behind the robot
                k = math.exp(-(now - born) / 0.35)
                c0, r0 = centre(x, y)
                for dc, dr, d in reach:
                    v = k * (1 - d / halo)
                    if v > glow.get((c0 + dc, r0 + dr), 0.02):
                        glow[(c0 + dc, r0 + dr)] = v
            for (c, r), v in glow.items():
                light(c, r, tint, v * 0.55)
            ribbon = polyline(fx.ribbon + [where]) if fx.ribbon else []    # the light wall it leaves
            for i, (c, r) in enumerate(ribbon):
                age = (len(ribbon) - 1 - i) / pw
                paint(c, r, mix(tint, WHITE, 0.5 if age < 2 else 0.0), max(0.35, 1 - age / 14))
            c0, r0 = self.robot = centre(*where)
            hd = fx.heading(model)
            for dc, dr, d in reach:                 # a disc, white at the core and towards its nose
                ahead = (dc / pw * DX[hd] - dr / ph * DY[hd]) / radius if radius else 0.0
                if d <= radius * 0.6 or (d <= radius and ahead > 0.5):
                    paint(c0 + dc, r0 + dr, WHITE)
                elif d <= radius:
                    paint(c0 + dc, r0 + dr, mix(tint, BG, max(0.0, -ahead) * 0.5))

        for x, y, born, colour, reach in fx.ripples:
            age = now - born
            c0, r0 = centre(x, y)
            cells = age * 3.4 * reach
            steps = int(6.3 * (1 + cells * pw)) + 8
            for i in range(steps):
                a = i * 6.2832 / steps
                light(int(round(c0 + (1 + cells * pw) * math.cos(a))), int(round(r0 + (1 + cells * ph) * math.sin(a))),
                      colour, max(0.0, 1 - age / 0.9) ** 2 * 0.9)

        if fx.banner and 0 <= now - fx.banner[2] < 1.8:
            text, colour, born = fx.banner
            age = now - born
            glyphs = [FONT.get(c, FONT[" "]) for c in text]
            cols = sum(len(g[0]) + 1 for g in glyphs) - 1
            ky = max(1, min(ph // 2 + 1, (w - 4) * ph // (cols * pw)))      # a font pixel, square on screen
            kx = max(1, min(round(ky * pw / ph), (w - 2) // cols))
            alpha = min(1.0, (1.8 - age) / 0.4)
            c0, r0 = (w - cols * kx) // 2, (h - 5 * ky) // 2
            for r in range(max(0, r0 - ky - 1), min(h, r0 + 6 * ky + 1)):
                px[r] = [mix(v, BLACK, 0.78 * alpha) for v in px[r]]
            for g in glyphs:
                for gr, bits in enumerate(g):
                    ink = WHITE if age < 0.12 else mix(WHITE, colour, 0.25 + 0.75 * gr / 4)
                    for gc, bit in enumerate(bits):
                        if bit == "1":
                            for r in range(r0 + gr * ky, r0 + gr * ky + ky):
                                for c in range(c0 + gc * kx, c0 + gc * kx + kx):
                                    paint(c, r, ink, alpha)
                c0 += (len(g[0]) + 1) * kx
        return px

    def glyph(self, colours):
        """(char, foreground, background) for the pixels of a character, rows first: a
        character has two colours, so its pixels go to the nearer of their two extremes."""
        if colours in self.glyphs:
            return self.glyphs[colours]
        if len(self.glyphs) > 60000:
            self.glyphs.clear()
        lo, hi = min(colours, key=luma), max(colours, key=luma)
        if lo == hi:
            out = (" ", lo, lo)
        else:
            on = [sum((a - b) ** 2 for a, b in zip(c, hi)) < sum((a - b) ** 2 for a, b in zip(c, lo)) for c in colours]
            lit, dark = [c for c, v in zip(colours, on) if v], [c for c, v in zip(colours, on) if not v]
            fg = tuple(sum(c[i] for c in lit) // len(lit) for i in range(3))
            bg = tuple(sum(c[i] for c in dark) // len(dark) for i in range(3))
            v = sum(1 << i for i, lit in enumerate(on) if lit)     # a code each, but the two columns are older characters
            out = ("▌" if v == 21 else "▐" if v == 42 else chr(0x1FB00 + v - 1 - (v > 21) - (v > 42)), fg, bg)
        self.glyphs[colours] = out
        return out


PITCHES = (24, 20, 16, 14, 12, 10, 8, 6, 5, 4, 3, 2)    # columns per cell: the largest that fits is drawn


def maze_shape(nx, ny, k, grain):
    """Characters across and down of the maze with cells k columns wide; None if its pixels
    do not make whole cells."""
    if k * grain[1] % 2:
        return None
    return nx * k + 1, (ny * k * grain[1] // 2 + grain[1]) // grain[1]


def maze_fit(nx, ny, width, height, grain):
    """(columns per cell, axes): the largest cells that fit, with the hex axes and the legend if
    they fit too; else small cells, the view following the robot."""
    for k in PITCHES:
        shape = maze_shape(nx, ny, k, grain)
        for axes in (1, 0):
            if shape and shape[0] + 2 * axes <= width and shape[1] + 2 * axes <= height:
                return k, axes
    return 4, 0


def maze_panel_width(nx, ny, width, height, grain):
    """Width of the whole maze at the largest cells fitting (width, height), or None."""
    k, axes = maze_fit(nx, ny, width, height, grain)
    cols, rows = maze_shape(nx, ny, k, grain)
    return cols + 2 * axes if cols + 2 * axes <= width and rows + 2 * axes <= height else None


def draw_maze(scr, top, left, height, width, model, ov, view, start, now):
    """Draws the maze centred in the box (top, left, height, width)."""
    nx, ny = view_size(model, view.full)
    grain = sx, sy = GRAINS[view.pixels]
    k, axes = maze_fit(nx, ny, width, height, grain)
    pw, ph = k * sx, k * sy // 2
    px = view.painter.pixels(model, ov, view.fx, start, (nx, ny), (pw, ph), grain, now)
    w, h = len(px[0]) // sx, len(px) // sy      # in characters
    vw, vh = min(w, width - 2 * axes), min(h, height - 2 * axes)
    if vw < 1 or vh < 1:
        return
    col, row = view.painter.robot or (0, len(px))       # the view follows the robot
    c0 = min(max(0, col // sx - vw // 2), w - vw)
    r0 = min(max(0, row // sy - vh // 2), h - vh)
    x0 = left + 2 * axes + (width - 2 * axes - vw) // 2
    y0 = top + (height - 2 * axes - vh) // 2
    for r in range(vh):
        y, rows = y0 + r, px[(r0 + r) * sy:(r0 + r + 1) * sy]
        if sx == 1:     # half blocks: a colour each, no choice to make
            scr.ch[y][x0:x0 + vw] = "▀" * vw
            scr.fg[y][x0:x0 + vw] = rows[0][c0:c0 + vw]
            scr.bg[y][x0:x0 + vw] = rows[1][c0:c0 + vw]
            continue
        was = view.painter.rows.get(r)
        if was and was[0] == (c0, vw, rows):     # most rows do not change from a frame to the next
            scr.ch[y][x0:x0 + vw], scr.fg[y][x0:x0 + vw], scr.bg[y][x0:x0 + vw] = was[1:]
            continue
        for c in range(vw):
            colours = []
            for line in rows:
                colours += line[(c0 + c) * sx:(c0 + c + 1) * sx]
            scr.ch[y][x0 + c], scr.fg[y][x0 + c], scr.bg[y][x0 + c] = view.painter.glyph(tuple(colours))
        view.painter.rows[r] = ((c0, vw, rows), scr.ch[y][x0:x0 + vw], scr.fg[y][x0:x0 + vw], scr.bg[y][x0:x0 + vw])
    if not axes:
        return
    here = (col // pw, ny - 1 - row // ph) if view.painter.robot else (-1, -1)
    for x in range(nx):     # hex coordinates, the robot's row and column lit
        scr.put(y0 + vh, x0 + (x * pw + pw // 2) // sx - c0, HEX[x], GOLD if x == here[0] else FAINT)
    for y in range(ny):
        scr.put(y0 + ((ny - 1 - y) * ph + ph // 2) // sy - r0, x0 - 2, HEX[y], GOLD if y == here[1] else FAINT)
    x, end = x0 - 2, left + width
    for glyph, colour, label in (("●", ORANGE, "robot"), ("━", ORANGE, "trail"), ("━", GREEN, "route"),
                                 ("━", CYAN, "fast"), ("◆", PINK, "candidate"), ("■", MAGENTA, "goal")):
        if x + len(label) + 2 > end:
            return
        x = scr.put(y0 + vh + 1, x, glyph, colour)
        x = scr.put(y0 + vh + 1, x, " %s  " % label, DIM, end=end)
    if x + 17 <= end:       # the floor's colours: cells to the goal
        x = scr.put(y0 + vh + 1, x, "near ", DIM)
        for i in range(8):
            x = scr.put(y0 + vh + 1, x, "█", add(ramp(FLOOR, i / 7), (30, 30, 30)))
        scr.put(y0 + vh + 1, x, " far", DIM)


def draw_splash(scr, top, left, height, width, view, now):
    """Before any telemetry: hex rain behind the logo, waiting for the robot."""
    for col in range(0, width, 2):
        seed = col * 7919 % 101
        speed, length = 4 + seed % 7, 3 + seed % 6
        head = int(now * speed + seed * 3) % (height + length)
        for k in range(length):
            row = head - k
            if 0 <= row < height:
                scr.put(top + row, left + col, HEX[(seed + row * 7 + int(now * 3)) % 16],
                        (170, 255, 200) if k == 0 else mix((0, 150, 70), BG, k / length))
    logo = next((logo for logo in LOGOS if len(logo[0]) + 4 <= width and len(logo) + 6 <= height), ())
    if view.connected:
        status = "link up, waiting for telemetry"
    else:
        status = view.link_text or "opening %s" % view.source
    status += "█" if int(now * 2) % 2 == 0 else " "
    lines = [(text, None) for text in logo]
    lines += [("", DIM), ("MICROMOUSE TELEMETRY CONSOLE", DIM), ("", DIM), (status, GOLD)]
    block_w = min(width, max(len(text) for text, _ in lines) + 4)
    y0 = top + max(0, (height - len(lines) - 2) // 2)
    for row in range(max(top, y0 - 1), min(top + height, y0 + len(lines) + 1)):
        scr.put(row, left + (width - block_w) // 2, " " * block_w)
    for i, (text, colour) in enumerate(lines):
        x = left + max(0, (width - len(text)) // 2)
        if y0 + i >= top + height:
            break
        if colour is None:      # a logo row: the brand's colours across, a wave of light, shadows dim
            for j, c in enumerate(text):
                shine = max(0.0, math.sin(j * 0.12 - now * 3)) ** 8 * 0.6
                ink = mix(ramp(BRAND, j / len(text)), WHITE, shine) if c in "█▀▄" else (60, 60, 110)
                scr.put(y0 + i, x + j, c, ink, end=left + width)
        else:
            scr.put(y0 + i, x, text, colour, end=left + width)
    row = y0 + len(lines) + 1
    if row < top + height:
        pos = int(now * 12) % (width + len(MOUSE)) - len(MOUSE)
        scr.put(row, left + max(0, pos), MOUSE[max(0, -pos):max(0, width - pos)], CYAN)


LOG_STYLES = (
    (lambda t: t.startswith("!!"), "bad"),
    (lambda t: "Goal reached" in t or t.startswith("End: OK"), "ok"),
    (lambda t: t.startswith("=="), "head"),
    (lambda t: t.startswith("End:") or t.startswith("? "), "warn"),
    (lambda t: t.startswith("--"), "info"),
    (lambda t: t.startswith(("forward", "route", "explore", "turn", "front aligned", "squared", "IR mm")), "dim"),
)


def log_style(text):
    for test, style in LOG_STYLES:
        if test(text):
            return style
    return ""


HELP_LINES = [
    ("head", "KEYS"),
    ("keyline", "  Enter        send the command"),
    ("keyline", "  Up/Down      command history"),
    ("keyline", "  PgUp/PgDn    scroll the log (Home/End)"),
    ("keyline", "  Tab          log / this help"),
    ("keyline", "  Ctrl+X       immediate STOP of the run"),
    ("keyline", "  Ctrl+L       redraw the screen"),
    ("keyline", "  Esc          quit"),
    ("keyline", "  /full /clear  16x16 view, clear the log"),
    ("keyline", "  /pixels      coarser maze, for terminals that do not draw sextants"),
    ("keyline", "  /effects     the map moves during a run too: sweep, ripples, flowing route"),
    ("keyline", "  /note text   adds a measurement to the last calibration file"),
    ("", ""),
    ("head", "ROBOT (all in README.md, Bluetooth console)"),
    ("", "  MODE n   1 search | 2 k race: 1/2 left/right follower 3 no curves 4 800/300 5 900/400 6 900/480 | 3 erase"),
    ("", "  START STOP PAUSE RESUME STEP ON|OFF"),
    ("", "  STATUS MAP IR WALLS SYNC TELEM ON|OFF"),
    ("", "  SPD FAST CURVE ACCEL TURN TACCEL TURNTICKS n   KP KD f   LOG 0-2"),
    ("", "  GOAL x y [x1 y1]  SAVE ERASE HOME DEFAULTS RESET"),
    ("", "  TUNE [name value]  CONT ON|OFF"),
    ("", "  CAL NOISE|STRAIGHT|TURN|CURVE|STEP|IR|DUMP|RUN  calibration"),
    ("", ""),
    ("head", "MAP"),
    ("", "  the robot is the white and orange disc, its trail behind it"),
    ("", "  green: the route it plans; cyan, stopped: the verified fast path"),
    ("", "  the floor's colour is the distance to the goal; dots = walls not seen"),
    ("", "  speed runs send no pose until they end: the robot is drawn along"),
    ("", "  the route at an estimated pace, then where it reports"),
    ("", "  the clock of a speed run is the robot's own (LOG 2), else the link's"),
]


class ViewState:
    """What the screen shows besides the model: log, input, panels."""

    def __init__(self):
        self.log = collections.deque(maxlen=5000)
        self.scroll = 0
        self.show_help = False
        self.full = False
        self.pixels = "sextant"     # the maze's grain: GRAINS
        self.input = ""
        self.connected = False
        self.link_text = ""
        self.source = ""
        self.replay = False
        self.capture = None     # (test, samples) while a calibration dump arrives
        self.rx_times = collections.deque(maxlen=4096)     # arrival of every line, for the RX activity
        self.started = time.monotonic()
        self.recording = ""
        self.fx = Effects()
        self.painter = MazePainter()


FRAMED_ROWS, FRAMED_COLS = 20, 60   # panels get their boxes from this size on
RUN_ROWS, RUN_COLS = 10, 46
STATUS_ROWS, STATUS_COLS = 7, 40


def render(rows, cols, model, ov, view, start, now):
    """Composes one frame. Returns the screen and the cursor position."""
    scr = Screen(rows, cols)
    if rows < MIN_ROWS or cols < MIN_COLS:
        scr.put(0, 0, "Terminal too small", GOLD)
        scr.put(1, 0, "minimum %dx%d, now %dx%d" % (MIN_COLS, MIN_ROWS, cols, rows), DIM)
        return scr, (min(2, rows - 1), 0)

    alert = RED if now < view.fx.alert_until and int(now * 6) % 2 == 0 else None
    framed = rows >= FRAMED_ROWS and cols >= FRAMED_COLS
    fw, fh = (4, 2) if framed else (0, 0)   # the box and a column of air each side
    body_top, body_h = 1, rows - 3
    nx, ny = view_size(model, view.full)
    side_need = MIN_LOG_COLS + (4 if framed else 2)
    grain = GRAINS[view.pixels]
    inner_w = maze_panel_width(nx, ny, cols - side_need - fw, body_h - fh, grain)
    if inner_w is not None:     # side by side
        if framed:
            maze_box, side_box = (body_top, 0, body_h, inner_w + fw), (body_top, inner_w + fw, body_h, cols - inner_w - fw)
        else:
            sep_x = inner_w + 2
            maze_box, side_box = (body_top, 1, body_h, inner_w), (body_top, sep_x + 1, body_h, cols - sep_x - 1)
            for row in range(body_top, body_top + body_h):
                scr.put(row, sep_x, "│", mix(CYAN, BG, 0.6))
    else:                       # stacked
        widest = next((k for k in PITCHES if maze_shape(nx, ny, k, grain) and nx * k + 3 <= cols - fw), 4)
        maze_h = min(maze_shape(nx, ny, widest, grain)[1] + 2 + fh, max(3 + fh, body_h * 2 // 3))
        maze_box = (body_top, 0, maze_h, cols) if framed else (body_top, 1, maze_h, cols - 2)
        side_box = (body_top + maze_h, 0, body_h - maze_h, cols)

    top, left, height, width = maze_box
    if framed:
        draw_frame(scr, top, left, height, width, "0x01", "MAZE %dx%d" % (nx, ny), colour=alert)
        top, left, height, width = top + 1, left + 2, height - 2, width - 4
    if model.last_telemetry is None:
        draw_splash(scr, top, left, height, width, view, now)
    else:
        draw_maze(scr, top, left, height, width, model, ov, view, start, now)

    top, left, height, width = side_box
    panels = framed and inner_w is not None
    if panels and height >= RUN_ROWS + STATUS_ROWS + 6 and width >= RUN_COLS:
        draw_run(scr, top, left, RUN_ROWS, width, model, ov, view.fx, now, alert)
        top, height = top + RUN_ROWS, height - RUN_ROWS
    status = panels and height >= STATUS_ROWS + 6 and width >= STATUS_COLS
    if status:
        draw_frame(scr, top, left, STATUS_ROWS, width, "0x03", "STATUS")
        draw_status(scr, top + 1, left + 1, STATUS_ROWS - 2, width - 2, model, ov, view, now)
        top, height = top + STATUS_ROWS, height - STATUS_ROWS
    if framed and height >= 3:
        title, note = draw_console(scr, top + 1, left + 1, height - 2, width - 2, view)
        draw_frame(scr, top, left, height, width, "0x04", title, note)
    elif height >= 2:
        title, note = draw_console(scr, top + 1, left, height - 1, width, view)
        scr.put(top, left, " %s%s " % (title, note), MAGENTA, end=left + width)

    draw_header(scr, cols, model, ov, view, now, compact=not status)

    # The prompt's arrow shows the link, like a shell's last exit status.
    x = scr.put(rows - 2, 0, "❯ ", GREEN if view.connected else RED)
    room = max(0, cols - x - 1)
    x = scr.put(rows - 2, x, view.input[-room:] if room else "", WHITE)
    cursor = (rows - 2, min(cols - 1, x))
    if int(now * 2) % 2 == 0:
        scr.put(cursor[0], cursor[1], "█", TEXT)
    draw_keys(scr, rows - 1, cols, view)
    return scr, cursor


def draw_header(scr, cols, model, ov, view, now, compact):
    scr.bar(0, BAR)
    brand = " %s RATATRON " % MOUSE
    for i, c in enumerate(brand):
        scr.put(0, i, c, BLACK, ramp(BRAND, i / len(brand)))
    x = scr.put(0, len(brand), "▓▒░", MAGENTA)
    rx_age = now - view.rx_times[-1] if view.rx_times else INF
    parts = [("%s %s" % ("●" if view.connected else "○", view.source),
              RED if not view.connected else WHITE if rx_age < 0.15 else GREEN)]
    if view.capture:
        parts.append(("receiving %s: %d samples" % view.capture, CYAN))
    if model.mode:
        parts.append(("%d %s" % (model.mode, MODE_NAME[model.mode]), CYAN))
    if model.activity:
        spin = SPIN[int(now * 10) % 4] + " " if model.activity in RUNNING else ""
        parts.append((spin + ACTIVITY_NAME[model.activity], TEXT))
    if compact:     # no status panel: its facts go here
        if model.pose:
            parts.append(("(%d,%d)%s" % (model.pose[0], model.pose[1], HEADINGS[model.pose[2]]), TEXT))
        parts.append(("%d cells" % model.visited_count(), TEXT))
        if ov.fast_cost is not None:
            parts.append(("fast: %d cells %d turns" % (len(ov.fast_path), ov.fast_turns), TEXT))
        parts.append((telemetry_age(model, now), TEXT))
    for i, (text, colour) in enumerate(parts):
        x = scr.put(0, x, " │ " if i else " ", FAINT)
        x = scr.put(0, x, text, colour)
    up = int(now - view.started)
    clock = " T+%02d:%02d:%02d " % (up // 3600, up // 60 % 60, up % 60)
    if x + len(clock) + 1 <= cols:
        scr.put(0, cols - len(clock), clock, CYAN)


def draw_run(scr, top, left, height, width, model, ov, fx, now, alert):
    """The run's phases, its clock in large digits and the speed runs' times."""
    draw_frame(scr, top, left, height, width, "0x02", "RUN", colour=alert)
    end = left + width - 2
    act = model.activity or "I"
    current = "GOHIFR".index(act) if act in "GOHFR" or (act == "I" and ov.fast_cost is not None) else -1
    x = left + 2
    for i, name in enumerate(("SEARCH", "OPTIM", "RETURN", "READY", "RACE", "BACK")):
        if i:
            x = scr.put(top + 1, x, " ━ " if width >= 64 else "━", GREEN if i <= current else FAINT, end=end)
        if i == current:
            x = scr.put(top + 1, x, " %s " % name, BLACK, mix(CYAN, WHITE, 0.5 + 0.5 * math.sin(now * 6)), end=end)
        else:
            x = scr.put(top + 1, x, name, mix(GREEN, BG, 0.35) if i < current else DIM, end=end)
    if act in "CWKE":
        scr.put(top + 1, x + 2, ACTIVITY_NAME[act], GOLD, end=end)

    value, state = fx.timer(now)
    best = min(fx.laps) if fx.laps else None
    text = "0.00" if value is None else "%.2f" % value if value < 100 else "%d:%04.1f" % (value // 60, value % 60)
    colours = {"idle": (FAINT, FAINT), "run": (CYAN, VIOLET), "lap": (WHITE, GOLD),
               "done": (GREEN, mix(GREEN, BG, 0.5))}[state]
    x = big_text(scr, top + 3, left + 3, text, colours[0], colours[1], end)
    scr.put(top + 7, x, "s", colours[1], end=end)
    label = {"idle": "no run yet", "run": ACTIVITY_NAME.get(act, ""), "done": "run finished",
             "lap": "speed run: best" if value == best and len(fx.laps) > 1 else "speed run to the goal"}[state]
    facts = (("BEST", "%.2f s" % best if best else "--", GOLD if best else FAINT),
             ("LAST", "%.2f s" % fx.laps[-1] if best else "--", TEXT if best else FAINT),
             ("RUNS", "%d" % len(fx.laps), TEXT if best else FAINT))
    x = max(x + 4, left + 39)
    if x + 14 <= end:       # room beside the clock
        scr.put(top + 3, x, label.upper(), colours[0], end=end)
        for i, (name, fact, colour) in enumerate(facts):
            scr.put(top + 5 + i, x, name, DIM, end=end)
            scr.put(top + 5 + i, x + 6, fact, colour, end=end)
        if fx.laps:         # one bar per speed run, the best in gold
            x = scr.put(top + 8, left + 3, "LAPS  ", DIM, end=end)
            for lap in fx.laps[-max(1, end - x):]:
                x = scr.put(top + 8, x, SPARK[max(1, int(8 * lap / max(fx.laps)))], GOLD if lap == best else CYAN, end=end)
    else:
        x = left + 3
        for name, fact, colour in facts:
            x = scr.put(top + 8, x, name + " ", DIM, end=end)
            x = scr.put(top + 8, x, fact + "  ", colour, end=end)


def draw_status(scr, top, left, height, width, model, ov, view, now):
    """Pose, explored share, fast path, link and RX activity."""
    end = left + width

    def line(row, label, segments):
        x = scr.put(top + row, left + 1, label.ljust(6), DIM, end=end)
        for text, colour in segments:
            x = scr.put(top + row, x, text, colour, end=end)

    x0, y0, x1, y1 = model.goal
    goal = "goal (%d,%d)" % (x0, y0) if (x0, y0) == (x1, y1) else "goal (%d,%d)-(%d,%d)" % model.goal
    pose = ("(%d,%d) %s" % (model.pose[0], model.pose[1], HEADINGS[model.pose[2]]), WHITE) if model.pose else ("--", FAINT)
    line(0, "POSE", [pose, ("   " + goal, MAGENTA)])

    nx, ny = view_size(model, view.full)
    explored = sum(model.visited[x][y] for x in range(nx) for y in range(ny))
    bar_w = max(4, min(48, width - 1 - 6 - 9))
    filled = bar_w * explored // (nx * ny)
    line(1, "MAP", [("▰", ramp((CYAN, VIOLET, MAGENTA), i / bar_w)) if i < filled else ("▱", FAINT) for i in range(bar_w)]
         + [(" %d/%d" % (explored, nx * ny), WHITE)])

    if ov.fast_cost is None:
        line(2, "FAST", [("no verified path yet", FAINT)])
    else:
        line(2, "FAST", [("%d cells" % len(ov.fast_path), CYAN), ("  %d turns" % ov.fast_turns, TEXT),
                         ("  cost %d" % ov.fast_cost, DIM)])

    state = ("● up", GREEN) if view.connected else ("○ %s" % (view.link_text or "down"), RED)
    line(3, "LINK", [state, ("  " + telemetry_age(model, now), DIM),
                     ("  bad %d" % model.bad_lines, RED if model.bad_lines else DIM)])

    span = max(4, min(60, width - 1 - 6 - 9))    # one column per second
    counts = [0] * span
    for t in view.rx_times:
        age = int(now - t)
        if 0 <= age < span:
            counts[span - 1 - age] += 1
    peak = max(counts) or 1
    rate = sum(1 for t in view.rx_times if now - t < 5) / 5
    line(4, "RX", [(SPARK[0] if n == 0 else SPARK[1 + (n * (len(SPARK) - 2) + peak - 1) // peak],
                    ramp((mix(GREEN, BG, 0.6), GREEN), i / span)) for i, n in enumerate(counts)]
         + [(" %5.1f/s" % rate, WHITE)])


def draw_console(scr, top, left, height, width, view):
    """The log (or the help) inside the box; returns its title and note."""
    if height < 1 or width < 2:
        return "CONSOLE", ""
    end = left + width
    if view.show_help:
        for i, (style, text) in enumerate(HELP_LINES[:height]):
            if style == "keyline":     # the keys, then what they do in a column
                key, _, rest = text.strip().partition("  ")
                x = scr.put(top + i, left + 3, key, CYAN, end=end)
                scr.put(top + i, max(x + 2, left + 16), rest.strip(), TEXT, end=end)
            elif style == "head":
                scr.put(top + i, left, "▌" + text, MAGENTA, end=end)
            else:
                scr.put(top + i, left + 1, text, TEXT, end=end)
        return "HELP", "  Tab: back to the log"
    lines = list(view.log)
    view.scroll = max(0, min(view.scroll, max(0, len(lines) - height)))
    first = max(0, len(lines) - height - view.scroll)
    for i, (style, text) in enumerate(lines[first:first + height]):
        y, colour = top + i, LOG_COLOUR.get(style, TEXT)
        if style not in ("", "dim", "boot"):
            scr.put(y, left, "▌", colour)
        if style == "boot" and text.startswith("["):
            scr.put(y, left + 1, text[:6], GREEN if "OK" in text[:6] else GOLD, end=end)
            scr.put(y, left + 7, text[6:], DIM, end=end)
        else:
            scr.put(y, left + 1, text, colour, end=end)
    note = "  ↓ %d more lines below" % view.scroll if view.scroll else ""
    return "CONSOLE", note


def draw_keys(scr, row, cols, view):
    """htop-style key bar, the recording light on the right."""
    scr.bar(row, BAR)
    x = 0
    for key, label in (("Enter", "send"), ("^X", "STOP"), ("Tab", "help"), ("Esc", "quit"),
                       ("↑↓", "history"), ("PgUp/Dn", "log")):
        if x + len(key) + len(label) + 3 > cols - 1:
            break
        x = scr.put(row, x, " %s " % key, WHITE if key == "^X" else BLACK, (200, 30, 40) if key == "^X" else mix(CYAN, BG, 0.3))
        x = scr.put(row, x, " %s  " % label, TEXT)
    if view.recording:
        rec = "● REC %s " % view.recording
        if x + len(rec) + 1 < cols:
            scr.put(row, cols - 1 - len(rec), rec, RED)


# ---- Links: serial port, replay file, session recorder -------------------------------------

def friendly_error(exc):
    text = str(exc)
    low = text.lower()
    if "exclusively lock" in low or "resource temporarily unavailable" in low:
        return "port held by another program (another monitor open?)"
    if "no such file" in low:
        return "no such port (rfcomm bind? see tools/rfcomm_reconnect.sh)"
    if "permission denied" in low:
        return "no permission on the port (dialout group)"
    if "host is down" in low or "connection refused" in low or "no route" in low:
        return "robot off or out of range"
    if "readiness to read but returned no data" in low:
        return "Bluetooth link down"
    return text


class SerialLink(threading.Thread):
    def __init__(self, port, baud, on_line, on_status, on_connect):
        super().__init__(daemon=True)
        self.port, self.baud = port, baud
        self.on_line, self.on_status, self.on_connect = on_line, on_status, on_connect
        self.stop_event = threading.Event()
        self.write_lock = threading.Lock()
        self.ser = None

    def run(self):
        while not self.stop_event.is_set():
            try:
                ser = serial.Serial(self.port, self.baud, timeout=0.1, exclusive=True)
            except (serial.SerialException, OSError, ValueError) as exc:
                self.on_status(False, friendly_error(exc))
                self.stop_event.wait(1.0)
                continue
            with self.write_lock:
                self.ser = ser
            self.on_status(True, "")
            self.on_connect()
            pending = b""
            try:
                while not self.stop_event.is_set():
                    chunk = ser.read(ser.in_waiting or 1)
                    if not chunk:
                        continue
                    pending += chunk
                    *lines, pending = pending.split(b"\n")
                    for raw in lines:
                        self.on_line(raw.decode("ascii", "replace").rstrip("\r"))
                    if len(pending) > 4096:     # noise without line ends
                        pending = b""
            except (serial.SerialException, OSError) as exc:
                self.on_status(False, friendly_error(exc))
            finally:
                with self.write_lock:
                    self.ser = None
                try:
                    ser.close()
                except (serial.SerialException, OSError):
                    pass
            self.stop_event.wait(1.0)

    def send(self, text):
        with self.write_lock:
            if self.ser is None:
                return False
            try:
                self.ser.write((text + "\n").encode("ascii", "replace"))
                return True
            except (serial.SerialException, OSError):
                return False

    def stop(self):
        self.stop_event.set()


class ReplayLink(threading.Thread):
    """Plays back a recorded session, or a plain transcript at 20 lines/s."""

    def __init__(self, path, speed, on_line, on_status, on_sent):
        super().__init__(daemon=True)
        self.path, self.speed = path, max(0.01, speed)
        self.on_line, self.on_status, self.on_sent = on_line, on_status, on_sent
        self.stop_event = threading.Event()

    def run(self):
        name = os.path.basename(self.path)
        try:
            with open(self.path, encoding="utf-8", errors="replace") as f:
                records = [line.rstrip("\n") for line in f]
        except OSError as exc:
            self.on_status(False, "cannot read %s: %s" % (name, exc))
            return
        self.on_status(True, "replaying " + name)
        previous = None
        for i, record in enumerate(records):
            fields = record.split("\t", 2)
            if len(fields) == 3 and re.match(r"^\d+(\.\d+)?$", fields[0]):
                t, direction, text = float(fields[0]), fields[1], fields[2]
            else:
                t, direction, text = i * 0.05, "<", record
            if previous is not None and self.stop_event.wait(min(2.0, max(0.0, t - previous) / self.speed)):
                return
            previous = t
            if direction == "<":
                self.on_line(text, t)
            elif direction == ">":
                self.on_sent(text)
            elif direction == "!":
                self.on_sent("-- %s --" % text)
        self.on_status(False, "end of the replay of " + name)

    def send(self, text):
        return False

    def stop(self):
        self.stop_event.set()


class Recorder:
    def __init__(self, path):
        os.makedirs(os.path.dirname(path), exist_ok=True)
        self.path = path
        self.file = open(path, "a", encoding="utf-8", buffering=1)
        self.t0 = time.monotonic()
        self.lock = threading.Lock()

    def write(self, direction, text):
        with self.lock:
            if self.file:
                self.file.write("%.3f\t%s\t%s\n" % (time.monotonic() - self.t0, direction, text))

    def close(self):
        with self.lock:
            if self.file:
                self.file.close()
                self.file = None


# ---- Application ---------------------------------------------------------------------------

class Monitor:
    def __init__(self, args, config):
        self.lock = threading.RLock()
        self.model = MazeModel()
        self.planner = Planner(config)
        self.start = (config["START_X"], config["START_Y"])
        self.view = ViewState()
        self.view.full, self.view.pixels = args.full, args.pixels
        self.view.replay = bool(args.replay)
        self.view.fx.rate = args.speed if args.replay else 1.0
        self.view.fx.lively = args.effects
        self.view.source = os.path.basename(args.replay) if args.replay else args.port
        self.capture = CalibrationCapture(args.calib_dir, save=not args.replay)
        self.history = []
        self.history_pos = None
        self.dirty = True
        self.recorder = None
        if not args.replay and not args.no_record:
            stamp = time.strftime("%Y-%m-%d_%H-%M-%S")
            self.recorder = Recorder(os.path.join(args.log_dir, stamp + ".log"))
        if args.replay:
            self.link = ReplayLink(args.replay, args.speed, self.on_line, self.on_status, self.on_replay_sent)
        else:
            self.link = SerialLink(args.port, args.baud, self.on_line, self.on_status, self.on_connect)
        self.add_log("head", "RATATRON//MONITOR  micromouse telemetry console")
        self.add_log("boot", "[ OK ] planner costs: search %d/cell %d/turn, fast %d/cell %d/turn"
                     % (self.planner.search + self.planner.fast))
        if self.recorder:
            self.view.recording = os.path.basename(self.recorder.path)
            self.add_log("boot", "[ OK ] recording to %s" % os.path.relpath(self.recorder.path))
        if args.replay:
            self.add_log("boot", "[ OK ] replay of %s at x%g" % (os.path.basename(args.replay), args.speed))
        else:
            self.add_log("boot", "[ .. ] link %s at %d baud" % (args.port, args.baud))

    # -- called from the link thread
    def on_line(self, text, stamp=None):
        """A line from the robot; `stamp` is its time in a replayed session."""
        if self.recorder:
            self.recorder.write("<", text)
        now = time.monotonic()
        with self.lock:
            self.view.rx_times.append(now)
            if text.startswith("@D"):
                message = self.capture.feed(text)
                if message:
                    self.add_log(*message)
                self.view.capture = self.capture.progress()
            elif not self.view.fx.line(self.model, self.planner, text, now if stamp is None else stamp, now) \
                    and text.strip():
                self.add_log(log_style(text), sanitize(text))
            self.dirty = True

    def on_status(self, connected, message):
        if self.recorder:   # link events in the recording help diagnose drops later
            self.recorder.write("!", message or ("connected" if connected else "disconnected"))
        with self.lock:
            changed = connected != self.view.connected or message != self.view.link_text
            self.view.connected, self.view.link_text = connected, message
            if changed and message:
                self.add_log("info" if connected else "bad", "-- " + message + " --")
            elif changed and connected:
                self.add_log("info", "-- connected to %s --" % self.view.source)
            self.dirty = True

    def on_connect(self):
        self.send("SYNC", quiet=True)   # ask for the map; refused harmlessly mid-run

    def on_replay_sent(self, text):
        with self.lock:
            if text.startswith("-- "):      # recorded link event
                self.add_log("info", sanitize(text))
            else:
                self.add_log("sent", "> " + sanitize(text))
            self.dirty = True

    # -- UI thread
    def add_log(self, style, text):
        self.view.log.append((style, text))
        if self.view.scroll:
            self.view.scroll += 1   # keep a scrolled-back view still while lines arrive

    def send(self, command, quiet=False):
        ok = self.link.send(command)
        if self.recorder and ok:
            self.recorder.write(">", command)
        if quiet:
            return
        with self.lock:
            if ok:
                self.add_log("sent", "> " + command)
            elif self.view.replay:
                self.add_log("dim", "> %s (replay: not sent)" % command)
            else:
                self.add_log("bad", "> %s (no connection: not sent)" % command)
            self.dirty = True

    def local_command(self, command):
        name, _, rest = command.partition(" ")
        name = name.lower()
        with self.lock:
            if name == "/full":
                self.view.full = not self.view.full
            elif name == "/pixels":
                names = list(GRAINS)
                self.view.pixels = names[(names.index(self.view.pixels) + 1) % len(names)]
                self.add_log("info", "maze drawn in %s pixels" % self.view.pixels)
            elif name == "/effects":
                self.view.fx.lively = not self.view.fx.lively
                self.add_log("info", "effects during the runs %s" % ("on" if self.view.fx.lively else "off"))
            elif name == "/clear":
                self.view.log.clear()
                self.view.scroll = 0
            elif name == "/note" and rest.strip():
                try:
                    self.add_log(*self.capture.note(sanitize(rest.strip())))
                except OSError as exc:
                    self.add_log("bad", "could not write the note: %s" % exc)
            else:
                self.add_log("warn", "local commands: /full /pixels /effects /clear /note <text>")
            self.dirty = True

    def submit(self):
        command = self.view.input.strip()
        self.view.input = ""
        self.history_pos = None
        if not command:
            return
        if not self.history or self.history[-1] != command:
            self.history.append(command)
            del self.history[:-100]
        if command.startswith("/"):
            self.local_command(command)
        else:
            self.send(command)

    def handle_key(self, key):
        """False to quit."""
        with self.lock:
            self.dirty = True
            view = self.view
            if key == "esc":
                return False
            if key == "enter":
                self.submit()
            elif key == "backspace":
                view.input = view.input[:-1]
                self.history_pos = None
            elif key == "^U":
                view.input = ""
            elif key == "^X":
                self.send("STOP")
            elif key == "tab":
                view.show_help = not view.show_help
            elif key == "up" and self.history:
                self.history_pos = len(self.history) - 1 if self.history_pos is None else max(0, self.history_pos - 1)
                view.input = self.history[self.history_pos]
            elif key == "down" and self.history_pos is not None:
                self.history_pos += 1
                if self.history_pos >= len(self.history):
                    self.history_pos, view.input = None, ""
                else:
                    view.input = self.history[self.history_pos]
            elif key == "pgup":
                view.scroll += 10
            elif key == "pgdn":
                view.scroll = max(0, view.scroll - 10)
            elif key == "home":
                view.scroll = len(view.log)
            elif key == "end":
                view.scroll = 0
            elif len(key) == 1 and len(view.input) < 60:
                view.input += key
                self.history_pos = None
        return True

    def frame(self, rows, cols):
        with self.lock:
            now = time.monotonic()
            self.view.fx.advance(now)
            ov = self.planner.overlay(self.model)
            return render(rows, cols, self.model, ov, self.view, self.start, now)

    def run(self, terminal):
        self.link.start()
        last_draw = 0.0
        try:
            while True:
                for key in terminal.keys(0.02):
                    if not self.handle_key(key):
                        return
                    if key == "^L":
                        terminal.shown = None
                now = time.monotonic()
                with self.lock:
                    # 20 frames/s while something moves (the robot, a banner, the splash), else 10.
                    rx = self.view.rx_times
                    moving = self.model.last_telemetry is None or self.model.activity in RUNNING \
                        or self.view.fx.busy(now) or (rx and now - rx[-1] < 0.5)
                    redraw = now - last_draw >= (0.05 if moving else 0.1) or (self.dirty and now - last_draw >= 0.02)
                    if redraw:
                        self.dirty = False
                if redraw:
                    size = shutil.get_terminal_size()
                    terminal.show(self.frame(size.lines, size.columns)[0])
                    last_draw = now
        except KeyboardInterrupt:
            pass
        finally:
            self.link.stop()
            if self.recorder:
                self.recorder.close()


# ---- Terminal: raw keys in, escape codes out ----------------------------------------------

KEY_NAMES = {b"[A": "up", b"[B": "down", b"OA": "up", b"OB": "down", b"[5~": "pgup", b"[6~": "pgdn",
             b"[H": "home", b"[F": "end", b"OH": "home", b"OF": "end", b"[1~": "home", b"[4~": "end"}
KEY_BYTES = {10: "enter", 13: "enter", 127: "backspace", 8: "backspace", 9: "tab", 21: "^U", 24: "^X", 12: "^L"}
KEY_SEQUENCE = re.compile(rb"\x1b(\[[0-9;]*[A-Za-z~]|O[A-Za-z])")


def parse_keys(data):
    """Key names of the bytes a terminal sent: printable characters as themselves."""
    keys, i = [], 0
    while i < len(data):
        sequence = KEY_SEQUENCE.match(data, i)
        if sequence:
            keys.append(KEY_NAMES.get(sequence.group(1)))
            i = sequence.end()
            continue
        keys.append("esc" if data[i] == 27 else KEY_BYTES.get(data[i]) or (chr(data[i]) if 32 <= data[i] < 127 else None))
        i += 1
    return [k for k in keys if k]


def colour_256(rgb):
    """The nearest xterm 256-colour index: its 6x6x6 cube or its grey ramp."""
    r, g, b = rgb
    if max(rgb) - min(rgb) < 12:
        return 16 if r < 5 else 231 if r > 243 else 232 + (r - 3) // 10
    return 16 + 36 * ((r + 25) // 51) + 6 * ((g + 25) // 51) + (b + 25) // 51


class Terminal:
    """The alternate screen in 24-bit colour (the nearest of 256 where COLORTERM does not
    announce it). Only the characters that changed since the last frame are sent."""

    def __init__(self):
        self.fd = sys.stdin.fileno()
        self.truecolour = os.environ.get("COLORTERM", "").lower() in ("truecolor", "24bit")
        self.shown = None       # the screen on the terminal; None repaints everything
        self.codes = {}

    def __enter__(self):
        self.saved = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        mode = termios.tcgetattr(self.fd)
        mode[0] &= ~termios.IXON    # Ctrl+S must not freeze the screen
        termios.tcsetattr(self.fd, termios.TCSADRAIN, mode)
        self.write("\x1b[?1049h\x1b[?25l\x1b[?7l")   # alternate screen, no cursor, no wrapping
        return self

    def __exit__(self, *exc):
        self.write("\x1b[0m\x1b[?7h\x1b[?25h\x1b[?1049l")
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.saved)

    @staticmethod
    def write(text):
        data = text.encode("utf-8")
        while data:
            data = data[os.write(sys.stdout.fileno(), data):]

    def keys(self, timeout):
        """Names of the keys pressed within `timeout` seconds."""
        if not select.select([self.fd], [], [], timeout)[0]:
            return []
        data = os.read(self.fd, 256)
        if not data:
            return ["esc"]      # the terminal is gone
        if data.endswith(b"\x1b") and select.select([self.fd], [], [], 0.05)[0]:
            data += os.read(self.fd, 256)   # the rest of a key's sequence, not Esc alone
        return parse_keys(data)

    def code(self, layer, rgb):
        key = (layer, rgb)
        if key not in self.codes:
            self.codes[key] = "\x1b[%d;2;%d;%d;%dm" % ((layer,) + rgb) if self.truecolour \
                else "\x1b[%d;5;%dm" % (layer, colour_256(rgb))
        return self.codes[key]

    def show(self, scr):
        old = self.shown
        if old is None or (old.rows, old.cols) != (scr.rows, scr.cols):
            old = None
        out, fg, bg = [] if old else ["\x1b[0m\x1b[2J"], None, None
        for y in range(scr.rows):
            ch, fgs, bgs = scr.ch[y], scr.fg[y], scr.bg[y]
            if old and ch == old.ch[y] and fgs == old.fg[y] and bgs == old.bg[y]:
                continue
            at = -1
            for x in range(scr.cols):
                if old and ch[x] == old.ch[y][x] and fgs[x] == old.fg[y][x] and bgs[x] == old.bg[y][x]:
                    continue
                if x != at:
                    out.append("\x1b[%d;%dH" % (y + 1, x + 1))
                if fgs[x] != fg:
                    fg = fgs[x]
                    out.append(self.code(38, fg))
                if bgs[x] != bg:
                    bg = bgs[x]
                    out.append(self.code(48, bg))
                out.append(ch[x])
                at = x + 1
        self.shown = scr
        if out:
            self.write("".join(out))


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", default="/dev/rfcomm0", help="serial port (default /dev/rfcomm0)")
    parser.add_argument("--baud", type=int, default=9600)
    parser.add_argument("--replay", metavar="FILE", help="replay a recorded session (or a transcript)")
    parser.add_argument("--speed", type=float, default=1.0, help="replay speed (default 1)")
    parser.add_argument("--no-record", action="store_true", help="do not record the session")
    parser.add_argument("--log-dir", default=LOG_DIR, help="folder of recorded sessions")
    parser.add_argument("--calib-dir", default=CALIB_DIR, help="folder of calibration data")
    parser.add_argument("--full", action="store_true", help="always draw the whole 16x16 maze")
    parser.add_argument("--pixels", choices=list(GRAINS), default="sextant",
                        help="the maze's grain: half where the terminal does not draw sextants (also /pixels)")
    parser.add_argument("--effects", action="store_true",
                        help="the map moves during a run too: a sweep over what is unseen, a ripple for every "
                             "new cell, the route flowing, the goal pulsing (also /effects)")
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    if not args.replay and serial is None:
        print("pyserial is missing: pip install pyserial (or use --replay)", file=sys.stderr)
        return 1
    monitor = Monitor(args, read_config())
    with Terminal() as terminal:
        monitor.run(terminal)
    if monitor.recorder:
        print("Session recorded in", os.path.relpath(monitor.recorder.path))
    return 0


if __name__ == "__main__":
    sys.exit(main())
