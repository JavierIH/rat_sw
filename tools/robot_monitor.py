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
"""
import argparse
import collections
import curses
import heapq
import locale
import os
import re
import sys
import threading
import time

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
            if not route:   # no verified route: the robot explores step by step
                cost = self.plan_to(model, targets, False, self.search)
                route = self.follow(model, cost, *pose, False, self.search)[0]
        self._cache_key = key
        self._cache = Overlay(fast_cost if fast_cost < INF else None, fast_path, fast_turns, route, cands)
        return self._cache


def route_text(turns, limit=ROUTE_TEXT_MAX):
    """The route as the firmware logs it: cells straight ahead, then D/I for a
    curve right/left in the last of them ("2D1I3"), cut with '+'."""
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


# ---- Rendering into an off-screen canvas ------------------------------------------------

GLYPHS_UNICODE = {
    "robot": "▲▶▼◀", "path": "•", "cand": "◇", "visited": "·", "post": "·",
    "h_wall": "─", "v_wall": "│", "h_unknown": "┄", "v_unknown": "┆",
    "junction": " ╶╷┌╴─┐┬╵└│├┘┴┤┼", "dot_on": "●", "dot_off": "○", "sep": "│",
    "down": "↓", "h_trace": "─", "v_trace": "│", "frame": "╭╮╰╯─│", "title": ("┤ ", " ├"),
    "spin": "◐◓◑◒", "bar": "▰▱", "spark": " ▁▂▃▄▅▆▇█", "gutter": "▌", "fade": "▓▒░",
    "mouse": "~(__^·>", "updown": "↑↓", "cursor": "█", "prompt": "❯",
}
GLYPHS_ASCII = {
    "robot": "^>v<", "path": "*", "cand": "o", "visited": ".", "post": "+",
    "h_wall": "-", "v_wall": "|", "h_unknown": ".", "v_unknown": ":",
    "junction": "++++++++++++++++", "dot_on": "*", "dot_off": "o", "sep": "|",
    "down": "v", "h_trace": "-", "v_trace": "|", "frame": "++++-|", "title": ("[ ", " ]"),
    "spin": "|/-\\", "bar": "#.", "spark": " .:-=+*#", "gutter": "|", "fade": "",
    "mouse": "~(__^.>", "updown": "Up/Dn", "cursor": "_", "prompt": ">",
}
LOGOS_UNICODE = ((     # the largest that fits is drawn
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
LOGOS_ASCII = ((
    " _   _  ___  _  ___  _   _      ",
    "|_) |_|  |  |_|  |  |_) | | |\\ |",
    "| \\ | |  |  | |  |  | \\ |_| | \\|",
),)

# Style -> (foreground, background, bold) in the xterm 256-colour palette, on a background of
# its own; init_styles() falls back to the nearest of the 8 basic colours.
BG, BAR, KEYBAR = 233, 236, 235
THEME = {
    "": (252, BG, False), "dim": (243, BG, False), "label": (244, BG, False), "value": (255, BG, True),
    "ok": (46, BG, True), "bad": (196, BG, True), "warn": (214, BG, False), "info": (45, BG, False),
    "head": (201, BG, True), "sent": (226, BG, False), "key": (51, BG, True), "boot": (243, BG, False),
    "wall": (255, BG, True), "unknown": (60, BG, False), "axis": (239, BG, False), "axis_hot": (226, BG, True),
    "robot": (226, 58, True), "halo": (226, 58, False), "goal": (201, 53, True), "goal_bg": (201, 53, False),
    "route": (46, BG, True), "route_trace": (34, BG, False), "fast": (51, BG, True), "fast_trace": (37, BG, False),
    "cand": (213, BG, True), "trail": (220, BG, True), "trail1": (178, BG, False), "trail2": (136, BG, False),
    "trail3": (94, BG, False), "border": (31, BG, False), "frame": (31, BG, False),
    "frame_title": (51, BG, True), "frame_tag": (201, BG, True),
    "title": (250, BAR, False), "title_sep": (239, BAR, False), "title_ok": (46, BAR, True),
    "title_bad": (196, BAR, True), "title_hot": (51, BAR, True), "brand_fade": (165, BAR, False),
    "keycap": (16, 37, True), "keycap_stop": (231, 160, True), "keylabel": (250, KEYBAR, False),
    "rec": (196, KEYBAR, True),
    "bar_on": (46, BG, False), "bar_off": (237, BG, False), "spark": (41, BG, False),
    "logo_shadow": (60, BG, False), "rain": (22, BG, False), "rain_head": (120, BG, True),
}
BRAND_COLOURS = (51, 45, 39, 63, 99, 135, 165)     # header block, cyan to violet
THEME.update({"brand%d" % i: (16, c, True) for i, c in enumerate(BRAND_COLOURS)})
THEME.update({"logo%d" % i: (c, BG, True) for i, c in enumerate((51, 45, 39, 63, 135, 201))})


def sanitize(text):
    """Printable ASCII only: control characters would break the screen layout."""
    return "".join(c if 32 <= ord(c) < 127 else "?" for c in text)


class Canvas:
    """A rows x cols grid of (char, style). Writes are clipped, never raise."""

    def __init__(self, rows, cols):
        self.rows, self.cols = rows, cols
        self.chars = [[" "] * cols for _ in range(rows)]
        self.styles = [[""] * cols for _ in range(rows)]

    def put(self, y, x, text, style="", width=None):
        if not 0 <= y < self.rows:
            return
        end = self.cols if width is None else min(self.cols, x + max(0, width))
        for ch in text:
            if x >= end:
                break
            if x >= 0:
                self.chars[y][x] = ch
                self.styles[y][x] = style
            x += 1

    def fill(self, y, x, height, width, ch=" ", style=""):
        for row in range(y, y + height):
            self.put(row, x, ch * width, style, width)

    def row_text(self, y):
        return "".join(self.chars[y])


def put_segments(cv, y, x, width, segments):
    """Writes (text, style) pieces one after another, clipped to width. Returns the end column."""
    end = x + max(0, width)
    for text, style in segments:
        if x >= end:
            break
        cv.put(y, x, text, style, end - x)
        x += len(text)
    return min(x, end)


def draw_frame(cv, top, left, height, width, glyphs, tag, title, note=""):
    """A box with its title in the top border: ╭─┤ 0x01 MAZE ├───╮."""
    tl, tr, bl, br, h, v = glyphs["frame"]
    cv.put(top, left, tl + h * (width - 2) + tr, "frame")
    for row in range(top + 1, top + height - 1):
        cv.put(row, left, v, "frame")
        cv.put(row, left + width - 1, v, "frame")
    cv.put(top + height - 1, left, bl + h * (width - 2) + br, "frame")
    opening, closing = glyphs["title"]
    put_segments(cv, top, left + 2, width - 4, [(opening, "frame"), (tag + " ", "frame_tag"),
                                                (title, "frame_title"), (note, "dim"), (closing, "frame")])


def spinner(glyphs, now):
    return glyphs["spin"][int(now * 10) % len(glyphs["spin"])]


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


CELL_SIZES = ((7, 3), (5, 2), (3, 1), (2, 1), (1, 1))  # characters inside a cell, across and down: square on screen
OPEN = (OPEN_ONCE, OPEN_VERIFIED)


def maze_width(nx, cw):
    return nx * (cw + 1) + 1


def maze_height(ny, ch):
    return ny * (ch + 1) + 1


def maze_fit(nx, ny, width, height):
    """(cell width, cell height, axes): the largest cells that fit with the legend row, with
    the hex axes if they fit too; else the widest that fit across, the view scrolled."""
    for cw, ch in CELL_SIZES:
        for axes in (1, 0):
            if maze_width(nx, cw) + 2 * axes <= width and maze_height(ny, ch) + 1 + axes <= height:
                return cw, ch, axes
    return next((w for w in (3, 2, 1) if maze_width(nx, w) <= width), 1), 1, 0


def maze_panel_width(nx, ny, width, height):
    """Width of the whole maze at the largest cells fitting (width, height), or None."""
    for cw, ch in CELL_SIZES:
        for axes in (1, 0):
            if maze_width(nx, cw) + 2 * axes <= width and maze_height(ny, ch) + 1 + axes <= height:
                return maze_width(nx, cw) + 2 * axes
    return None


def trail_style(age):
    """The run's trail fades behind the robot, like phosphor."""
    return "trail" if age < 4 else "trail1" if age < 10 else "trail2" if age < 20 else "trail3"


def draw_maze(cv, top, left, height, width, model, ov, glyphs, full, start):
    """Draws the maze centred in the box (top, left, height, width)."""
    nx, ny = view_size(model, full)
    cw, ch, axes = maze_fit(nx, ny, width, height)
    pw, ph = cw + 1, ch + 1     # cell pitch
    vnx = min(nx, max(1, (width - 1 - 2 * axes) // pw))
    vny = min(ny, max(1, (height - 2 - axes) // ph))
    rx, ry = (model.pose[0], model.pose[1]) if model.pose else (0, 0)
    vx0 = min(max(0, rx - vnx // 2), nx - vnx)
    vy0 = min(max(0, ry - vny // 2), ny - vny)
    gx = left + 2 * axes + max(0, (width - maze_width(vnx, cw) - 2 * axes) // 2)
    gy = top + max(0, (height - maze_height(vny, ch) - 1 - axes) // 2)
    running = model.activity in RUNNING
    route = set(ov.route)
    fast = set() if running else set(ov.fast_path)
    trail_age = {cell: len(model.trail) - 1 - i for i, cell in enumerate(model.trail)}

    def line_row(py):   # canvas row of the horizontal grid line py
        return gy + (vy0 + vny - py) * ph

    def line_col(px):   # canvas column of the vertical grid line px
        return gx + (px - vx0) * pw

    def center(x, y):
        return line_row(y + 1) + 1 + (ch - 1) // 2, line_col(x) + 1 + (cw - 1) // 2

    def inside(x, y):
        return vx0 <= x < vx0 + vnx and vy0 <= y < vy0 + vny

    def vline(px, y):   # vertical wall on grid column px, cell row y
        if px <= 0 or px >= MAZE:
            return WALL
        return model.east[px - 1][y]

    def hline(x, py):   # horizontal wall on grid row py, cell column x
        if py <= 0 or py >= MAZE:
            return WALL
        return model.north[x][py - 1]

    def post(px, py):
        up = py < vy0 + vny and vline(px, py) == WALL
        down = py > vy0 and vline(px, py - 1) == WALL
        lft = px > vx0 and hline(px - 1, py) == WALL
        rgt = px < vx0 + vnx and hline(px, py) == WALL
        index = up * 8 + lft * 4 + down * 2 + rgt
        if index == 0:
            return glyphs["post"], "unknown"
        return glyphs["junction"][index], "wall"

    def content(x, y):
        # Route markers win over the goal letter, in the goal colour.
        goal = model.is_goal(x, y)
        if model.pose and (x, y) == model.pose[:2]:
            return glyphs["robot"][model.pose[2]], "robot"
        if (x, y) in ov.candidates:
            return glyphs["cand"], "goal" if goal else "cand"
        if (x, y) in route:
            return glyphs["path"], "goal" if goal else "route"
        if (x, y) in fast:
            return glyphs["path"], "goal" if goal else "fast"
        if goal:
            return "G", "goal"
        if (x, y) == start:
            return "S", "dim"
        if model.visited[x][y]:
            return glyphs["visited"], trail_style(trail_age[(x, y)]) if (x, y) in trail_age else "dim"
        return " ", ""

    # Walls and posts.
    for py in range(vy0, vy0 + vny + 1):
        row = line_row(py)
        for px in range(vx0, vx0 + vnx + 1):
            col = line_col(px)
            cv.put(row, col, *post(px, py))
            if px < vx0 + vnx:
                w = hline(px, py)
                if w == WALL:
                    cv.put(row, col + 1, glyphs["h_wall"] * cw, "wall")
                elif w == UNKNOWN:
                    cv.put(row, col + 1, glyphs["h_unknown"] * cw, "unknown")
            if py < vy0 + vny:
                w = vline(px, py)
                for k in range(ch):
                    if w == WALL:
                        cv.put(line_row(py + 1) + 1 + k, col, glyphs["v_wall"], "wall")
                    elif w == UNKNOWN:
                        cv.put(line_row(py + 1) + 1 + k, col, glyphs["v_unknown"], "unknown")

    # The goal glows, through the open walls between its cells.
    for x, y in model.goal_cells():
        if not inside(x, y):
            continue
        row, col = line_row(y + 1) + 1, line_col(x) + 1
        cv.fill(row, col, ch, cw, " ", "goal_bg")
        if model.is_goal(x + 1, y) and inside(x + 1, y) and vline(x + 1, y) in OPEN:
            cv.fill(row, col + cw, ch, 1, " ", "goal_bg")
        if model.is_goal(x, y + 1) and inside(x, y + 1) and hline(x, y + 1) in OPEN:
            cv.fill(row - 1, col, 1, cw, " ", "goal_bg")
        if all(model.is_goal(x + i, y + j) and inside(x + i, y + j) for i in (0, 1) for j in (0, 1)) and \
                all(w in OPEN for w in (hline(x, y + 1), hline(x + 1, y + 1), vline(x + 1, y), vline(x + 1, y + 1))):
            cv.put(line_row(y + 1), line_col(x + 1), " ", "goal_bg")
    if model.pose and inside(*model.pose[:2]) and cw >= 3:
        cv.fill(line_row(model.pose[1] + 1) + 1, line_col(model.pose[0]) + 1, ch, cw, " ", "halo")

    # Paths as traces between the cells' pads, like a circuit board.
    traces = []
    if fast:
        traces.append(([start] + ov.fast_path, "fast_trace"))
    if route and model.pose:
        traces.append(([model.pose[:2]] + ov.route, "route_trace"))
    blank = (" ", glyphs["h_unknown"], glyphs["v_unknown"])
    for cells, style in traces:
        for a, b in zip(cells, cells[1:]):
            if abs(a[0] - b[0]) + abs(a[1] - b[1]) != 1 or not inside(*a) or not inside(*b):
                continue
            (ra, ca), (rb, cb) = center(*a), center(*b)
            if ra == rb:
                spots, glyph = [(ra, c) for c in range(min(ca, cb) + 1, max(ca, cb))], glyphs["h_trace"]
            else:
                spots, glyph = [(r, ca) for r in range(min(ra, rb) + 1, max(ra, rb))], glyphs["v_trace"]
            for r, c in spots:
                if 0 <= r < cv.rows and 0 <= c < cv.cols and cv.chars[r][c] in blank:
                    under = cv.styles[r][c]
                    cv.put(r, c, glyph, "goal" if under == "goal_bg" else "halo" if under == "halo" else style)

    for y in range(vy0, vy0 + vny):
        for x in range(vx0, vx0 + vnx):
            glyph, style = content(x, y)
            if glyph != " ":
                cv.put(*center(x, y), glyph, style)

    bottom = line_row(vy0) + 1
    if axes:    # hex coordinates, the robot's row and column lit
        for x in range(vx0, vx0 + vnx):
            cv.put(bottom, center(x, 0)[1], HEX[x], "axis_hot" if model.pose and x == model.pose[0] else "axis")
        for y in range(vy0, vy0 + vny):
            cv.put(center(0, y)[0], gx - 2, HEX[y], "axis_hot" if model.pose and y == model.pose[1] else "axis")
        bottom += 1

    items = [(glyphs["robot"][0], "robot", " robot"), (glyphs["path"], "route", " route"),
             (glyphs["path"], "fast", " fast"), (glyphs["cand"], "cand", " candidate"),
             ("G", "goal", " goal"), (glyphs["h_unknown"] * 2, "unknown", " unseen")]
    if vnx < nx or vny < ny:
        items.insert(0, ("view %dx%d of %dx%d" % (vnx, vny, nx, ny), "dim", ""))
    lx = gx - 2 * axes
    row, x = bottom, lx
    for glyph, style, label in items:     # wrapped to the panel width, as many rows as fit
        need = len(glyph) + len(label)
        if x > lx and x + need > left + width:
            row, x = row + 1, lx
        if row >= top + height:
            return
        cv.put(row, x, glyph, style, left + width - x)
        cv.put(row, x + len(glyph), label, "dim", left + width - x - len(glyph))
        x += need + 2


def draw_splash(cv, top, left, height, width, view, glyphs, now):
    """Before any telemetry: hex rain behind the logo, waiting for the robot."""
    for col in range(0, width, 2):
        seed = col * 7919 % 101
        speed, length = 4 + seed % 7, 3 + seed % 6
        head = int(now * speed + seed * 3) % (height + length)
        for k in range(length):
            row = head - k
            if 0 <= row < height:
                cv.put(top + row, left + col, HEX[(seed + row * 7 + int(now * 3)) % 16],
                       "rain_head" if k == 0 else "rain")
    logo = next((logo for logo in (LOGOS_ASCII if view.ascii else LOGOS_UNICODE)
                 if len(logo[0]) + 4 <= width and len(logo) + 6 <= height), ())
    if view.connected:
        status = "link up, waiting for telemetry"
    else:
        status = view.link_text or "opening %s" % view.source
    status += glyphs["cursor"] if int(now * 2) % 2 == 0 else " "
    lines = [(text, None) for text in logo]
    lines += [("", ""), ("MICROMOUSE TELEMETRY CONSOLE", "dim"), ("", ""), (status, "warn")]
    block_w = min(width, max(len(text) for text, _ in lines) + 4)
    y0 = top + max(0, (height - len(lines) - 2) // 2)
    x0 = left + (width - block_w) // 2
    cv.fill(y0 - 1, x0, len(lines) + 2, block_w, " ", "")
    for i, (text, style) in enumerate(lines):
        x = left + max(0, (width - len(text)) // 2)
        if style is None:   # a logo row: gradient across, shadows dimmer
            for j, c in enumerate(text):
                shadow = c not in "█▀▄" and not view.ascii
                cv.put(y0 + i, x + j, c, "logo_shadow" if shadow else "logo%d" % (j * 6 // len(text)),
                       left + width - x - j)
        else:
            cv.put(y0 + i, x, text, style, left + width - x)
    mouse = glyphs["mouse"]
    row = y0 + len(lines) + 1
    if row < top + height:
        pos = int(now * 12) % (width + len(mouse)) - len(mouse)
        visible = mouse[max(0, -pos):max(0, width - pos)]
        cv.put(row, left + max(0, pos), visible, "key")


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
    ("keyline", "  /full /ascii /clear  16x16 view, symbols, clear the log"),
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
    ("", "  robot and its heading, planned route, candidate cells"),
    ("", "  G goal, S start, dot = visited cell (yellow: this run)"),
    ("", "  dotted lines = walls not confirmed yet"),
    ("", "  stopped: the verified fast path is drawn"),
]


class ViewState:
    """What the screen shows besides the model: log, input, panels."""

    def __init__(self):
        self.log = collections.deque(maxlen=5000)
        self.scroll = 0
        self.show_help = False
        self.full = False
        self.ascii = False
        self.input = ""
        self.connected = False
        self.link_text = ""
        self.source = ""
        self.replay = False
        self.capture = None     # (test, samples) while a calibration dump arrives
        self.rx_times = collections.deque(maxlen=4096)     # arrival of every line, for the RX activity
        self.started = time.monotonic()
        self.recording = ""


FRAMED_ROWS, FRAMED_COLS = 20, 60   # panels get their boxes from this size on
STATUS_ROWS, STATUS_COLS = 8, 40


def render(rows, cols, model, ov, view, start, now):
    """Composes one frame. Returns the canvas and the cursor position."""
    cv = Canvas(rows, cols)
    glyphs = GLYPHS_ASCII if view.ascii else GLYPHS_UNICODE
    if rows < MIN_ROWS or cols < MIN_COLS:
        cv.put(0, 0, "Terminal too small", "warn")
        cv.put(1, 0, "minimum %dx%d, now %dx%d" % (MIN_COLS, MIN_ROWS, cols, rows), "dim")
        return cv, (min(2, rows - 1), 0)

    framed = rows >= FRAMED_ROWS and cols >= FRAMED_COLS
    f = 2 if framed else 0
    body_top, body_h = 1, rows - 3
    nx, ny = view_size(model, view.full)
    side_need = MIN_LOG_COLS + (4 if framed else 2)
    inner_w = maze_panel_width(nx, ny, cols - side_need, body_h - f)
    if inner_w is not None:     # side by side
        if framed:
            maze_box, side_box = (body_top, 0, body_h, inner_w + 2), (body_top, inner_w + 2, body_h, cols - inner_w - 2)
        else:
            sep_x = inner_w + 2
            maze_box, side_box = (body_top, 1, body_h, inner_w), (body_top, sep_x + 1, body_h, cols - sep_x - 1)
            for row in range(body_top, body_top + body_h):
                cv.put(row, sep_x, glyphs["sep"], "border")
    else:                       # stacked
        maze_h = min(2 * ny + 2 + f, max(3 + f, body_h * 2 // 3))
        maze_box = (body_top, 0, maze_h, cols) if framed else (body_top, 1, maze_h, cols - 2)
        side_box = (body_top + maze_h, 0, body_h - maze_h, cols)

    top, left, height, width = maze_box
    if framed:
        draw_frame(cv, top, left, height, width, glyphs, "0x01", "MAZE %dx%d" % (nx, ny))
        top, left, height, width = top + 1, left + 1, height - 2, width - 2
    if model.last_telemetry is None:
        draw_splash(cv, top, left, height, width, view, glyphs, now)
    else:
        draw_maze(cv, top, left, height, width, model, ov, glyphs, view.full, start)

    top, left, height, width = side_box
    status = framed and inner_w is not None and height >= STATUS_ROWS + 6 and width >= STATUS_COLS
    if status:
        draw_frame(cv, top, left, STATUS_ROWS, width, glyphs, "0x02", "STATUS")
        draw_status(cv, top + 1, left + 1, STATUS_ROWS - 2, width - 2, model, ov, view, glyphs, now)
        top, height = top + STATUS_ROWS, height - STATUS_ROWS
    if framed and height >= 3:
        title, note = draw_console(cv, top + 1, left + 1, height - 2, width - 2, view, glyphs)
        draw_frame(cv, top, left, height, width, glyphs, "0x03", title, note)
    elif height >= 2:
        title, note = draw_console(cv, top + 1, left, height - 1, width, view, glyphs)
        cv.put(top, left, " %s%s " % (title, note), "head", width)

    draw_header(cv, cols, model, ov, view, glyphs, now, compact=not status)

    # Prompt and key bar.
    # The arrow shows the link, like a shell's last exit status.
    prompt = [("%s " % glyphs["prompt"], "ok" if view.connected else "bad")]
    x = put_segments(cv, rows - 2, 0, cols, prompt)
    room = max(0, cols - x - 1)
    visible = view.input[-room:] if room else ""
    cv.put(rows - 2, x, visible, "value")
    draw_keys(cv, rows - 1, cols, view, glyphs)
    return cv, (rows - 2, min(cols - 1, x + len(visible)))


def draw_header(cv, cols, model, ov, view, glyphs, now, compact):
    cv.fill(0, 0, 1, cols, " ", "title")
    brand = " %s RATATRON " % glyphs["mouse"]
    for i, c in enumerate(brand):
        cv.put(0, i, c, "brand%d" % (i * len(BRAND_COLOURS) // len(brand)))
    x = len(brand)
    cv.put(0, x, glyphs["fade"], "brand_fade")
    x += len(glyphs["fade"])
    rx_age = now - view.rx_times[-1] if view.rx_times else INF
    led = glyphs["dot_on"] if view.connected else glyphs["dot_off"]
    parts = [("%s %s" % (led, view.source),
              "title_bad" if not view.connected else "title_hot" if rx_age < 0.15 else "title_ok")]
    if view.capture:
        parts.append(("receiving %s: %d samples" % view.capture, "title_hot"))
    if model.mode:
        parts.append(("%d %s" % (model.mode, MODE_NAME[model.mode]), "title_hot"))
    if model.activity:
        spin = spinner(glyphs, now) + " " if model.activity in RUNNING else ""
        parts.append((spin + ACTIVITY_NAME[model.activity], "title"))
    if compact:     # no status panel: its facts go here
        if model.pose:
            parts.append(("(%d,%d)%s" % (model.pose[0], model.pose[1], HEADINGS[model.pose[2]]), "title"))
        parts.append(("%d cells" % model.visited_count(), "title"))
        if ov.fast_cost is not None:
            parts.append(("fast: %d cells %d turns" % (len(ov.fast_path), ov.fast_turns), "title"))
        parts.append((telemetry_age(model, now), "title"))
    for i, (text, style) in enumerate(parts):
        x = put_segments(cv, 0, x, cols - x, [(" " + glyphs["sep"] + " " if i else " ", "title_sep"), (text, style)])
    up = int(now - view.started)
    clock = " T+%02d:%02d:%02d " % (up // 3600, up // 60 % 60, up % 60)
    if x + len(clock) + 1 <= cols:
        cv.put(0, cols - len(clock), clock, "title_hot")


def draw_status(cv, top, left, height, width, model, ov, view, glyphs, now):
    """Mode, pose, explored share, fast path, link and RX activity."""
    def line(row, label, segments):
        if row < height:
            put_segments(cv, top + row, left + 1, width - 1, [(label.ljust(6), "label")] + segments)

    act = model.activity
    doing = []
    if act in RUNNING:
        doing = [("  " + spinner(glyphs, now) + " ", "key"), (ACTIVITY_NAME[act], "")]
    elif act:
        doing = [("  " + ACTIVITY_NAME[act], "dim")]
    line(0, "MODE", [("%d %s" % (model.mode, MODE_NAME[model.mode]), "value") if model.mode else ("--", "dim")] + doing)

    x0, y0, x1, y1 = model.goal
    goal = "goal (%d,%d)" % (x0, y0) if (x0, y0) == (x1, y1) else "goal (%d,%d)-(%d,%d)" % model.goal
    pose = ("(%d,%d) %s" % (model.pose[0], model.pose[1], HEADINGS[model.pose[2]]), "value") if model.pose else ("--", "dim")
    line(1, "POSE", [pose, ("   " + goal, "dim")])

    nx, ny = view_size(model, view.full)
    explored = sum(model.visited[x][y] for x in range(nx) for y in range(ny))
    bar_w = max(4, min(48, width - 1 - 6 - 9))
    filled = bar_w * explored // (nx * ny)
    line(2, "MAP", [(glyphs["bar"][0] * filled, "bar_on"), (glyphs["bar"][1] * (bar_w - filled), "bar_off"),
                    (" %d/%d" % (explored, nx * ny), "value")])

    if ov.fast_cost is None:
        line(3, "FAST", [("no verified path yet", "dim")])
    else:
        line(3, "FAST", [("%d cells" % len(ov.fast_path), "fast"), ("  %d turns" % ov.fast_turns, ""),
                         ("  cost %d" % ov.fast_cost, "dim")])

    state = ("%s up" % glyphs["dot_on"], "ok") if view.connected else \
        ("%s %s" % (glyphs["dot_off"], view.link_text or "down"), "bad")
    line(4, "LINK", [state, ("  " + telemetry_age(model, now), "dim"),
                     ("  bad %d" % model.bad_lines, "bad" if model.bad_lines else "dim")])

    span = max(4, min(60, width - 1 - 6 - 9))    # one column per second
    counts = [0] * span
    for t in view.rx_times:
        age = int(now - t)
        if 0 <= age < span:
            counts[span - 1 - age] += 1
    peak, levels = max(counts) or 1, glyphs["spark"]
    spark = "".join(levels[0] if c == 0 else levels[1 + (c * (len(levels) - 2) + peak - 1) // peak] for c in counts)
    rate = sum(1 for t in view.rx_times if now - t < 5) / 5
    line(5, "RX", [(spark, "spark"), (" %5.1f/s" % rate, "value")])


def draw_console(cv, top, left, height, width, view, glyphs):
    """The log (or the help) inside the box; returns its title and note."""
    if height < 1 or width < 2:
        return "CONSOLE", ""
    if view.show_help:
        for i, (style, text) in enumerate(HELP_LINES[:height]):
            if style == "keyline":     # the keys, then what they do in a column
                key, _, rest = text.strip().partition("  ")
                x = put_segments(cv, top + i, left + 3, width - 3, [(key, "key")])
                x = max(x + 2, left + 16)
                cv.put(top + i, x, rest.strip(), "", left + width - x)
            elif style == "head":
                cv.put(top + i, left, glyphs["gutter"] + text, "head", width)
            else:
                cv.put(top + i, left + 1, text, style, width - 1)
        return "HELP", "  Tab: back to the log"
    lines = list(view.log)
    view.scroll = max(0, min(view.scroll, max(0, len(lines) - height)))
    first = max(0, len(lines) - height - view.scroll)
    for i, (style, text) in enumerate(lines[first:first + height]):
        y = top + i
        if style not in ("", "dim", "boot"):
            cv.put(y, left, glyphs["gutter"], style)
        if style == "boot" and text.startswith("["):
            cv.put(y, left + 1, text[:6], "ok" if "OK" in text[:6] else "warn", width - 1)
            cv.put(y, left + 7, text[6:], "dim", width - 7)
        else:
            cv.put(y, left + 1, text, style, width - 1)
    note = "  %s %d more lines below" % (glyphs["down"], view.scroll) if view.scroll else ""
    return "CONSOLE", note


def draw_keys(cv, row, cols, view, glyphs):
    """htop-style key bar, the recording light on the right."""
    cv.fill(row, 0, 1, cols, " ", "keylabel")
    x = 0
    for key, label in (("Enter", "send"), ("^X", "STOP"), ("Tab", "help"), ("Esc", "quit"),
                       (glyphs["updown"], "history"), ("PgUp/Dn", "log")):
        cap, text = " %s " % key, "%s " % label
        if x + len(cap) + len(text) > cols - 1:
            break
        cv.put(row, x, cap, "keycap_stop" if key == "^X" else "keycap")
        cv.put(row, x + len(cap), text, "keylabel")
        x += len(cap) + len(text) + 1
    if view.recording:
        rec = "%s REC %s " % (glyphs["dot_on"], view.recording)
        if x + len(rec) + 1 < cols:
            cv.put(row, cols - 1 - len(rec), rec, "rec")


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
            self.on_status(False, "no se puede leer %s: %s" % (name, exc))
            return
        self.on_status(True, "reproduciendo " + name)
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
                self.on_line(text)
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
        self.view.full, self.view.ascii = args.full, args.ascii
        self.view.replay = bool(args.replay)
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
    def on_line(self, text):
        if self.recorder:
            self.recorder.write("<", text)
        with self.lock:
            self.view.rx_times.append(time.monotonic())
            if text.startswith("@D"):
                message = self.capture.feed(text)
                if message:
                    self.add_log(*message)
                self.view.capture = self.capture.progress()
            elif not self.model.apply(text) and text.strip():
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
            elif name == "/ascii":
                self.view.ascii = not self.view.ascii
            elif name == "/clear":
                self.view.log.clear()
                self.view.scroll = 0
            elif name == "/note" and rest.strip():
                try:
                    self.add_log(*self.capture.note(sanitize(rest.strip())))
                except OSError as exc:
                    self.add_log("bad", "could not write the note: %s" % exc)
            else:
                self.add_log("warn", "local commands: /full /ascii /clear /note <text>")
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

    def handle_key(self, key, stdscr):
        """False to quit."""
        with self.lock:
            self.dirty = True
            view = self.view
            if key == 27:
                return False
            if key in (curses.KEY_ENTER, 10, 13):
                self.submit()
            elif key in (curses.KEY_BACKSPACE, 127, 8):
                view.input = view.input[:-1]
                self.history_pos = None
            elif key == 21:     # Ctrl+U
                view.input = ""
            elif key == 24:     # Ctrl+X
                self.send("STOP")
            elif key == 12:     # Ctrl+L
                stdscr.clear()
            elif key == 9:
                view.show_help = not view.show_help
            elif key == curses.KEY_UP and self.history:
                self.history_pos = len(self.history) - 1 if self.history_pos is None else max(0, self.history_pos - 1)
                view.input = self.history[self.history_pos]
            elif key == curses.KEY_DOWN and self.history_pos is not None:
                self.history_pos += 1
                if self.history_pos >= len(self.history):
                    self.history_pos, view.input = None, ""
                else:
                    view.input = self.history[self.history_pos]
            elif key == curses.KEY_PPAGE:
                view.scroll += 10
            elif key == curses.KEY_NPAGE:
                view.scroll = max(0, view.scroll - 10)
            elif key == curses.KEY_HOME:
                view.scroll = len(view.log)
            elif key == curses.KEY_END:
                view.scroll = 0
            elif key == curses.KEY_RESIZE:
                curses.update_lines_cols()
            elif 32 <= key < 127 and len(view.input) < 60:
                view.input += chr(key)
                self.history_pos = None
        return True

    def frame(self, rows, cols):
        with self.lock:
            ov = self.planner.overlay(self.model)
            return render(rows, cols, self.model, ov, self.view, self.start, time.monotonic())

    def run(self, stdscr):
        styles = init_styles()
        stdscr.bkgd(" ", styles[""])
        stdscr.keypad(True)
        stdscr.timeout(50)
        try:
            curses.curs_set(1)
        except curses.error:
            pass
        self.link.start()
        last_draw = 0.0
        try:
            while True:
                key = stdscr.getch()
                if key != -1 and not self.handle_key(key, stdscr):
                    break
                now = time.monotonic()
                with self.lock:
                    # Animations (splash, spinner, RX light) at 10 frames/s, else 2.
                    rx = self.view.rx_times
                    animated = self.model.last_telemetry is None or self.model.activity in RUNNING \
                        or (rx and now - rx[-1] < 0.5)
                    redraw = self.dirty or now - last_draw >= (0.1 if animated else 0.5)
                    self.dirty = False
                if redraw:
                    rows, cols = stdscr.getmaxyx()
                    cv, cursor = self.frame(rows, cols)
                    blit(stdscr, cv, styles, cursor)
                    last_draw = now
        finally:
            self.link.stop()
            if self.recorder:
                self.recorder.close()


XTERM16 = ((0, 0, 0), (128, 0, 0), (0, 128, 0), (128, 128, 0), (0, 0, 128), (128, 0, 128), (0, 128, 128),
           (192, 192, 192), (128, 128, 128), (255, 0, 0), (0, 255, 0), (255, 255, 0), (0, 0, 255),
           (255, 0, 255), (0, 255, 255), (255, 255, 255))
BASIC_RGB = ((0, 0, 0), (205, 0, 0), (0, 205, 0), (205, 205, 0), (0, 0, 238), (205, 0, 205), (0, 205, 205),
             (229, 229, 229))   # curses COLOR_BLACK .. COLOR_WHITE


def xterm_rgb(index):
    if index < 16:
        return XTERM16[index]
    if index < 232:
        i = index - 16
        return tuple((0, 95, 135, 175, 215, 255)[v] for v in (i // 36, i // 6 % 6, i % 6))
    gray = 8 + 10 * (index - 232)
    return gray, gray, gray


def basic_colour(index):
    """The nearest of the 8 basic colours to an xterm 256-colour index."""
    rgb = xterm_rgb(index)
    return min(range(8), key=lambda c: sum((a - b) ** 2 for a, b in zip(rgb, BASIC_RGB[c])))


def init_styles():
    """curses attributes of every THEME style: its own 256 colours, the nearest 8 basic
    ones (dark tones dimmed, on the terminal's background), or reverse video for bars."""
    styles = collections.defaultdict(int)
    colours, default = 0, curses.COLOR_BLACK
    if curses.has_colors():
        curses.start_color()
        try:
            curses.use_default_colors()
            default = -1
        except curses.error:
            pass
        colours = curses.COLORS
    pairs = {}
    for name, (fg, bg, bold) in THEME.items():
        attr = curses.A_BOLD if bold else 0
        if colours >= 256:
            key = fg, bg
        else:
            f, b = basic_colour(fg), basic_colour(bg) if bg != BG else curses.COLOR_BLACK
            if f == b == curses.COLOR_BLACK:
                f = curses.COLOR_WHITE
            if sum(xterm_rgb(fg)) < 400:
                attr |= curses.A_DIM
            key = f, (default if b == curses.COLOR_BLACK else b)
            if not colours and bg != BG:
                attr |= curses.A_REVERSE
        if colours:
            if key not in pairs and len(pairs) + 1 < curses.COLOR_PAIRS:
                try:
                    curses.init_pair(len(pairs) + 1, *key)
                    pairs[key] = len(pairs) + 1
                except curses.error:
                    pass
            if key in pairs:
                attr |= curses.color_pair(pairs[key])
        styles[name] = attr
    return styles


def blit(stdscr, cv, styles, cursor):
    """Copies the canvas to the screen. curses only transmits what changed,
    so redrawing everything each frame does not flicker."""
    stdscr.erase()
    for y in range(cv.rows):
        # Never write the bottom-right cell: curses errors after scrolling it.
        limit = cv.cols - 1 if y == cv.rows - 1 else cv.cols
        chars, row_styles = cv.chars[y], cv.styles[y]
        x = 0
        while x < limit:
            style = row_styles[x]
            end = x + 1
            while end < limit and row_styles[end] == style:
                end += 1
            try:
                stdscr.addstr(y, x, "".join(chars[x:end]), styles[style])
            except curses.error:
                pass
            x = end
    try:
        stdscr.move(*cursor)
    except curses.error:
        pass
    stdscr.refresh()


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
    parser.add_argument("--ascii", action="store_true", help="solo caracteres ASCII")
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    if not args.replay and serial is None:
        print("Falta pyserial: pip install pyserial (o usa --replay)", file=sys.stderr)
        return 1
    locale.setlocale(locale.LC_ALL, "")
    os.environ.setdefault("ESCDELAY", "50")     # Esc quits promptly (curses waits 1 s by default)
    monitor = Monitor(args, read_config())
    curses.wrapper(monitor.run)
    if monitor.recorder:
        print("Session recorded in", os.path.relpath(monitor.recorder.path))
    return 0


if __name__ == "__main__":
    sys.exit(main())
