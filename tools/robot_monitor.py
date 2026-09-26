#!/usr/bin/env python3
"""Live monitor and console for the rat micromouse.

Draws the maze while the robot explores it (from the robot's '@' telemetry
lines, see src/telemetry.h) next to the run log, and sends typed commands.
The planned route is recomputed here with the robot's own planner and costs
(read from src/robot_config.h), so the drawing shows what the robot will do.

    python3 tools/robot_monitor.py [--port /dev/rfcomm0] [--baud 9600]
    python3 tools/robot_monitor.py --replay tools/logs/<session>.log [--speed 4]

Sessions are recorded to tools/logs/ (--no-record to disable) and can be
replayed. Calibration dumps (CAL commands) are saved as CSV files in
tools/calib_data/; add your own measurements to the last one with /nota.
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

MODE_NAME = {1: "BUSQUEDA", 2: "CARRERA RAPIDA", 3: "SENSORES", 4: "BORRAR MAPA"}
ACTIVITY_NAME = {"I": "parado", "C": "cuenta atras", "G": "explorando hacia la meta",
                 "O": "optimizando la ruta", "H": "explorando hacia la salida",
                 "F": "carrera rapida", "R": "volviendo a la salida",
                 "S": "monitor de sensores", "E": "borrando el mapa", "K": "calibrando"}
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
        text += str(run) + ("D" if t > 0 else "I" if t < 0 else "")
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
            return "info", "recibiendo datos de calibracion: " + (rest or "?")
        if self.active is None:
            return None
        if word == "INFO":
            self.active["info"].append(rest)
        elif word == "COLS":
            self.active["cols"] = rest
        elif word == "END":
            capture, self.active = self.active, None
            if not self.save_files:
                return "info", "datos de calibracion: %d muestras (reproduccion: no se guardan)" % len(capture["rows"])
            try:
                path = self._save(capture, rest)
            except OSError as exc:
                return "bad", "no se pudieron guardar los datos: %s" % exc
            self.last_path = path
            return "ok", "datos guardados en %s (%d muestras). Anota medidas con /nota <texto>" % (
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
            return "warn", "no hay datos de calibracion a los que anadir la nota"
        with open(self.last_path, "r+", encoding="utf-8") as f:
            lines = f.readlines()
            header = 0
            while header < len(lines) and lines[header].startswith("#"):
                header += 1
            lines.insert(header, "# nota: %s\n" % text)
            f.seek(0)
            f.writelines(lines)
            f.truncate()
        return "ok", "nota anadida a %s" % os.path.relpath(self.last_path)


# ---- Rendering into an off-screen canvas ------------------------------------------------

GLYPHS_UNICODE = {
    "robot": "▲▶▼◀", "path": "•", "cand": "◇", "visited": "·", "post": "·",
    "h_wall": "─", "v_wall": "│", "h_unknown": "┄", "v_unknown": "┆",
    "junction": " ╶╷┌╴─┐┬╵└│├┘┴┤┼", "dot_on": "●", "dot_off": "○", "sep": "│",
    "down": "↓",
}
GLYPHS_ASCII = {
    "robot": "^>v<", "path": "*", "cand": "o", "visited": ".", "post": "+",
    "h_wall": "-", "v_wall": "|", "h_unknown": ".", "v_unknown": ":",
    "junction": "++++++++++++++++", "dot_on": "*", "dot_off": "o", "sep": "|",
    "down": "v",
}


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


def maze_width(nx, cw):
    return nx * (cw + 1) + 1


def draw_maze(cv, top, left, height, width, model, ov, glyphs, full, start):
    """Draws the maze inside the box (top, left, height, width)."""
    nx, ny = view_size(model, full)
    cw = next((w for w in (3, 2, 1) if maze_width(nx, w) <= width), 1)
    rows_for_maze = max(1, height - 1)      # last row: legend
    vnx = min(nx, max(1, (width - 1) // (cw + 1)))
    vny = min(ny, max(1, (rows_for_maze - 1) // 2))
    rx, ry = (model.pose[0], model.pose[1]) if model.pose else (0, 0)
    vx0 = min(max(0, rx - vnx // 2), nx - vnx)
    vy0 = min(max(0, ry - vny // 2), ny - vny)
    running = model.activity in RUNNING
    route = set(ov.route)
    fast = set() if running else set(ov.fast_path)
    trail = set(model.trail)

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
        # Route markers win over the goal letter (a route into the goal must
        # stay visible) but take the goal colour there.
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
            return glyphs["visited"], "trail" if (x, y) in trail else "dim"
        return " ", ""

    for row, y in enumerate(range(vy0 + vny - 1, vy0 - 1, -1)):
        wall_y, cell_y = top + 2 * row, top + 2 * row + 1
        for col, x in enumerate(range(vx0, vx0 + vnx)):
            px = left + col * (cw + 1)
            cv.put(wall_y, px, *post(x, y + 1))
            w = hline(x, y + 1)
            if w == WALL:
                cv.put(wall_y, px + 1, glyphs["h_wall"] * cw, "wall")
            elif w == UNKNOWN:
                cv.put(wall_y, px + 1, glyphs["h_unknown"] * cw, "unknown")
            w = vline(x, y)
            if w == WALL:
                cv.put(cell_y, px, glyphs["v_wall"], "wall")
            elif w == UNKNOWN:
                cv.put(cell_y, px, glyphs["v_unknown"], "unknown")
            ch, style = content(x, y)
            cv.put(cell_y, px + 1 + (cw - 1) // 2, ch, style)
        right = left + vnx * (cw + 1)
        cv.put(wall_y, right, *post(vx0 + vnx, y + 1))
        w = vline(vx0 + vnx, y)
        if w == WALL:
            cv.put(cell_y, right, glyphs["v_wall"], "wall")
        elif w == UNKNOWN:
            cv.put(cell_y, right, glyphs["v_unknown"], "unknown")
    bottom = top + 2 * vny
    for col, x in enumerate(range(vx0, vx0 + vnx)):
        px = left + col * (cw + 1)
        cv.put(bottom, px, *post(x, vy0))
        w = hline(x, vy0)
        if w == WALL:
            cv.put(bottom, px + 1, glyphs["h_wall"] * cw, "wall")
        elif w == UNKNOWN:
            cv.put(bottom, px + 1, glyphs["h_unknown"] * cw, "unknown")
    cv.put(bottom, left + vnx * (cw + 1), *post(vx0 + vnx, vy0))

    items = ["%s robot" % glyphs["robot"][0], "%s ruta" % glyphs["path"], "%s candidata" % glyphs["cand"],
             "G meta", "%s sin ver" % (glyphs["h_unknown"] * 2)]
    if vnx < nx or vny < ny:
        items.insert(0, "vista %dx%d de %dx%d" % (vnx, vny, nx, ny))
    row, line = bottom + 1, ""
    for item in items:     # wrapped to the panel width, as many rows as fit
        if line and len(line) + 2 + len(item) > width:
            if row >= top + height:
                return
            cv.put(row, left, line, "dim", width)
            row, line = row + 1, ""
        line = item if not line else line + "  " + item
    if line and row < top + height:
        cv.put(row, left, line, "dim", width)


LOG_STYLES = (
    (lambda t: t.startswith("!!"), "bad"),
    (lambda t: "Meta alcanzada" in t or t.startswith("Fin: OK"), "ok"),
    (lambda t: t.startswith("=="), "head"),
    (lambda t: t.startswith("Fin:") or t.startswith("? "), "warn"),
    (lambda t: t.startswith("--"), "info"),
    (lambda t: t.startswith(("avance", "ruta", "exploracion", "giro", "alineado", "IR mm")), "dim"),
)


def log_style(text):
    for test, style in LOG_STYLES:
        if test(text):
            return style
    return ""


HELP_LINES = [
    ("head", "TECLAS"),
    ("", "  Enter        enviar el comando"),
    ("", "  Arriba/Abajo historial de comandos"),
    ("", "  RePag/AvPag  desplazar el log (Inicio/Fin)"),
    ("", "  Tab          log / esta ayuda"),
    ("", "  Ctrl+X       STOP inmediato del run"),
    ("", "  Ctrl+L       repintar la pantalla"),
    ("", "  Esc          salir"),
    ("", "  /full /ascii /clear  vista 16x16, simbolos, borrar log"),
    ("", "  /nota texto  anade una medida al ultimo fichero de calibracion"),
    ("", ""),
    ("head", "ROBOT (HELP en el robot los lista todos)"),
    ("", "  MODE n   1 busqueda 2 rapida 3/4 seguidor 5 sensores 6 borrar"),
    ("", "  START STOP PAUSE RESUME STEP ON|OFF"),
    ("", "  STATUS MAP IR WALLS SYNC TELEM ON|OFF"),
    ("", "  SPD FAST TURN TURNTICKS n   KP KI KD KE f   LOG 0-2"),
    ("", "  GOAL x y [x1 y1]  SAVE ERASE HOME DEFAULTS RESET"),
    ("", "  CAL NOISE|STRAIGHT|TURN|STEP|IR|DUMP  datos de calibracion"),
    ("", ""),
    ("head", "MAPA"),
    ("", "  robot con su orientacion, ruta prevista, celdas candidatas"),
    ("", "  G meta, S salida, punto = celda visitada (amarillo: este run)"),
    ("", "  lineas punteadas = paredes aun sin confirmar"),
    ("", "  parado: se dibuja el camino rapido verificado"),
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


def render(rows, cols, model, ov, view, start, now):
    """Composes one frame. Returns the canvas and the cursor position."""
    cv = Canvas(rows, cols)
    glyphs = GLYPHS_ASCII if view.ascii else GLYPHS_UNICODE
    if rows < MIN_ROWS or cols < MIN_COLS:
        cv.put(0, 0, "Terminal demasiado pequena", "warn")
        cv.put(1, 0, "minimo %dx%d, ahora %dx%d" % (MIN_COLS, MIN_ROWS, cols, rows), "dim")
        return cv, (min(2, rows - 1), 0)

    # Header.
    cv.fill(0, 0, 1, cols, " ", "title")
    parts = [" RAT ", (glyphs["dot_on"] if view.connected else glyphs["dot_off"]) + " " + view.source]
    if view.capture:
        parts.append("recibiendo %s: %d muestras" % view.capture)
    if model.mode:
        parts.append("%d %s" % (model.mode, MODE_NAME[model.mode]))
    if model.activity:
        parts.append(ACTIVITY_NAME[model.activity])
    if model.pose:
        parts.append("(%d,%d)%s" % (model.pose[0], model.pose[1], HEADINGS[model.pose[2]]))
    parts.append("%d celdas" % model.visited_count())
    if ov.fast_cost is not None:
        parts.append("rapida: %d celdas %d giros" % (len(ov.fast_path), ov.fast_turns))
    if model.last_telemetry is None:
        parts.append("sin telemetria")
    else:
        age = now - model.last_telemetry
        parts.append("tlm %.1fs" % age if age < 99 else "tlm --")
    x = 0
    for i, part in enumerate(parts):
        style = "title" if i != 1 else "title_ok" if view.connected else "title_bad"
        text = part if i == 0 else " " + glyphs["sep"] + " " + part
        cv.put(0, x, text, style)
        x += len(text)

    body_top, body_h = 1, rows - 3
    nx, ny = view_size(model, view.full)
    maze_w = next((maze_width(nx, cw) for cw in (3, 2, 1)
                   if maze_width(nx, cw) + 2 + MIN_LOG_COLS <= cols and 2 * ny + 2 <= body_h), None)
    if maze_w is not None:      # side by side
        sep_x = maze_w + 2
        maze_box = (body_top, 1, body_h, maze_w)
        side_box = (body_top, sep_x + 1, body_h, cols - sep_x - 1)
        for row in range(body_top, body_top + body_h):
            cv.put(row, sep_x, glyphs["sep"], "border")
    else:                       # stacked
        maze_h = min(2 * ny + 2, max(3, body_h * 2 // 3))
        maze_box = (body_top, 1, maze_h, cols - 2)
        side_box = (body_top + maze_h, 0, body_h - maze_h, cols)
    draw_maze(cv, *maze_box, model, ov, glyphs, view.full, start)
    draw_side_panel(cv, *side_box, view, glyphs)

    # Input line and key hints.
    prompt = "(reproduccion) > " if view.replay else "> "
    cv.put(rows - 2, 0, prompt, "key")
    room = max(0, cols - len(prompt) - 1)
    visible = view.input[-room:] if room else ""
    cv.put(rows - 2, len(prompt), visible)
    hints = "Enter enviar  Arriba/Abajo historial  RePag/AvPag log  Tab ayuda  Ctrl+X STOP  Esc salir"
    cv.put(rows - 1, 0, hints, "dim", cols - 1)
    return cv, (rows - 2, min(cols - 1, len(prompt) + len(visible)))


def draw_side_panel(cv, top, left, height, width, view, glyphs):
    if height < 2 or width < 4:
        return
    if view.show_help:
        cv.put(top, left, " AYUDA (Tab: volver al log) ", "head", width)
        for i, (style, text) in enumerate(HELP_LINES[:height - 1]):
            cv.put(top + 1 + i, left + 1, text, style, width - 1)
        return
    rows = height - 1
    lines = list(view.log)
    view.scroll = max(0, min(view.scroll, max(0, len(lines) - rows)))
    first = max(0, len(lines) - rows - view.scroll)
    title = " LOG "
    if view.scroll:
        title += "(%s %d lineas mas abajo) " % (glyphs["down"], view.scroll)
    cv.put(top, left, title, "head", width)
    for i, (style, text) in enumerate(lines[first:first + rows]):
        cv.put(top + 1 + i, left + 1, text, style, width - 1)


# ---- Links: serial port, replay file, session recorder -------------------------------------

def friendly_error(exc):
    text = str(exc)
    low = text.lower()
    if "exclusively lock" in low or "resource temporarily unavailable" in low:
        return "puerto ocupado por otro programa (otro monitor abierto?)"
    if "no such file" in low:
        return "no existe el puerto (rfcomm bind? ver tools/rfcomm_reconnect.sh)"
    if "permission denied" in low:
        return "sin permiso sobre el puerto (grupo dialout)"
    if "host is down" in low or "connection refused" in low or "no route" in low:
        return "robot apagado o fuera de alcance"
    if "readiness to read but returned no data" in low:
        return "enlace Bluetooth caido"
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
        self.on_status(False, "fin de la reproduccion de " + name)

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

    # -- called from the link thread
    def on_line(self, text):
        if self.recorder:
            self.recorder.write("<", text)
        with self.lock:
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
            self.recorder.write("!", message or ("conectado" if connected else "desconectado"))
        with self.lock:
            changed = connected != self.view.connected or message != self.view.link_text
            self.view.connected, self.view.link_text = connected, message
            if changed and message:
                self.add_log("info" if connected else "bad", "-- " + message + " --")
            elif changed and connected:
                self.add_log("info", "-- conectado a %s --" % self.view.source)
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
                self.add_log("dim", "> %s (reproduccion: no se envia)" % command)
            else:
                self.add_log("bad", "> %s (sin conexion: no enviado)" % command)
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
            elif name == "/nota" and rest.strip():
                try:
                    self.add_log(*self.capture.note(sanitize(rest.strip())))
                except OSError as exc:
                    self.add_log("bad", "no se pudo escribir la nota: %s" % exc)
            else:
                self.add_log("warn", "comandos locales: /full /ascii /clear /nota <texto>")
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
                    redraw = self.dirty or now - last_draw >= 0.5
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


def init_styles():
    styles = collections.defaultdict(int)
    if curses.has_colors():
        curses.start_color()
        try:
            curses.use_default_colors()
            bg = -1
        except curses.error:
            bg = curses.COLOR_BLACK
        pairs = {
            "ok": (curses.COLOR_GREEN, bg), "bad": (curses.COLOR_RED, bg),
            "warn": (curses.COLOR_YELLOW, bg), "info": (curses.COLOR_CYAN, bg),
            "wall": (curses.COLOR_WHITE, bg), "unknown": (curses.COLOR_BLUE, bg),
            "robot": (curses.COLOR_YELLOW, bg), "route": (curses.COLOR_GREEN, bg),
            "fast": (curses.COLOR_CYAN, bg), "cand": (curses.COLOR_CYAN, bg),
            "goal": (curses.COLOR_MAGENTA, bg), "trail": (curses.COLOR_YELLOW, bg),
            "sent": (curses.COLOR_YELLOW, bg), "head": (curses.COLOR_MAGENTA, bg),
            "key": (curses.COLOR_CYAN, bg), "border": (curses.COLOR_BLUE, bg),
            "title": (curses.COLOR_BLACK, curses.COLOR_CYAN),
            "title_ok": (curses.COLOR_BLACK, curses.COLOR_GREEN),
            "title_bad": (curses.COLOR_WHITE, curses.COLOR_RED),
        }
        for i, (name, (fg, pair_bg)) in enumerate(pairs.items(), start=1):
            try:
                curses.init_pair(i, fg, pair_bg)
                styles[name] = curses.color_pair(i)
            except curses.error:
                pass
    else:
        styles["title"] = styles["title_ok"] = styles["title_bad"] = curses.A_REVERSE
    for name in ("bad", "ok", "wall", "robot", "route", "goal", "sent", "head",
                 "title", "title_ok", "title_bad"):
        styles[name] |= curses.A_BOLD
    styles["dim"] |= curses.A_DIM
    styles["unknown"] |= curses.A_DIM
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
    parser.add_argument("--port", default="/dev/rfcomm0", help="puerto serie (por defecto /dev/rfcomm0)")
    parser.add_argument("--baud", type=int, default=9600)
    parser.add_argument("--replay", metavar="FICHERO", help="reproduce una sesion grabada (o una transcripcion)")
    parser.add_argument("--speed", type=float, default=1.0, help="velocidad de reproduccion (por defecto 1)")
    parser.add_argument("--no-record", action="store_true", help="no grabar la sesion")
    parser.add_argument("--log-dir", default=LOG_DIR, help="carpeta de sesiones grabadas")
    parser.add_argument("--calib-dir", default=CALIB_DIR, help="carpeta de datos de calibracion")
    parser.add_argument("--full", action="store_true", help="dibujar siempre el laberinto 16x16 entero")
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
        print("Sesion grabada en", os.path.relpath(monitor.recorder.path))
    return 0


if __name__ == "__main__":
    sys.exit(main())
