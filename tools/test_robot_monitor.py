#!/usr/bin/env python3
"""Tests for robot_monitor.py:  python3 -m unittest discover -s tools

The key test replays what the real firmware sends over Bluetooth during
simulated runs (test/host --transcript) and checks, at every decision the
robot logs, that the monitor's planner computes the same cost and action.
"""
import copy
import fcntl
import os
import pty
import re
import select
import signal
import struct
import subprocess
import sys
import tempfile
import termios
import time
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)

import robot_monitor as rm  # noqa: E402

HOST_DIR = os.path.join(ROOT, "test", "host")
HOST_TESTS = os.path.join(HOST_DIR, "build", "host_tests")
MONITOR = os.path.join(HERE, "robot_monitor.py")
ACTION_NAME = {"F": "FORWARD", "L": "LEFT", "R": "RIGHT", "U": "U-TURN", None: "-"}
DECISION = re.compile(r"^(GOAL|OPTIM|RETURN) \((\d+),(\d+)\)([NESW]) F[01?] L[01?] R[01?] cost=(\d+) -> (.+)$")
ROUTE = re.compile(r"^(FAST|RETURN) \((\d+),(\d+)\)([NESW]) turn (-?\d+) \+ route ([0-9RL]*\+?) \((\d+) cells\)$")
FAST_COST = re.compile(r"^Fast path verified: cost (\d+)$")

_built = False


def firmware_transcript(seed, openings, practice=False, phantom=False):
    global _built
    if not _built:
        subprocess.run(["make", "-s", "-C", HOST_DIR, "build/host_tests"], check=True)
        _built = True
    args = [HOST_TESTS, "--transcript", str(seed), str(openings)]
    args += (["practice"] if practice else []) + (["phantom"] if phantom else [])
    return subprocess.run(args, check=True, capture_output=True, text=True).stdout.splitlines()


def map_state(model):
    # Border slots are never read (the border is always a wall); full rows fill them, cell lines do not.
    north = [col[:-1] for col in model.north]
    east = copy.deepcopy(model.east[:-1])
    return copy.deepcopy((north, east, model.visited))


class TranscriptChecker:
    """Feeds firmware output to a MazeModel and cross-checks the planner."""

    def __init__(self, test):
        self.test = test
        self.model = rm.MazeModel()
        self.planner = rm.Planner(rm.read_config())
        self.decisions = self.segments = self.syncs = self.fast_costs = 0

    def run(self, lines):
        snapshot, rows_left = None, None
        for line in lines:
            if line.startswith("@"):
                self.model.apply(line)
                if line.startswith("@Y") and snapshot is not None:
                    rows_left = int(line[2], 16) + 1
                elif line.startswith("@R") and rows_left is not None:
                    rows_left -= 1
                    if rows_left == 0:
                        # The map built during the run must equal the full sync's.
                        self.test.assertEqual(snapshot, (map_state(self.model), self.model.pose))
                        snapshot, rows_left = None, None
                        self.syncs += 1
            elif line == "#CHECK":
                snapshot = (map_state(self.model), self.model.pose)
            elif line.startswith("#RESULT"):
                self.test.assertTrue(line.endswith(" 0"), line)    # RUN_OK
            else:
                self.check_line(line)
        self.test.assertEqual(self.model.bad_lines, 0)

    def check_line(self, line):
        m = DECISION.match(line)
        if m:
            phase, x, y, h, cost, action = m.groups()
            x, y, h = int(x), int(y), rm.HEADINGS.index(h)
            self.test.assertEqual(self.model.pose, (x, y, h), line)
            if phase == "GOAL":
                targets = self.model.goal_cells()
            elif phase == "RETURN":
                targets = {self.planner.start}
            else:
                targets = self.planner.candidates(self.model)
            costs = self.planner.plan_to(self.model, targets, False, self.planner.search)
            self.test.assertEqual(costs[rm.state(x, y, h)], int(cost), line)
            best = self.planner.best_action(self.model, costs, x, y, h, False, self.planner.search)
            self.test.assertEqual(ACTION_NAME[best], action, line)
            self.decisions += 1
            return
        m = ROUTE.match(line)
        if m:
            tag, x, y, h, turn, text, cells = m.groups()
            x, y, h = int(x), int(y), rm.HEADINGS.index(h)
            self.test.assertEqual(self.model.pose, (x, y, h), line)
            targets = self.model.goal_cells() if tag == "FAST" else {self.planner.start}
            costs = self.planner.plan_to(self.model, targets, True, self.planner.fast)
            first, turns = self.planner.route(self.model, costs, x, y, h, True, self.planner.fast)
            self.test.assertEqual((first, rm.route_text(turns), len(turns)), (int(turn), text, int(cells)), line)
            self.segments += 1
            return
        m = FAST_COST.match(line)
        if m:
            self.test.assertEqual(self.planner.overlay(self.model).fast_cost, int(m.group(1)), line)
            self.fast_costs += 1


class TestAgainstFirmware(unittest.TestCase):
    def check(self, seed, openings, practice=False, phantom=False):
        checker = TranscriptChecker(self)
        checker.run(firmware_transcript(seed, openings, practice, phantom))
        self.assertGreater(checker.decisions, 5)
        self.assertGreater(checker.segments, 0)
        self.assertEqual(checker.syncs, 2)          # after the search and after the speed run
        self.assertEqual(checker.fast_costs, 2)
        return checker

    def test_practice_mazes(self):
        for seed in range(1, 16):
            for openings in (0, 2):
                with self.subTest(seed=seed, openings=openings):
                    self.check(seed, openings, practice=True)

    def test_map_repairs_keep_the_monitor_in_sync(self):
        # Phantom walls around the goal force forgetting walls mid-run; the monitor must still agree.
        for seed in range(1, 6):
            with self.subTest(seed=seed):
                lines = firmware_transcript(seed, 1, practice=True, phantom=True)
                self.assertTrue(any("repair" in line for line in lines))
                TranscriptChecker(self).run(lines)
                lines = firmware_transcript(seed * 31, 40, phantom=True)
                self.assertTrue(any("repair" in line for line in lines))
                TranscriptChecker(self).run(lines)

    def test_competition_mazes(self):
        total = 0
        for seed in range(1, 9):
            for openings in (0, 40, 150):
                with self.subTest(seed=seed, openings=openings):
                    total += self.check(seed * 7919, openings).decisions
        self.assertGreater(total, 1000)


class TestProtocol(unittest.TestCase):
    def test_messages(self):
        m = rm.MazeModel()
        for line in ("@G3232", "@M2", "@AG", "@P21E", "@C21E#o.?"):
            self.assertTrue(m.apply(line))
        self.assertEqual(m.goal, (3, 2, 3, 2))
        self.assertEqual(m.mode, 2)
        self.assertEqual(m.pose, (2, 1, 1))
        self.assertEqual(m.wall(2, 1, 0), rm.WALL)
        self.assertEqual(m.wall(2, 1, 1), rm.OPEN_VERIFIED)
        self.assertEqual(m.wall(3, 1, 3), rm.OPEN_VERIFIED)     # same wall from the other side
        self.assertEqual(m.wall(2, 1, 2), rm.OPEN_ONCE)
        self.assertEqual(m.wall(2, 1, 3), rm.UNKNOWN)
        self.assertTrue(m.visited[2][1])
        self.assertEqual(m.trail, [(2, 1)])
        self.assertTrue(m.apply("@R0" + "M" + "0" * 14 + "4"))
        self.assertEqual((m.north[0][0], m.east[0][0], m.visited[0][0]), (rm.OPEN_ONCE, rm.WALL, True))
        self.assertTrue(m.apply("@Y2"))
        self.assertFalse(m.visited[2][1])
        self.assertEqual(m.bad_lines, 0)

    def test_borders_are_walls(self):
        m = rm.MazeModel()
        self.assertEqual(m.wall(0, 0, 3), rm.WALL)
        self.assertEqual(m.wall(0, 0, 2), rm.WALL)
        self.assertEqual(m.wall(15, 15, 0), rm.WALL)
        self.assertEqual(m.wall(15, 15, 1), rm.WALL)
        m.apply("@CF0E????")    # border sides stay walls whatever the line says
        self.assertEqual(m.wall(15, 0, 1), rm.WALL)

    def test_corrupted_lines_are_counted_not_fatal(self):
        m = rm.MazeModel()
        for line in ("@P", "@PZZN", "@C00N#", "@R0XYZ", "@G9911", "@M9", "@AQ", "@Q", "@Y", "@C00X####"):
            self.assertTrue(m.apply(line), line)
        self.assertEqual(m.bad_lines, 10)
        self.assertIsNone(m.pose)
        self.assertFalse(m.apply("GOAL (0,0)N ..."))


class TestPlanner(unittest.TestCase):
    def test_open_field_prefers_few_turns(self):
        m = rm.MazeModel()
        p = rm.Planner(rm.read_config())
        costs = p.plan_to(m, m.goal_cells(), False, p.fast)
        self.assertEqual(costs[rm.state(0, 0, 0)], 14 * p.fast[0] + p.fast[1])
        self.assertEqual(p.best_action(m, costs, 0, 0, 0, False, p.fast), "F")
        self.assertEqual(p.best_action(m, costs, 0, 0, 2, False, p.fast), "L")
        verified = p.plan_to(m, m.goal_cells(), True, p.fast)
        self.assertGreaterEqual(verified[rm.state(0, 0, 0)], rm.INF)
        self.assertIsNone(p.overlay(m).fast_cost)


def practice_model():
    checker = TranscriptChecker(unittest.TestCase())
    lines = firmware_transcript(3, 0, practice=True)
    checker.run(lines[:lines.index("#CHECK")])
    return checker.model, checker.planner


class TestRender(unittest.TestCase):
    SIZES = [(24, 80), (30, 100), (40, 120), (50, 200), (60, 250), (12, 44), (20, 60), (16, 50), (14, 46), (11, 43), (5, 20)]

    def view_with_log(self):
        view = rm.ViewState()
        view.source = "/dev/rfcomm0"
        view.connected = True
        for i in range(300):
            view.log.append(("", rm.sanitize("line %d \x1b[31m with\tcontrol \x00 and very long " % i + "x" * 300)))
        view.input = "SPD 150" + "9" * 70
        return view

    def check_frame(self, scr, cursor, rows, cols):
        self.assertEqual((scr.rows, scr.cols), (rows, cols))
        for y in range(rows):
            self.assertEqual((len(scr.ch[y]), len(scr.fg[y]), len(scr.bg[y])), (cols, cols, cols))
            self.assertTrue(all(len(c) == 1 and ord(c) >= 32 for c in scr.ch[y]))
        self.assertTrue(0 <= cursor[0] < rows and 0 <= cursor[1] < cols, cursor)

    def test_every_size(self):
        for model, planner in (practice_model(), self.competition_model()):
            ov = planner.overlay(model)
            for rows, cols in self.SIZES:
                for pixels, show_help in (("sextant", False), ("sextant", True), ("half", False)):
                    with self.subTest(rows=rows, cols=cols, pixels=pixels, help=show_help):
                        view = self.view_with_log()
                        view.pixels, view.show_help = pixels, show_help
                        scr, cursor = rm.render(rows, cols, model, ov, view, (0, 0), time.monotonic())
                        self.check_frame(scr, cursor, rows, cols)
                        if rows >= rm.MIN_ROWS and cols >= rm.MIN_COLS and pixels == "half":
                            # The robot is always in view: its white pixel among the maze's half blocks.
                            self.assertTrue(any(scr.ch[y][x] == "\u2580" and rm.WHITE in (scr.fg[y][x], scr.bg[y][x])
                                                for y in range(rows) for x in range(cols)))

    def test_a_character_from_its_pixels(self):
        painter, cyan, dark = rm.MazePainter(), rm.WALL_COLOUR, rm.BG
        self.assertEqual(painter.glyph((dark,) * 6), (" ", dark, dark))
        self.assertEqual(painter.glyph((cyan, dark) * 3), ("\u258c", cyan, dark))           # a wall on the left
        self.assertEqual(painter.glyph((cyan, cyan, dark, dark, dark, dark)), ("\U0001fb02", cyan, dark))
        self.assertEqual(painter.glyph((dark, dark, dark, dark, cyan, cyan)), ("\U0001fb2d", cyan, dark))
        glow = rm.add(dark, rm.WALL_GLOW)       # a third colour joins the nearer of the two
        self.assertEqual(painter.glyph((cyan, glow, cyan, dark, cyan, dark))[0], "\u258c")

    def test_splash_before_telemetry(self):
        for rows, cols in self.SIZES:
            for now in (0.0, 0.55, 7.3):    # animation frames
                with self.subTest(rows=rows, cols=cols, now=now):
                    view = self.view_with_log()
                    view.started = 0.0
                    scr, cursor = rm.render(rows, cols, rm.MazeModel(), rm.Overlay(), view, (0, 0), now)
                    self.check_frame(scr, cursor, rows, cols)

    def competition_model(self):
        checker = TranscriptChecker(self)
        lines = firmware_transcript(11, 40)
        checker.run(lines[:lines.index("#CHECK")])
        return checker.model, checker.planner

    def test_practice_maze_drawing(self):
        model, planner = practice_model()
        ov, painter, p = planner.overlay(model), rm.MazePainter(), 6
        px = painter.pixels(model, ov, rm.Effects(), (0, 0), (4, 3), (p, p), (1, 2), 0.0)
        base = painter.static(model, (4, 3), (p, p), (1, 2))
        self.assertEqual((len(px), len(px[0])), (20, 25))   # an even number of rows: two to a character
        # West and south sides are the real maze border: always solid.
        self.assertTrue(all(px[r][0] == rm.WALL_COLOUR for r in range(19)))
        self.assertTrue(all(c == rm.WALL_COLOUR for c in px[18]))
        self.assertEqual(painter.robot, (3, 15))            # back at the start (0,0)
        self.assertEqual(px[15][3], rm.WHITE)
        self.assertGreater(px[1][21][0], base[1][21][0] + 30)    # the goal (3,2), top right, glows
        # The verified fast path is drawn from the start to the goal, through every cell.
        self.assertEqual(ov.fast_path[-1], (3, 2))
        for x, y in ov.fast_path:
            self.assertNotEqual(px[(2 - y) * p + 3][x * p + 3], base[(2 - y) * p + 3][x * p + 3], (x, y))
        # Unvisited cells keep the background; visited ones get a floor.
        self.assertEqual(base[15][21], rm.BG)
        self.assertNotEqual(base[14][2], rm.BG)

    def test_drawn_robot_glides_and_runs_are_timed(self):
        model, planner = practice_model()
        fx = rm.Effects()
        route = planner.overlay(model).fast_path
        for t, line in ((0.0, "@P00N"), (1.0, "@AC"), (6.0, "@AF")):
            self.assertTrue(fx.line(model, planner, line, t, t))
        self.assertEqual(fx.path, [(0, 0)] + list(route))   # no pose until the end: drawn along the route
        fx.advance(6.0)
        fx.advance(6.4)
        self.assertNotEqual(fx.pos(), (0.0, 0.0))
        self.assertFalse(fx.line(model, planner, "route 5 cells, 1 curves: end=IR t=1804ms err=1mm", 7.9, 7.9))
        fx.line(model, planner, "@P32E", 7.9, 7.9)
        self.assertEqual(fx.path[-1], (3, 2))
        self.assertEqual(fx.timer(8.0)[1], "run")
        fx.line(model, planner, "Goal reached in the speed run after 1 legs", 8.2, 8.2)
        self.assertEqual(fx.timer(9.0), (1.804, "lap"))     # the robot's own time
        self.assertEqual(fx.banner[0], "GOAL")
        for wall in (8.3, 8.6, 9.0, 9.5):
            fx.advance(wall)
        self.assertEqual(fx.pos(), (3.0, 2.0))
        self.assertEqual(fx.ribbon[-1], (3, 2))
        fx.line(model, planner, "@P00N", 12.0, 12.0)         # carried back by hand: no glide through walls
        self.assertEqual((fx.path, fx.ribbon), ([(0, 0)], []))
        fx.line(model, planner, "@P02N", 13.0, 13.0)         # a straight leg of two cells
        self.assertEqual(fx.path, [(0, 0), (0, 1), (0, 2)])

    def test_effects_during_a_run_are_optional(self):
        calm, lively = rm.Effects(), rm.Effects()
        lively.lively = True
        pictures = []
        for fx in (calm, lively):
            model, planner = rm.MazeModel(), rm.Planner(rm.read_config())
            for t, line in ((0.0, "@P00N"), (0.1, "@AG"), (0.2, "@C00N.###"), (0.6, "@C01N.#.#")):
                fx.line(model, planner, line, t, t)
            painter = rm.MazePainter()
            ov = planner.overlay(model)
            pictures.append([painter.pixels(model, ov, fx, (0, 0), (16, 16), (12, 9), (2, 3), t) for t in (2.0, 2.3)])
        self.assertEqual((len(calm.ripples), len(lively.ripples)), (0, 2))
        post, goal = (9 * 9, 12 * 5), (9 * 8 - 4, 12 * 7 + 6)      # (row, column): an unseen post, inside the goal
        for (r, c), name in ((post, "sweep"), (goal, "goal")):
            self.assertEqual(pictures[0][0][r][c], pictures[0][1][r][c], name)       # still during the run
            self.assertNotEqual(pictures[1][0][r][c], pictures[1][1][r][c], name)

    def test_keys_and_colours(self):
        self.assertEqual(rm.parse_keys(b"ab\r\x1b[A\x1b[6~\x18\x7f\x1bOF\x1b[99;5u\x00\x1b"),
                         ["a", "b", "enter", "up", "pgdn", "^X", "backspace", "end", "esc"])
        self.assertEqual([rm.colour_256(c) for c in ((0, 0, 0), (255, 255, 255), (255, 0, 0), (128, 128, 128))],
                         [16, 231, 196, 244])

    def test_log_scroll_and_help(self):
        model, planner = practice_model()
        view = self.view_with_log()
        view.scroll = 50
        cv, _ = rm.render(30, 120, model, planner.overlay(model), view, (0, 0), time.monotonic())
        screen = "\n".join(cv.row_text(y) for y in range(30))
        self.assertIn("50 more lines below", screen)
        view.scroll = 10 ** 6       # clamped, never out of range
        rm.render(30, 120, model, planner.overlay(model), view, (0, 0), time.monotonic())
        self.assertLessEqual(view.scroll, len(view.log))


class TestCalibrationCapture(unittest.TestCase):
    DUMP = ["@D BEGIN straight 1 150", '@D INFO period_ms=5 samples=2 result=OK build="Sep 22 2026"',
            "@D INFO ticks_per_mm=9", "@D COLS t_ms,enc_l,enc_r", "@D 0,0,0", "@D 5,12,11",
            "@D END result=OK samples=2"]

    def test_dump_saved_as_csv_with_notes(self):
        with tempfile.TemporaryDirectory() as directory:
            capture = rm.CalibrationCapture(directory)
            messages = [capture.feed(line) for line in self.DUMP]
            self.assertEqual(messages[0][0], "info")
            self.assertEqual(messages[-1][0], "ok")
            self.assertIsNone(capture.progress())
            self.assertEqual(capture.note("measured 181 mm")[0], "ok")
            with open(capture.last_path, encoding="utf-8") as f:
                content = f.read()
            self.assertTrue(capture.last_path.endswith("_straight.csv"))
            self.assertIn("# test: straight 1 150\n", content)
            self.assertIn("# ticks_per_mm=9\n", content)
            header, data = content.split("t_ms,enc_l,enc_r\n")
            self.assertEqual(data, "0,0,0\n5,12,11\n")
            self.assertTrue(all(line.startswith("#") for line in header.splitlines()))
            self.assertTrue(header.endswith("# note: measured 181 mm\n"), header)

    def test_two_dumps_in_the_same_second_keep_both_files(self):
        with tempfile.TemporaryDirectory() as directory:
            capture = rm.CalibrationCapture(directory)
            paths = []
            for _ in range(3):
                for line in self.DUMP:
                    capture.feed(line)
                paths.append(capture.last_path)
            self.assertEqual(len(set(paths)), 3)
            self.assertEqual(len(os.listdir(directory)), 3)

    def test_progress_and_stray_lines(self):
        with tempfile.TemporaryDirectory() as directory:
            capture = rm.CalibrationCapture(directory)
            self.assertIsNone(capture.feed("@D 1,2,3"))
            capture.feed(self.DUMP[0])
            capture.feed(self.DUMP[4])
            self.assertEqual(capture.progress(), ("straight 1 150", 1))
            self.assertEqual(capture.note("x")[0], "warn")
            self.assertEqual(os.listdir(directory), [])

    def test_replay_does_not_write_files(self):
        with tempfile.TemporaryDirectory() as directory:
            capture = rm.CalibrationCapture(directory, save=False)
            for line in self.DUMP:
                capture.feed(line)
            self.assertEqual(os.listdir(directory), [])


class TestTerminal(unittest.TestCase):
    """Runs the real program in a pseudo-terminal."""

    def run_monitor(self, args, rows, cols, keys=b"", wait=2.0, resize=None):
        pid, fd = pty.fork()
        if pid == 0:
            try:
                fcntl.ioctl(0, termios.TIOCSWINSZ, struct.pack("HHHH", rows, cols, 0, 0))
                os.environ.update(TERM="xterm-256color", LANG="en_US.UTF-8", LC_ALL="en_US.UTF-8")
                os.execv(sys.executable, [sys.executable, MONITOR] + args)
            finally:
                os._exit(127)
        output = bytearray()

        def pump(seconds):
            end = time.time() + seconds
            while time.time() < end:
                ready, _, _ = select.select([fd], [], [], 0.05)
                if ready:
                    try:
                        output.extend(os.read(fd, 65536))
                    except OSError:
                        return

        pump(wait)
        if resize:
            fcntl.ioctl(fd, termios.TIOCSWINSZ, struct.pack("HHHH", resize[0], resize[1], 0, 0))
            os.kill(pid, signal.SIGWINCH)
            pump(0.6)
        for key in keys:
            os.write(fd, bytes([key]))
            pump(0.05)
        os.write(fd, b"\x1b")
        status = None
        end = time.time() + 5
        while time.time() < end:
            pump(0.1)
            done, status = os.waitpid(pid, os.WNOHANG)
            if done:
                break
        else:
            os.kill(pid, signal.SIGKILL)
            os.waitpid(pid, 0)
            self.fail("the monitor did not quit on Esc")
        os.close(fd)
        text = re.sub(r"\x1b\[[0-9;?]*[A-Za-z]", "", output.decode("utf-8", "replace"))    # without the escape codes
        self.assertNotIn("Traceback", text)
        self.assertTrue(os.WIFEXITED(status) and os.WEXITSTATUS(status) == 0, text[-2000:])
        return text

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.replay = os.path.join(self.tmp.name, "transcript.txt")
        with open(self.replay, "w", encoding="utf-8") as f:
            f.write("\n".join(firmware_transcript(5, 0, practice=True)) + "\n")
        self.args = ["--replay", self.replay, "--speed", "40", "--calib-dir", self.tmp.name]

    def tearDown(self):
        self.tmp.cleanup()

    def test_replay_draws_and_quits(self):
        text = self.run_monitor(self.args, 40, 120)
        self.assertIn("RAT", text)
        self.assertIn("replaying", text)

    def test_small_terminal_keys_and_resize(self):
        keys = b"STATUS\r\t\t/full\r/pixels\r/effects\r/clear\r\x1b[A\x1b[5~"
        self.run_monitor(self.args, 20, 60, keys=keys, resize=(12, 44))

    def test_too_small_terminal(self):
        text = self.run_monitor(self.args, 8, 30)
        self.assertIn("Terminal too small", text)


if __name__ == "__main__":
    unittest.main()
