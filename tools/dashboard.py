#!/usr/bin/env python3
"""Live terminal dashboard for the rat robot: LEDs, IR sensors and encoders.

Reads "DATA,l1,l2,l3,l4,l5,l6,sl,fl,fr,sr,enc_l,enc_r" lines produced by the
diag_test firmware (src/test_diag.c) over a serial/RFCOMM link and renders
them live in a colored terminal panel.

Usage:
    python3 tools/dashboard.py [--port /dev/rfcomm0] [--baud 9600]
Press 'q' to quit.
"""
import argparse
import curses
import locale
import os
import subprocess
import threading
import time

import serial

FIELDS = ["l1", "l2", "l3", "l4", "l5", "l6", "sl", "fl", "fr", "sr", "enc_l", "enc_r"]
LED_KEYS = ["l1", "l2", "l3", "l4", "l5", "l6"]
IR_ROWS = [("SL", "sl"), ("FL", "fl"), ("FR", "fr"), ("SR", "sr")]

# Paired HC-05 on the robot (see `bluetoothctl paired-devices`); SPP channel found via `sdptool records`.
HC05_MAC = "20:13:09:11:04:32"
HC05_CHANNEL = 1

# Color pair ids (assigned in init_colors)
P_TITLE, P_OK, P_BAD, P_WARN, P_HEAD, P_DIM, P_VALUE = range(1, 8)


def _port_alive(port, baud, timeout=2.0):
    """Try to actually read a byte from `port` — the only reliable liveness check.

    BlueZ can report the HC-05 as "Connected" and /dev/rfcommN can exist while the
    RFCOMM/TTY session itself is dead (e.g. after a dropped link): reads then fail
    immediately with SerialException "device reports readiness to read but returned
    no data" instead of timing out cleanly. Path/connected-state checks alone miss this.
    """
    try:
        with serial.Serial(port, baud, timeout=timeout) as ser:
            return bool(ser.read(1))
    except serial.SerialException:
        return False


def ensure_rfcomm_bound(port, mac, channel, baud):
    """(Re)bind an rfcomm device node so data actually flows, not just os.path.exists().

    Releasing + rebinding forces a fresh connect attempt instead of reusing a wedged
    RFCOMM session left over from a dropped link. Requires sudo — will prompt for a
    password on the real terminal since this runs before curses takes over the screen.
    """
    if not port.startswith("/dev/rfcomm") or _port_alive(port, baud):
        return
    rfcomm_id = port[len("/dev/rfcomm"):]
    print("%s has no live data from %s, (re)binding (channel %d)..." % (port, mac, channel))
    subprocess.run(["sudo", "rfcomm", "release", rfcomm_id], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        subprocess.run(["sudo", "rfcomm", "bind", rfcomm_id, mac, str(channel)], check=True)
    except FileNotFoundError:
        print("rfcomm command not found (bluez-utils not installed?); skipping auto-bind.")
        return
    except subprocess.CalledProcessError as exc:
        print("rfcomm bind failed (%s); is the HC-05 powered on and in range?" % exc)
        return
    for _ in range(20):
        if os.path.exists(port):
            break
        time.sleep(0.1)
    if not _port_alive(port, baud, timeout=3.0):
        print("Warning: %s bound but still no data — check the HC-05 is powered on and in range." % port)


class RobotState:
    def __init__(self):
        self.lock = threading.Lock()
        self.values = None
        self.last_update = 0.0
        self.line_count = 0
        self.connected = False
        self.last_error = ""
        self.start_time = time.time()


def reader_thread(port, baud, state):
    while True:
        try:
            with serial.Serial(port, baud, timeout=1) as ser:
                with state.lock:
                    state.connected = True
                    state.last_error = ""
                while True:
                    raw = ser.readline().decode("ascii", errors="ignore").strip()
                    if not raw.startswith("DATA,"):
                        continue
                    parts = raw.split(",")[1:]
                    if len(parts) != len(FIELDS):
                        continue
                    try:
                        values = [int(p) for p in parts]
                    except ValueError:
                        continue
                    with state.lock:
                        state.values = dict(zip(FIELDS, values))
                        state.last_update = time.time()
                        state.line_count += 1
        except serial.SerialException as exc:
            with state.lock:
                state.connected = False
                state.last_error = str(exc)
            time.sleep(1.0)


def init_colors():
    curses.start_color()
    curses.use_default_colors()
    curses.init_pair(P_TITLE, curses.COLOR_MAGENTA, -1)
    curses.init_pair(P_OK, curses.COLOR_GREEN, -1)
    curses.init_pair(P_BAD, curses.COLOR_RED, -1)
    curses.init_pair(P_WARN, curses.COLOR_YELLOW, -1)
    curses.init_pair(P_HEAD, curses.COLOR_CYAN, -1)
    curses.init_pair(P_DIM, curses.COLOR_WHITE, -1)
    curses.init_pair(P_VALUE, curses.COLOR_WHITE, -1)


def proximity(mm):
    if mm < 100:
        return curses.color_pair(P_BAD) | curses.A_BOLD, "NEAR!"
    if mm < 200:
        return curses.color_pair(P_WARN), "near "
    return curses.color_pair(P_OK), "clear"


def bar(value_mm, width, max_mm=400):
    value_mm = max(0, min(value_mm, max_mm))
    filled = int(width * (1 - value_mm / max_mm))
    return "\u2588" * filled + "\u2591" * (width - filled)


MIN_WIDTH, MAX_WIDTH = 76, 120
MIN_HEIGHT = 24


def box_line(stdscr, y, width, left, mid, right, fill="\u2500"):
    stdscr.addstr(y, 0, left + fill * (width - 2) + right, curses.color_pair(P_HEAD))


def draw(stdscr, state, port):
    locale.setlocale(locale.LC_ALL, "")
    init_colors()
    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.timeout(100)

    while True:
        key = stdscr.getch()
        if key in (ord("q"), ord("Q")):
            return

        with state.lock:
            values = state.values
            last_update = state.last_update
            line_count = state.line_count
            connected = state.connected
            last_error = state.last_error

        max_y, max_x = stdscr.getmaxyx()
        width = max(MIN_WIDTH, min(max_x - 2, MAX_WIDTH))
        bar_width = width - 46

        stdscr.erase()
        if max_y < MIN_HEIGHT or max_x < MIN_WIDTH + 2:
            try:
                stdscr.addstr(0, 0, "Terminal too small, please enlarge it", curses.color_pair(P_WARN))
                stdscr.addstr(1, 0, "(need at least %dx%d)" % (MIN_WIDTH + 2, MIN_HEIGHT))
            except curses.error:
                pass
            stdscr.refresh()
            continue

        try:
            box_line(stdscr, 0, width, "\u2554", "\u2566", "\u2557", "\u2550")
            title = " RAT ROBOT \u2014 LIVE PANEL "
            stdscr.addstr(1, 0, "\u2551", curses.color_pair(P_HEAD))
            stdscr.addstr(1, (width - len(title)) // 2, title, curses.color_pair(P_TITLE) | curses.A_BOLD)
            stdscr.addstr(1, width - 1, "\u2551", curses.color_pair(P_HEAD))
            box_line(stdscr, 2, width, "\u2560", "\u256c", "\u2563", "\u2550")

            stdscr.addstr(4, 2, "Port: %s" % port)
            status_attr = curses.color_pair(P_OK) | curses.A_BOLD if connected else curses.color_pair(P_BAD) | curses.A_BOLD
            stdscr.addstr(4, 28, "\u25cf", status_attr)
            stdscr.addstr(4, 30, "CONNECTED" if connected else "DISCONNECTED", status_attr)
            stdscr.addstr(5, 2, "Updates: %d" % line_count, curses.color_pair(P_DIM))
            if not connected and last_error:
                stdscr.addstr(5, 24, last_error[: width - 26], curses.color_pair(P_BAD))
            stdscr.addstr(6, 2, "Motors: ", curses.color_pair(P_DIM))
            stdscr.addstr(6, 10, "OFF", curses.color_pair(P_BAD) | curses.A_BOLD)
            stdscr.addstr(6, 14, "(diag_test never drives them)", curses.color_pair(P_DIM))

            box_line(stdscr, 8, width, "\u2560", "\u256c", "\u2563", "\u2500")

            if values is None:
                elapsed = time.time() - state.start_time
                waiting_msg = "Waiting for data... (%ds)" % elapsed
                stdscr.addstr(11, 2, waiting_msg, curses.color_pair(P_WARN))
                if connected:
                    stdscr.addstr(12, 2, "Port is open but no valid DATA line yet — Bluetooth link may still be (re)connecting.", curses.color_pair(P_DIM))
                else:
                    stdscr.addstr(12, 2, "Port not open. Check: rfcomm bind still alive? HC-05 already connected to something else (e.g. the phone app)?", curses.color_pair(P_DIM))
            else:
                age = time.time() - last_update
                stale = age > 1.0

                stdscr.addstr(10, 2, "LEDS", curses.color_pair(P_HEAD) | curses.A_BOLD)
                x = 4
                for label, k in zip(["L1", "L2", "L3", "L4", "L5", "L6"], LED_KEYS):
                    on = values[k]
                    attr = curses.color_pair(P_OK) | curses.A_BOLD if on else curses.color_pair(P_DIM)
                    stdscr.addstr(12, x, "%s[" % label)
                    stdscr.addstr(12, x + 3, "\u25cf" if on else "\u25cb", attr)
                    stdscr.addstr(12, x + 4, "]")
                    x += 9

                box_line(stdscr, 14, width, "\u2560", "\u256c", "\u2563", "\u2500")
                stdscr.addstr(16, 2, "IR SENSORS (mm)", curses.color_pair(P_HEAD) | curses.A_BOLD)
                for i, (label, k) in enumerate(IR_ROWS):
                    mm = values[k]
                    attr, tag = proximity(mm)
                    row = 18 + i * 2
                    stdscr.addstr(row, 2, "%s" % label, curses.color_pair(P_VALUE) | curses.A_BOLD)
                    stdscr.addstr(row, 5, "%4d mm" % mm, curses.color_pair(P_VALUE))
                    stdscr.addstr(row, 13, "[%s]" % bar(mm, bar_width), attr)
                    stdscr.addstr(row, 16 + bar_width, tag, attr | curses.A_BOLD)

                box_line(stdscr, 27, width, "\u2560", "\u256c", "\u2563", "\u2500")
                stdscr.addstr(29, 2, "ENCODERS (raw ticks, 0-65535)", curses.color_pair(P_HEAD) | curses.A_BOLD)
                stdscr.addstr(31, 2, "L:", curses.A_BOLD)
                stdscr.addstr(31, 5, "%6d" % values["enc_l"], curses.color_pair(P_VALUE) | curses.A_BOLD)
                stdscr.addstr(31, 20, "R:", curses.A_BOLD)
                stdscr.addstr(31, 23, "%6d" % values["enc_r"], curses.color_pair(P_VALUE) | curses.A_BOLD)
                stdscr.addstr(31, 34, "(spin a wheel by hand, values should change)", curses.color_pair(P_DIM))

                if stale:
                    stdscr.addstr(33, 2, "(no new data for %.1fs)" % age, curses.color_pair(P_WARN))

            box_line(stdscr, 35, width, "\u255a", "\u2569", "\u255d", "\u2550")
            stdscr.addstr(36, 2, "q", curses.color_pair(P_WARN) | curses.A_BOLD)
            stdscr.addstr(36, 3, ": quit", curses.color_pair(P_DIM))
        except curses.error:
            pass  # terminal too small for a frame; skip until it's resized

        stdscr.refresh()


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", default="/dev/rfcomm0", help="Serial device (default: /dev/rfcomm0)")
    parser.add_argument("--baud", type=int, default=9600)
    parser.add_argument("--mac", default=HC05_MAC, help="HC-05 Bluetooth MAC to bind --port to if missing")
    parser.add_argument("--channel", type=int, default=HC05_CHANNEL, help="RFCOMM SPP channel (default: %(default)s)")
    parser.add_argument("--no-bind", action="store_true", help="Don't try to auto-bind --port via rfcomm")
    args = parser.parse_args()

    if not args.no_bind:
        ensure_rfcomm_bound(args.port, args.mac, args.channel, args.baud)

    state = RobotState()
    t = threading.Thread(target=reader_thread, args=(args.port, args.baud, state), daemon=True)
    t.start()

    curses.wrapper(draw, state, args.port)


if __name__ == "__main__":
    main()

