#!/usr/bin/env python3
"""Live monitor + command console for the rat robot firmware (main.c).

Shows the robot's Bluetooth output (run log, maps, replies) in a scrolling
panel and sends whatever you type as a command, no reflash needed. The most
used ones (HELP on the robot lists them all):
    MODE n       1 search, 2 speed run, 3/4 left/right wall follower,
                 5 sensor monitor, 6 erase map
    START / STOP launch / stop the selected mode
    STATUS, MAP  state and parameters / ASCII map with the speed-run path
    SPD n, FAST n, TURN n, KP f, KD f, KE f   live tuning (SAVE keeps them)

Usage:
    python3 tools/robot_monitor.py [--port /dev/rfcomm0] [--baud 9600]
Type a command, press Enter to send it. Press ESC to quit.
"""
import argparse
import curses
import locale
import threading
import time
from collections import deque

import serial

P_TITLE, P_OK, P_BAD, P_WARN, P_HEAD, P_DIM, P_SENT = range(1, 8)

LOG_MAXLEN = 500


class MonitorState:
    def __init__(self):
        self.lock = threading.Lock()
        self.log = deque(maxlen=LOG_MAXLEN)
        self.connected = False
        self.last_error = ""
        self.serial_obj = None


def reader_thread(port, baud, state):
    while True:
        try:
            with serial.Serial(port, baud, timeout=0.1) as ser:
                with state.lock:
                    state.serial_obj = ser
                    state.connected = True
                    state.last_error = ""
                    state.log.append(("info", "-- conectado --"))
                buf = b""
                while True:
                    # ser.readline()'s default byte-at-a-time scan is slow; read whatever
                    # is already buffered (or wait up to timeout for the first byte) and
                    # split lines ourselves so new output shows up as soon as it arrives.
                    chunk = ser.read(max(1, ser.in_waiting))
                    if not chunk:
                        continue
                    buf += chunk
                    while b"\n" in buf:
                        raw_line, buf = buf.split(b"\n", 1)
                        line = raw_line.decode("ascii", errors="ignore").rstrip("\r")
                        if line == "":
                            continue
                        with state.lock:
                            state.log.append(("rx", line))
        except serial.SerialException as exc:
            with state.lock:
                state.connected = False
                state.serial_obj = None
                state.last_error = str(exc)
                state.log.append(("bad", "-- desconectado: %s --" % exc))
            time.sleep(1.0)


def send_command(state, cmd):
    with state.lock:
        ser = state.serial_obj
        connected = state.connected
        if ser is not None and connected:
            try:
                ser.write((cmd + "\n").encode("ascii"))
                state.log.append(("sent", "> " + cmd))
                return True
            except serial.SerialException as exc:
                state.log.append(("bad", "-- error enviando: %s --" % exc))
                return False
        else:
            state.log.append(("bad", "-- no conectado, comando descartado --"))
            return False


def init_colors():
    curses.start_color()
    curses.use_default_colors()
    curses.init_pair(P_TITLE, curses.COLOR_MAGENTA, -1)
    curses.init_pair(P_OK, curses.COLOR_GREEN, -1)
    curses.init_pair(P_BAD, curses.COLOR_RED, -1)
    curses.init_pair(P_WARN, curses.COLOR_YELLOW, -1)
    curses.init_pair(P_HEAD, curses.COLOR_CYAN, -1)
    curses.init_pair(P_DIM, curses.COLOR_WHITE, -1)
    curses.init_pair(P_SENT, curses.COLOR_YELLOW, -1)


def line_attr(kind, text):
    if kind == "sent":
        return curses.color_pair(P_SENT) | curses.A_BOLD
    if kind == "bad" or text.startswith("!!"):
        return curses.color_pair(P_BAD) | curses.A_BOLD
    if kind == "info" or "Meta alcanzada" in text or text.startswith("Fin: OK"):
        return curses.color_pair(P_OK) | curses.A_BOLD
    if text.startswith("==") or text.startswith("Fin:"):
        return curses.color_pair(P_TITLE) | curses.A_BOLD
    return curses.A_NORMAL  # plain terminal default, always visible regardless of theme


def draw(stdscr, state, port):
    locale.setlocale(locale.LC_ALL, "")
    init_colors()
    curses.curs_set(1)
    stdscr.timeout(50)  # redraw ~20x/sec regardless of typing, so new robot output shows up promptly

    input_buf = ""

    while True:
        try:
            key = stdscr.getch()
        except curses.error:
            key = -1

        if key == 27:  # ESC
            return
        elif key in (curses.KEY_ENTER, 10, 13):
            cmd = input_buf.strip()
            if cmd:
                send_command(state, cmd)
            input_buf = ""
        elif key in (curses.KEY_BACKSPACE, 127, 8):
            input_buf = input_buf[:-1]
        elif 32 <= key < 127:
            if len(input_buf) < 60:
                input_buf += chr(key)

        max_y, max_x = stdscr.getmaxyx()
        width = max(60, min(max_x - 2, 120))
        height = max(20, min(max_y - 1, 40))

        with state.lock:
            log_snapshot = list(state.log)[-(height - 6):]
            connected = state.connected
            last_error = state.last_error

        stdscr.erase()
        try:
            stdscr.addstr(0, 0, "\u2554" + "\u2550" * (width - 2) + "\u2557", curses.color_pair(P_HEAD))
            title = " RAT ROBOT \u2014 MONITOR + CONTROL "
            stdscr.addstr(1, 0, "\u2551", curses.color_pair(P_HEAD))
            stdscr.addstr(1, (width - len(title)) // 2, title, curses.color_pair(P_TITLE) | curses.A_BOLD)
            stdscr.addstr(1, width - 1, "\u2551", curses.color_pair(P_HEAD))

            status_attr = curses.color_pair(P_OK) | curses.A_BOLD if connected else curses.color_pair(P_BAD) | curses.A_BOLD
            stdscr.addstr(2, 2, "Port: %s" % port)
            stdscr.addstr(2, 26, "\u25cf", status_attr)
            stdscr.addstr(2, 28, "CONNECTED" if connected else "DISCONNECTED", status_attr)
            if not connected and last_error:
                stdscr.addstr(2, 42, last_error[: width - 44], curses.color_pair(P_BAD))

            stdscr.addstr(3, 0, "\u2560" + "\u2550" * (width - 2) + "\u2563", curses.color_pair(P_HEAD))

            for i, (kind, line) in enumerate(log_snapshot):
                row = 4 + i
                if row >= height - 3:
                    break
                stdscr.addstr(row, 2, line[: width - 4], line_attr(kind, line))

            stdscr.addstr(height - 3, 0, "\u2560" + "\u2550" * (width - 2) + "\u2563", curses.color_pair(P_HEAD))
            stdscr.addstr(height - 2, 2, "> " + input_buf, curses.color_pair(P_WARN) | curses.A_BOLD)
            stdscr.addstr(height - 1, 0, "\u255a" + "\u2550" * (width - 2) + "\u255d", curses.color_pair(P_HEAD))
            stdscr.addstr(
                height, 0,
                "Enter: enviar   ESC: salir",
                curses.color_pair(P_DIM),
            )

            cheatsheet = [
                ("MODE n", "1 busqueda  2 rapida  3/4 seguidor izq/der  5 sensores  6 borrar"),
                ("START/STOP", "lanza / detiene el modo (como el boton START)"),
                ("PAUSE/RESUME", "frena y espera / continua"),
                ("STEP ON|OFF", "pausa tras cada accion (RESUME para seguir)"),
                ("STATUS", "modo, posicion, parametros y camino rapido"),
                ("MAP", "mapa ASCII con el camino rapido (robot parado)"),
                ("IR / WALLS", "lectura de sensores / paredes detectadas ahora"),
                ("SPD FAST TURN", "PWM de busqueda / crucero rapido / giro"),
                ("KP KD KE f", "centrado P / D y mantener rumbo sin paredes"),
                ("SAVE / ERASE", "guarda mapa+meta+parametros / borra el mapa"),
                ("HOME", "el robot esta en la salida mirando al norte"),
                ("HELP", "lista completa de comandos"),
            ]
            cheat_row = height + 2
            if cheat_row < max_y - 1:
                stdscr.addstr(cheat_row, 0, "\u2554" + "\u2550" * (width - 2) + "\u2557", curses.color_pair(P_HEAD))
                cheat_title = " COMANDOS "
                stdscr.addstr(cheat_row + 1, 0, "\u2551", curses.color_pair(P_HEAD))
                stdscr.addstr(cheat_row + 1, (width - len(cheat_title)) // 2, cheat_title, curses.color_pair(P_TITLE) | curses.A_BOLD)
                stdscr.addstr(cheat_row + 1, width - 1, "\u2551", curses.color_pair(P_HEAD))
                stdscr.addstr(cheat_row + 2, 0, "\u2560" + "\u2550" * (width - 2) + "\u2563", curses.color_pair(P_HEAD))
                for i, (cmd, desc) in enumerate(cheatsheet):
                    row = cheat_row + 3 + i
                    if row >= max_y - 1:
                        break
                    stdscr.addstr(row, 2, "%-14s" % cmd, curses.color_pair(P_WARN) | curses.A_BOLD)
                    stdscr.addstr(row, 17, desc[: width - 19], curses.color_pair(P_DIM))
                last_row = min(cheat_row + 3 + len(cheatsheet), max_y - 1)
                stdscr.addstr(last_row, 0, "\u255a" + "\u2550" * (width - 2) + "\u255d", curses.color_pair(P_HEAD))

            stdscr.move(height - 2, 4 + len(input_buf))
        except curses.error:
            pass

        stdscr.refresh()


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", default="/dev/rfcomm0", help="Serial device (default: /dev/rfcomm0)")
    parser.add_argument("--baud", type=int, default=9600)
    args = parser.parse_args()

    state = MonitorState()
    t = threading.Thread(target=reader_thread, args=(args.port, args.baud, state), daemon=True)
    t.start()

    curses.wrapper(draw, state, args.port)


if __name__ == "__main__":
    main()
