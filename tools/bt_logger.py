"""Bluetooth logger for driving the robot from scripts (tools/robot.sh).

Holds /dev/rfcomm0 open, records the session to tools/logs/ as
robot_monitor.py does, saves @D dumps as CSV in tools/calib_data/, and sends
to the robot every line written to the FIFO $RAT_BT_DIR/bt_in (default
/tmp/rat_bt). Reconnects by itself when the link drops. Only one program may
hold the port: close robot_monitor.py first. `/nota <text>` lines annotate
the last dump."""
import os
import select
import stat
import sys
import time

import serial

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, "tools"))
import robot_monitor as rm  # noqa: E402

HERE = os.environ.get("RAT_BT_DIR", "/tmp/rat_bt")
os.makedirs(HERE, exist_ok=True)
FIFO = os.path.join(HERE, "bt_in")
if os.path.exists(FIFO) and not stat.S_ISFIFO(os.stat(FIFO).st_mode):
    os.remove(FIFO)
if not os.path.exists(FIFO):
    os.mkfifo(FIFO)

stamp = time.strftime("%Y-%m-%d_%H-%M-%S")
log_path = os.path.join(REPO, "tools", "logs", stamp + ".log")
log = open(log_path, "w", encoding="utf-8", buffering=1)
with open(os.path.join(HERE, "bt_log_path"), "w") as f:
    f.write(log_path + "\n")
t0 = time.monotonic()
capture = rm.CalibrationCapture(os.path.join(REPO, "tools", "calib_data"))


def record(direction, text):
    log.write("%.3f\t%s\t%s\n" % (time.monotonic() - t0, direction, text))


def open_port():
    while True:
        try:
            p = serial.Serial("/dev/rfcomm0", 9600, timeout=0)
            record("!", "conectado")
            return p
        except (serial.SerialException, OSError) as exc:
            record("!", "sin conexion: %s" % exc)
            time.sleep(1.0)


port = open_port()
fifo = os.open(FIFO, os.O_RDWR | os.O_NONBLOCK)
rx, cmd = b"", b""
while True:
    ready, _, _ = select.select([port.fileno(), fifo], [], [], 0.5)
    if port.fileno() in ready:
        try:
            rx += port.read(4096)
        except (serial.SerialException, OSError):
            record("!", "desconectado")
            port.close()
            time.sleep(1.0)
            port = open_port()
            continue
        while b"\n" in rx:
            raw, rx = rx.split(b"\n", 1)
            text = raw.decode("utf-8", "replace").rstrip("\r")
            record("<", text)
            if text.startswith("@D"):
                result = capture.feed(text)
                if result and result[0] in ("ok", "bad"):
                    record("!", result[1])
    if fifo in ready:
        cmd += os.read(fifo, 4096)
        while b"\n" in cmd:
            line, cmd = cmd.split(b"\n", 1)
            text = line.decode().strip()
            if not text:
                continue
            if text.startswith("/nota "):
                record("!", capture.note(text[6:])[1])
                continue
            try:
                port.write((text + "\n").encode())
                record(">", text)
            except (serial.SerialException, OSError):
                record("!", "no enviado (desconectado): " + text)
            time.sleep(0.05)
