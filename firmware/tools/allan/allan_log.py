"""
Record the ESP32's USB data stream for Allan variance analysis.

    python allan_log.py --hours 2                  (finds the ESP32's USB port by itself)
    python allan_log.py --port COM4 --hours 2      (Windows, a specific port)
    python allan_log.py --port /dev/ttyUSB0        (Linux; macOS: /dev/cu.usbserial-* or /dev/cu.SLAB_USBtoUART)

Setup once: python -m venv .venv, then .venv/bin/pip install -r requirements.txt
(Windows: .venv\\Scripts\\pip). On Linux, add yourself to the serial group once:
sudo usermod -aG dialout $USER (log out and back in).

- Opens the port without resetting the ESP32.
- Keeps the computer awake while recording (and only while recording).
- Writes logs/allan_<date>.csv with one row per $I line (100 Hz), plus a .status file
  updated every minute. Create a file named STOP in the logs folder to end early;
  everything recorded so far stays usable.
"""
import argparse
import ctypes
import datetime as dt
import os
import subprocess
import sys
import time

import serial
from serial.tools import list_ports

ap = argparse.ArgumentParser()
ap.add_argument("--port", default="", help="serial port; leave out to find the ESP32 automatically")
ap.add_argument("--hours", type=float, default=2.0)
ap.add_argument("--out", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs"))
args = ap.parse_args()

# USB-serial chips found on ESP32 boards: CP210x, CH340, Espressif native USB, FTDI
ESP_USB_VIDS = {0x10C4, 0x1A86, 0x303A, 0x0403}
if not args.port:
    found = [p for p in list_ports.comports() if p.vid in ESP_USB_VIDS]
    if len(found) == 1:
        args.port = found[0].device
    else:
        ports = "\n".join(f"  {p.device}  {p.description}" for p in list_ports.comports()) or "  (none)"
        sys.exit(("Several possible ESP32 ports" if found else "No ESP32 found on USB") +
                 f". Pick one with --port:\n{ports}")

os.makedirs(args.out, exist_ok=True)
stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
csv_path = os.path.join(args.out, f"allan_{stamp}.csv")
status_path = csv_path.replace(".csv", ".status.txt")
other_path = csv_path.replace(".csv", ".esp32.log")
stop_path = os.path.join(args.out, "STOP")
if os.path.exists(stop_path):
    os.remove(stop_path)

# Keep the computer from sleeping while we record.
ES_CONTINUOUS, ES_SYSTEM_REQUIRED = 0x80000000, 0x00000001
awake = None
if sys.platform == "win32":
    ctypes.windll.kernel32.SetThreadExecutionState(ES_CONTINUOUS | ES_SYSTEM_REQUIRED)
else:
    cmd = (["caffeinate", "-i", "-w", str(os.getpid())] if sys.platform == "darwin" else
           ["systemd-inhibit", "--what=sleep:idle", "--who=allan_log", "--why=Recording IMU data", "sleep", "infinity"])
    try:
        awake = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except OSError:
        print("Note: couldn't stop the computer from sleeping; turn sleep off yourself for long recordings.")

ser = serial.Serial()
ser.port, ser.baudrate, ser.timeout = args.port, 115200, 0.5
ser.dtr = False   # don't reset the board
ser.rts = False
try:
    ser.open()
except Exception as e:
    print(f"Could not open {args.port}: {e}\nClose idf.py monitor and the lab's USB connection, then try again.")
    sys.exit(1)

duration = args.hours * 3600
start = time.time()
next_status = start + 60
rows = bad = gaps = 0
last_t = None
win_g = [0.0, 0.0, 0.0]
win_n = 0
temp = float("nan")
buf = b""


def write_status(final=False):
    el = time.time() - start
    mean = [g / win_n for g in win_g] if win_n else [float("nan")] * 3
    txt = (f"{'FINISHED' if final else 'RECORDING'}  {el/60:6.1f} / {duration/60:.0f} min\n"
           f"samples {rows}  ({rows/max(el,1):.1f} Hz)  gaps {gaps}  bad lines {bad}\n"
           f"temperature {temp:.2f} C\n"
           f"gyro mean last minute (deg/s): {mean[0]:+.4f} {mean[1]:+.4f} {mean[2]:+.4f}\n"
           f"file {csv_path}\n")
    with open(status_path, "w") as f:
        f.write(txt)
    print(txt.replace("\n", " | "), flush=True)


print(f"Recording {args.hours:g} h from {args.port} into {csv_path}", flush=True)
with open(csv_path, "w", newline="", encoding="utf-8") as out, \
        open(other_path, "w", encoding="utf-8", errors="replace") as other:
    out.write("t_ms,ax,ay,az,gx,gy,gz,mx,my,mz,temp\n")
    try:
        while time.time() - start < duration and not os.path.exists(stop_path):
            buf += ser.read(4096)
            *lines, buf = buf.split(b"\n")
            for raw in lines:
                line = raw.decode(errors="replace").strip()
                if not line.startswith("$I,"):
                    if line:
                        try:
                            other.write(f"{time.time()-start:9.1f}  {line}\n")
                        except Exception:
                            pass    # a garbled side-log line must never stop the recording
                    continue
                f = line[3:].split(",")
                try:
                    v = [float(x) for x in f]
                except ValueError:
                    bad += 1
                    continue
                if len(v) != 11:
                    bad += 1
                    continue
                t = int(v[0])
                if last_t is not None and not (5 <= t - last_t <= 15):
                    gaps += 1
                last_t = t
                out.write(line[3:] + "\n")
                rows += 1
                win_g = [win_g[i] + v[4 + i] for i in range(3)]
                win_n += 1
                temp = v[10]
            if time.time() >= next_status:
                out.flush()
                other.flush()
                write_status()
                win_g, win_n = [0.0, 0.0, 0.0], 0
                next_status += 60
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
        if sys.platform == "win32":
            ctypes.windll.kernel32.SetThreadExecutionState(ES_CONTINUOUS)
        elif awake:
            awake.terminate()
        write_status(final=True)
