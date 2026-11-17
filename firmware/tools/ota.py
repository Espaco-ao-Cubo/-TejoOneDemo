"""
Send new firmware to the ESP32 over Wi-Fi.

    idf.py build
    python tools/ota.py            (from the firmware folder, in an ESP-IDF terminal or any
                                    Python 3 on Windows, Linux or macOS)

Your computer must be on the same network as the board (its IMU-Lab access point, or your
home Wi-Fi if you set one in menuconfig). Uses only the Python standard library.

Options: --host 192.168.4.1 (or imu-lab.local)   --bin build/blink.bin   --password ...
"""
import argparse
import http.client
import json
import os
import struct
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ap = argparse.ArgumentParser(description="Update the ESP32 IMU Lab firmware over Wi-Fi")
ap.add_argument("--host", default="192.168.4.1")
ap.add_argument("--bin", default=os.path.join(ROOT, "build", "blink.bin"))
ap.add_argument("--password", default="")
ap.add_argument("--timeout", type=float, default=60)
args = ap.parse_args()


def image_info(data):
    """Read the app description that ESP-IDF puts right after the image headers."""
    off = 24 + 8
    magic, = struct.unpack_from("<I", data, off)
    if data[0] != 0xE9 or magic != 0xABCD5432:
        sys.exit(f"{args.bin} is not an ESP-IDF app image")
    version = data[off + 16:off + 48].split(b"\0")[0].decode()
    project = data[off + 48:off + 80].split(b"\0")[0].decode()
    sha = data[off + 144:off + 152].hex()
    return project, version, sha


def get_version(timeout=3):
    c = http.client.HTTPConnection(args.host, 80, timeout=timeout)
    c.request("GET", "/version")
    r = c.getresponse()
    body = r.read()
    c.close()
    return json.loads(body)


with open(args.bin, "rb") as f:
    data = f.read()
project, version, sha = image_info(data)
age = (time.time() - os.path.getmtime(args.bin)) / 60
print(f"File:   {args.bin}  ({len(data)/1024:.0f} KB, {project} {version}, build {sha}, built {age:.0f} min ago)")

try:
    before = get_version()
except OSError as e:
    sys.exit(f"Can't reach the board at {args.host}: {e}\n"
             "Is the PC connected to the IMU-Lab Wi-Fi? Was this firmware with OTA support flashed over USB once?")
print(f"Board:  {before['project']} {before['version']}, build {before['build']}, running from {before['slot']} ({before['state']})")
if before["build"] == sha:
    print("The board already runs this exact build. Sending it anyway.")

conn = http.client.HTTPConnection(args.host, 80, timeout=args.timeout)
conn.putrequest("POST", "/update")
conn.putheader("Content-Type", "application/octet-stream")
conn.putheader("Content-Length", str(len(data)))
if args.password:
    conn.putheader("X-OTA-Password", args.password)
conn.endheaders()
t0 = time.time()
step = 16 * 1024
for i in range(0, len(data), step):
    conn.send(data[i:i + step])
    done = min(i + step, len(data))
    pct = 100 * done / len(data)
    el = time.time() - t0
    speed = f"{done/1024/el:6.0f} KB/s" if el > 1 else ""   # the first chunks only fill Windows' buffer
    print(f"\rUpload: {pct:5.1f} %  {speed}   ", end="", flush=True)
print(f"\rUpload: 100.0 %  {len(data)/1024/(time.time()-t0):.0f} KB/s average, {time.time()-t0:.0f} s")
resp = conn.getresponse()
text = resp.read().decode(errors="replace")
conn.close()
print(f"Board says: {text}")
if resp.status != 200:
    sys.exit(1)

print("Waiting for it to restart", end="", flush=True)
deadline = time.time() + 40
after = None
time.sleep(2)
rejoined = False
while time.time() < deadline:
    try:
        after = get_version(timeout=2)
        break
    except OSError:
        print(".", end="", flush=True)
        time.sleep(1)
        # The board's access point restarts with it, and Windows doesn't always rejoin by itself.
        if not rejoined and sys.platform == "win32" and args.host == "192.168.4.1" and time.time() > deadline - 30:
            rejoined = True
            os.system('netsh wlan connect name="IMU-Lab" >nul 2>&1')
print()
if not after:
    sys.exit("No answer after the restart. Windows may have dropped the IMU-Lab Wi-Fi: reconnect and run\n"
             f"  python tools\\ota.py --host {args.host}   (it will show which build is running)")
print(f"Board:  {after['project']} {after['version']}, build {after['build']}, running from {after['slot']} ({after['state']})")
if after["build"] == sha:
    print("Update OK. It confirms itself as good after 10 s; if it crashes before that, it rolls back on its own.")
else:
    print("The board is NOT running the new build: it probably crashed and rolled back. Check the log over USB.")
    sys.exit(1)
