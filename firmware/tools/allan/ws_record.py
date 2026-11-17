"""
Record the ESP32's Wi-Fi packets (raw sensors + both on-board filters) to a CSV.

    python ws_record.py --secs 120 --out logs/motion.csv

Works alongside the lab in the browser: the board serves several clients.
"""
import argparse
import base64
import os
import socket
import struct
import time

ap = argparse.ArgumentParser()
ap.add_argument("--host", default="192.168.4.1")
ap.add_argument("--secs", type=float, default=120)
ap.add_argument("--out", default="motion.csv")
ap.add_argument("--quiet", action="store_true")
args = ap.parse_args()

s = socket.create_connection((args.host, 80), timeout=3)
key = base64.b64encode(os.urandom(16)).decode()
s.sendall((f"GET /ws HTTP/1.1\r\nHost: {args.host}\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n"
           f"Sec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\n\r\n").encode())
resp = b""
while b"\r\n\r\n" not in resp:
    resp += s.recv(1024)
head, rest = resp.split(b"\r\n\r\n", 1)
if b" 101 " not in head.split(b"\r\n")[0]:
    raise SystemExit(head.split(b"\r\n")[0].decode())
buf = bytearray(rest)
s.settimeout(1)

FMT = "<2sBBIHH" + "f" * 27   # 120 bytes, matches imu_packet_t
cols = ("t_ms,flags,ax,ay,az,gx,gy,gz,mx,my,mz,temp,qw,qx,qy,qz,mw,mx_,my_,mz_,"
        "bex,bey,bez,bmx,bmy,bmz,sx,sy,sz")


def need(n):
    while len(buf) < n:
        chunk = s.recv(4096)
        if not chunk:
            raise ConnectionError("closed")
        buf.extend(chunk)


rows = 0
end = time.time() + args.secs
next_note = time.time() + 10
with open(args.out, "w", newline="") as f:
    f.write(cols + "\n")
    while time.time() < end:
        try:
            need(2)
            n = buf[1] & 0x7F
            hl = 2
            if n == 126:
                need(4); n = struct.unpack(">H", buf[2:4])[0]; hl = 4
            need(hl + n)
            p = bytes(buf[hl:hl + n])
            del buf[:hl + n]
        except socket.timeout:
            continue
        if len(p) != 120 or p[:2] != b"IQ":
            continue
        v = struct.unpack(FMT, p)
        f.write(f"{v[3]},{v[2]}," + ",".join(f"{x:.6g}" for x in v[6:]) + "\n")
        rows += 1
        if not args.quiet and time.time() >= next_note:
            print(f"{args.secs - (end - time.time()):5.0f} s  {rows} packets", flush=True)
            next_note += 10
s.close()
print(f"wrote {rows} packets to {args.out}")
