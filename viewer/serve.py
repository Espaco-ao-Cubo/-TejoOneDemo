#!/usr/bin/env python3
"""Serve the TejoOne viewer to phones, tablets and other computers on the same Wi-Fi.

    python3 serve.py          (Linux / macOS)
    py serve.py               (Windows)
    python3 serve.py 8080     (another port)

Then open the http:// address it prints in Safari on the iPhone/iPad (or any browser).
It has to be http, not https: browsers only let plain http pages talk to the ESP32's
ws:// connection. Once the page has loaded it no longer needs this server, so the
tablet may switch to the ESP32's own "IMU-Lab" Wi-Fi afterwards.
Only this folder is shared. Stop with Ctrl+C.
"""
import functools
import http.server
import os
import socket
import sys

PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 8000
HERE = os.path.dirname(os.path.abspath(__file__))


class Handler(http.server.SimpleHTTPRequestHandler):
    extensions_map = {
        **http.server.SimpleHTTPRequestHandler.extensions_map,
        ".js": "text/javascript",
        ".glb": "model/gltf-binary",
        ".png": "image/png",
    }

    def end_headers(self):
        self.send_header("Cache-Control", "no-store")   # tablets always get the latest version
        super().end_headers()

    def log_message(self, fmt, *args):
        if args and str(args[1]) not in ("200", "304"):
            sys.stderr.write("  %s %s\n" % (self.address_string(), fmt % args))


def lan_addresses():
    found = set()
    # the address used to reach the ESP32's network or the internet (no packets are sent)
    for probe in ("192.168.4.1", "10.255.255.255", "8.8.8.8"):
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
                s.connect((probe, 80))
                found.add(s.getsockname()[0])
        except OSError:
            pass
    try:
        for info in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
            found.add(info[4][0])
    except OSError:
        pass
    return sorted(a for a in found if not a.startswith("127.") and not a.startswith("169.254."))


def main():
    handler = functools.partial(Handler, directory=HERE)
    try:
        server = http.server.ThreadingHTTPServer(("0.0.0.0", PORT), handler)
    except OSError as e:
        sys.exit(f"Port {PORT} is busy ({e}). Try: python3 serve.py 8080")
    sys.stdout.reconfigure(line_buffering=True)
    print("TejoOne viewer is running. Open one of these on the iPhone / iPad / other device:")
    for a in lan_addresses() or ["<this computer's IP address>"]:
        print(f"    http://{a}:{PORT}/")
    print(f"On this computer: http://localhost:{PORT}/")
    print("The device must be on the same Wi-Fi as this computer. Stop with Ctrl+C.")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopped.")


if __name__ == "__main__":
    main()
