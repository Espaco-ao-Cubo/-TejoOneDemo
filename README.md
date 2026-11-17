# TejoOne demonstration

A hand-held model of the TejoOne 1U CubeSat with an ESP32 and a 9-axis IMU inside. The
ESP32 works out the satellite's attitude with an error-state Kalman filter and streams it
over Wi-Fi; a browser shows the TejoOne CAD model turning with it, live telemetry beside it,
and an exploded view when someone knocks twice on the satellite.

| Folder | What it is |
|---|---|
| [`firmware/`](firmware/) | ESP32 firmware (ESP-IDF 6.1): sensors, ESKF, Wi-Fi streaming, admin page, updates over Wi-Fi |
| [`viewer/`](viewer/) | The show page: 3D model, telemetry graphs, presentation mode, exploded view |

## Running the demo

1. **Flash the ESP32** once over USB — see [`firmware/README.md`](firmware/README.md).
   Later updates can go over Wi-Fi with `firmware/tools/ota.py`.
2. **Open the viewer** — see [`viewer/README.md`](viewer/README.md):
   - Windows: `viewer/Open TejoOne.bat`
   - Linux: `viewer/start-linux.sh`
   - macOS: open `viewer/index.html`
   - iPhone / iPad: run `viewer/serve.py` on a computer and open the address it prints
3. Join the ESP32's Wi-Fi **IMU-Lab** (password `imulab1234`), choose **ESP32 · Wi-Fi** and
   connect to `192.168.4.1` (or `imu-lab.local` when the ESP32 is on your home network).
4. Before the show: magnetometer calibration, then **Present → Satellite setup**.

Works on Windows, Linux and macOS (firmware build and viewer) and on iOS/iPadOS 16.4+
(viewer, served over the Wi-Fi).
