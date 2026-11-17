# TejoOne demo firmware (ESP32)

Runs inside the demo satellite. Reads the ICM-20600 (gyro + accelerometer) and AK09918
(magnetometer) over I2C, fuses them with an error-state Kalman filter (ESKF) and a Mahony
filter, and streams everything to the viewer over Wi-Fi. ESP-IDF 6.1, target `esp32`.

(The CMake project is still called `blink`, the ESP-IDF example it started from, so the
firmware file is `build/blink.bin`.)

## Wiring

| IMU pin | ESP32 pin | Note |
|---|---|---|
| VCC | 3V3 | Not 5 V unless the module has a regulator |
| GND | GND | |
| SDA | GPIO 21 | ICM-20600 at 0x68/0x69, AK09918 at 0x0C |
| SCL | GPIO 22 | 400 kHz |

Fusion runs at 500 Hz with the ESP32's internal pull-ups. With 2.2–4.7 kΩ pull-ups from SDA
and SCL to 3.3 V, set 1000 Hz in menuconfig.

## Build and flash (Windows, Linux, macOS)

1. Install **ESP-IDF 6.1**:
   - Windows: the ESP-IDF installer, then use the *ESP-IDF PowerShell/CMD* shortcut.
   - Linux / macOS: `git clone -b v6.1 --recursive https://github.com/espressif/esp-idf`,
     `./install.sh esp32`, then in every new terminal `. ./export.sh`.
   - Or, on any system with Docker: open this folder in VS Code and *Reopen in Container*
     (`.devcontainer/` uses Espressif's official image).
2. In this folder:
   ```sh
   idf.py set-target esp32      # first time only
   idf.py build
   idf.py -p PORT flash monitor
   ```
   `PORT` is `COM4` (or similar) on Windows, `/dev/ttyUSB0` on Linux, and
   `/dev/cu.usbserial-*` or `/dev/cu.SLAB_USBtoUART` on macOS. On Linux, give yourself
   access to serial ports once: `sudo usermod -aG dialout $USER`, then log out and in.
   Leave the monitor with `Ctrl+]`.

The first build downloads `mdns` and `led_strip` from the ESP component registry (versions
pinned in `dependencies.lock`).

### Settings

`idf.py menuconfig` → **IMU Lab**:

| Setting | Default |
|---|---|
| Sensor read and fusion rate | 500 Hz |
| Home Wi-Fi name / password | empty (join later from the admin page) |
| ESP32 access point name / password | `IMU-Lab` / `imulab1234` |
| Hostname on your home network | `imu-lab` (`imu-lab.local`) |
| Admin password (firmware updates, admin commands) | empty |

Your choices are saved in `sdkconfig`, which is **not** committed: it can contain Wi-Fi and
admin passwords. Project-wide settings belong in `sdkconfig.defaults`.

## Using it

- The ESP32 always runs its own Wi-Fi, **IMU-Lab** (password `imulab1234`). Its admin page is
  `http://192.168.4.1/`: live orientation, commands, firmware update, log, and up to 5 saved
  home networks (it joins the strongest).
- The show viewer (`../viewer`) connects to `ws://imu-lab.local/ws` or `ws://192.168.4.1/ws`.

### Data stream

USB, 115200 baud, one text line every 10 ms:

```
$I,t_ms,ax,ay,az,gx,gy,gz,mx,my,mz,temp      (g, °/s, µT, °C)
```

Wi-Fi: a 120-byte binary WebSocket frame every 10 ms at `ws://<esp32>/ws`, little-endian:

| Bytes | Field | Type |
|---|---|---|
| 0–3 | `"IQ"`, version 1, flags (bit 0 gravity used, 1 heading used, 2 mag present, 3 gyro clipped, 4 accel clipped) | char·2, u8, u8 |
| 4–11 | time (ms), fusion loop rate (Hz), slowest loop (µs) | u32, u16, u16 |
| 12–51 | accel (g), gyro (°/s): means over the last 10 ms; mag (µT); temperature | f32 × 10 |
| 52–83 | ESKF quaternion, Mahony quaternion (w, x, y, z) | f32 × 8 |
| 84–119 | ESKF bias, Mahony bias (°/s), ESKF 1-σ attitude (°) | f32 × 9 |

Commands sent as WebSocket text:

| Command | Effect |
|---|---|
| `RESET` | Re-start both filters from the current accelerometer and magnetometer |
| `CAL ox oy oz sx sy sz +x +y +z` | Magnetometer offset (µT), scale and axis map; saved in flash |
| `CALCLEAR` | Forget the stored magnetometer calibration |

### Update over Wi-Fi (OTA)

```sh
idf.py build
python tools/ota.py            # --host imu-lab.local, --password ... if one is set
```

Two firmware slots (`partitions.csv`); a new image must run for 10 s before it is kept,
otherwise the board rolls back to the previous one. Changes to the partition table or the
bootloader still need USB.

## Tools

| Path | What it does |
|---|---|
| `tools/ota.py` | Firmware update over Wi-Fi (Python standard library only) |
| `tools/imu_lab.html` | The full IMU Fusion Lab: all six filters side by side, USB or Wi-Fi, calibration, Allan variance. Open it in Chrome or Edge. |
| `tools/allan/allan_log.py` | Records the USB stream for Allan variance (finds the port itself) |
| `tools/allan/allan_analyze.py` | Noise parameters and suggested ESKF settings from a recording |
| `tools/allan/ws_record.py` | Records the Wi-Fi stream to CSV |

For the Allan tools: `python -m venv .venv`, then `.venv/bin/pip install -r requirements.txt`
(Windows: `.venv\Scripts\pip ...`) inside `tools/allan/`. Raw recordings are large and stay
out of git; the `_adev.csv` and `_report.txt` results are kept in `tools/allan/logs/`.
