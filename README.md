# TejoOne attitude viewer

Live 3D view of the TejoOne 1U CubeSat model, driven by the error-state Kalman filter (ESKF)
running on the ESP32 inside the demo satellite (ICM-20600 + AK09918). Knock twice on the real
satellite and the model opens up into an exploded view.

Everything runs in the browser and works offline: the page, the 3D library (`lib/`) and the
model (`tejo_model.js`) all sit in this folder. Nothing is stored on the ESP32.

## Start it

| Device | How |
|---|---|
| **Windows** | Double-click `Open TejoOne.bat` (Edge or Chrome). |
| **Linux** | `./start-linux.sh` (Chrome, Chromium or Firefox). If it isn't executable: `chmod +x start-linux.sh`. |
| **macOS** | Open `index.html` in Chrome, Edge or Safari. |
| **iPhone / iPad** | See below. Needs iOS / iPadOS 16.4 or newer. |

### iPhone and iPad

Safari can't run a page opened from the Files app, and a secure `https://` page isn't allowed
to talk to the ESP32. So a computer shares the page on the Wi-Fi over plain `http://`:

1. On any computer on the same Wi-Fi, run the server:
   - Windows: double-click `Share on Wi-Fi for iPad.bat`
   - Linux: `./start-linux.sh --serve`
   - macOS / Linux: `python3 serve.py`
2. It prints an address like `http://192.168.1.23:8000/`. Open it in Safari on the iPad.
3. Optional: Share → **Add to Home Screen**. It then opens full screen like an app.

Once the page has loaded it no longer needs the computer, so the iPad can switch to the
ESP32's own `IMU-Lab` Wi-Fi afterwards.

## Connect the ESP32

Join the board's network `IMU-Lab` (or the same network as the board), choose
**ESP32 · Wi-Fi**, enter `imu-lab.local` or `192.168.4.1` and press **Connect ESP32**.
The model then follows the ESKF running on the ESP32. Without a board, **Simulated** shows
a demo motion.

## Before the show

1. **Calibration and ESP32 setup** (bottom of the page): magnetometer calibration, then
   **Send calibration to ESP32**.
2. **Present → Satellite setup**: camera face toward the audience, top solar panel up, then
   the 5 steps. The camera face is the satellite's +X (front), the top panel its +Z.
3. Keys while presenting: **Space** set front, **F** follow turning, **G** graphs,
   **E** open/close the model, **Esc** exit. On a tablet, use the buttons.

## Exploded view

Knock twice, quickly, on the **real** satellite: the IMU feels the two jolts and the model
opens. Knock twice again to close it. Backups: **E**, or the **Look inside** button.
`index.html#inside` starts with it open.

Tune it under **Calibration and ESP32 setup → Double tap to open the model**:

- **Threshold**: knock once and read *Last jolt*; set the threshold to about half of that.
  Taps missed → lower it. Opens while being handled → raise it.
- **Shortest gap / Longest gap**: the second knock must come in this window after the first
  (default 0.10–0.60 s). Slow tappers → raise *Longest gap*.
- **Simulate a double tap** tests it without hardware.

Settings are remembered per browser.

## Rebuild the model

Colours and visibility come from `TejoOne_organized.blend` (collections you exclude or hide
in Blender are left out). With Blender 5.2:

```sh
blender -b TejoOne_organized.blend -P tools/show_model_export.py -- tejo_web.glb
<blender>/5.2/python/bin/python tools/quantize_glb.py tejo_web.glb tejo_web.glb
python3 tools/pack_model.py tejo_web.glb tejo_model.js
```

`show_model_export.py` stores identical parts (screws, rods) once and trims the heavy ones;
`quantize_glb.py` stores coordinates compactly (KHR_mesh_quantization, under 0.01 mm error).

After a CAD change, first export `newnew_tejo_one.SLDASM` from SolidWorks as glTF Binary and
rebuild the `.blend` with `tools/blender_organize.py`. That rebuilds the `.blend` from
scratch, so colour edits made in Blender have to be redone.

## Files

| File | What it is |
|---|---|
| `index.html` | The page |
| `tejo_model.js` | The 3D model, packed for the page |
| `tejo_web.glb` | The same model as a normal glTF file |
| `lib/` | three.js r128 and its GLTFLoader (MIT licence) |
| `serve.py` | Shares the folder on the Wi-Fi for phones and tablets |
| `start-linux.sh`, `*.bat` | Launchers |
| `tools/` | Model build scripts |
