"""
Allan deviation of a recording made with allan_log.py.

    python allan_analyze.py logs/allan_<date>.csv

Writes <name>_adev.csv (load it in the IMU Fusion Lab's Allan variance chapter) and
<name>_report.txt with the noise parameters and suggested ESKF settings.
"""
import sys

import numpy as np

if len(sys.argv) < 2:
    sys.exit(__doc__)
path = sys.argv[1]
d = np.loadtxt(path, delimiter=",", skiprows=1)
t = (d[:, 0] - d[0, 0]) / 1000.0
step = np.diff(d[:, 0])
tau0 = float(np.median(step)) / 1000.0
gap_ms = float(np.sum(step[step > 15] - 10))
hours = t[-1] / 3600

chan = {"gx": d[:, 4], "gy": d[:, 5], "gz": d[:, 6], "ax": d[:, 1], "ay": d[:, 2], "az": d[:, 3]}
temp = d[:, 10]


def adev(x, tau0, points=90):
    """Overlapping Allan deviation via the integrated signal."""
    n = len(x)
    theta = np.concatenate(([0.0], np.cumsum(x) * tau0))
    ms = np.unique(np.logspace(0, np.log10((n - 1) / 9), points).astype(int))
    taus, devs = [], []
    for m in ms:
        diff = theta[2 * m:] - 2 * theta[m:-m] + theta[:-2 * m]
        avar = np.sum(diff ** 2) / (2 * (m * tau0) ** 2 * len(diff))
        taus.append(m * tau0)
        devs.append(np.sqrt(avar))
    return np.array(taus), np.array(devs)


def fit(taus, devs):
    """White noise N (sigma at tau=1 on the -1/2 line), bias instability B, rate random walk K."""
    lt, ld = np.log10(taus), np.log10(devs)
    slope = np.gradient(ld, lt)
    imin = int(np.argmin(devs))
    white = (np.abs(slope + 0.5) < 0.15) & (taus <= max(taus[imin], 1.0))
    N = float(np.median(devs[white] * np.sqrt(taus[white]))) if white.any() else float(np.interp(0, lt, devs))
    B = float(devs[imin] / 0.664)
    rrw = (np.abs(slope - 0.5) < 0.2) & (np.arange(len(taus)) > imin)
    K = float(np.median(devs[rrw] * np.sqrt(3.0 / taus[rrw]))) if rrw.sum() >= 3 else None
    return N, B, float(taus[imin]), K


res = {}
for k, x in chan.items():
    taus, devs = adev(x - x.mean(), tau0)
    res[k] = (taus, devs, fit(taus, devs))

base = path[:-4]
taus = res["gx"][0]
with open(base + "_adev.csv", "w") as f:
    f.write("tau_s,gx_dps,gy_dps,gz_dps,ax_g,ay_g,az_g\n")
    for i, tau in enumerate(taus):
        f.write(f"{tau:.4f}," + ",".join(f"{res[k][1][i]:.6e}" for k in chan) + "\n")

L = []
L.append(f"Recording: {hours:.2f} h, {len(t)} samples, tau0 {tau0*1000:.1f} ms, gaps {gap_ms/1000:.1f} s total")
L.append(f"Temperature: {temp.min():.2f} to {temp.max():.2f} C (drift {temp[-1]-temp[0]:+.2f} C)")
L.append("")
L.append("GYRO            white noise (ARW)            bias instability            rate random walk")
for k in ("gx", "gy", "gz"):
    N, B, tb, K = res[k][2]
    ks = f"{K:.2e} deg/s/sqrt(s)" if K else "not reached (record longer)"
    L.append(f"  {k}   {N:.4f} deg/s/sqrt(Hz) = {N*60:.3f} deg/sqrt(h)   "
             f"{B*3600:6.2f} deg/h at tau {tb:6.0f} s   {ks}")
L.append("")
L.append("ACCEL           white noise (VRW)            bias instability")
for k in ("ax", "ay", "az"):
    N, B, tb, K = res[k][2]
    L.append(f"  {k}   {N*1e6:7.1f} ug/sqrt(Hz)                {B*1e6:7.1f} ug at tau {tb:6.0f} s")

Ng = max(res[k][2][0] for k in ("gx", "gy", "gz"))
Ks = [res[k][2][3] for k in ("gx", "gy", "gz") if res[k][2][3]]
Bg = max(res[k][2][1] for k in ("gx", "gy", "gz"))
tB = float(np.median([res[k][2][2] for k in ("gx", "gy", "gz")]))
bw = max(Ks) if Ks else Bg / np.sqrt(tB)
L.append("")
L.append("Suggested ESKF settings (measured values are for a perfectly still board;")
L.append("real use adds vibration, so the filter uses them inflated):")
L.append(f"  measured   sigma gyro      {Ng:.4f} deg/s/sqrt(Hz)    sigma bias walk {bw:.2e} deg/s/sqrt(s)"
         + ("" if Ks else "  (estimated from bias instability)"))
L.append(f"  suggested  gyro_noise      {Ng*3:.4f} deg/s/sqrt(Hz)    bias_walk       {bw*3:.2e} deg/s/sqrt(s)   (x3)")
L.append(f"  currently  gyro_noise      0.0100 deg/s/sqrt(Hz)    bias_walk       5.00e-04 deg/s/sqrt(s)")
report = "\n".join(L)
with open(base + "_report.txt", "w") as f:
    f.write(report + "\n")
print(report)
print(f"\nWrote {base}_adev.csv and {base}_report.txt")
