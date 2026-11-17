"""Shrink a GLB with KHR_mesh_quantization (run with Blender's Python, which has numpy):
    "<blender>/5.2/python/bin/python.exe" quantize_glb.py in.glb out.glb

POSITION -> 16-bit integers (normalised), NORMAL -> 8-bit integers (normalised).
Each mesh gets its own uniform scale + offset, carried by a small child node, so shared
meshes (the ~40 identical screws) stay shared. Precision: 1/32767 of the part's size,
under 0.01 mm for anything on this satellite.
"""
import json, struct, sys
import numpy as np

src, dst = sys.argv[1], sys.argv[2]
data = open(src, "rb").read()
jl = struct.unpack_from("<I", data, 12)[0]
gltf = json.loads(data[20:20 + jl])
bl = struct.unpack_from("<I", data, 20 + jl)[0]
binc = data[28 + jl:28 + jl + bl]

CT = {5120: np.int8, 5121: np.uint8, 5122: np.int16, 5123: np.uint16, 5125: np.uint32, 5126: np.float32}
NC = {"SCALAR": 1, "VEC2": 2, "VEC3": 3, "VEC4": 4, "MAT4": 16}
acc, views = gltf["accessors"], gltf["bufferViews"]

def read(ai):
    a = acc[ai]; v = views[a["bufferView"]]
    dt = np.dtype(CT[a["componentType"]]); n = NC[a["type"]]
    off = v.get("byteOffset", 0) + a.get("byteOffset", 0)
    stride = v.get("byteStride", dt.itemsize * n)
    raw = np.frombuffer(binc, dtype=np.uint8, count=stride * (a["count"] - 1) + dt.itemsize * n, offset=off)
    out = np.empty((a["count"], n), dtype=dt)
    for i in range(n):
        out[:, i] = np.ndarray((a["count"],), dtype=dt, buffer=raw.tobytes(), offset=i * dt.itemsize, strides=(stride,))
    return out

# per-mesh dequantisation: uniform scale s and offset c
mesh_q = {}
for mi, m in enumerate(gltf["meshes"]):
    pos = np.concatenate([read(p["attributes"]["POSITION"]).astype(np.float64) for p in m["primitives"]])
    lo, hi = pos.min(0), pos.max(0)
    c = (lo + hi) / 2; s = max(float((hi - lo).max()) / 2, 1e-9)
    mesh_q[mi] = (c, s)

new_bin = bytearray(); new_views = []
def add_view(b, stride=None, target=None):
    while len(new_bin) % 4: new_bin.append(0)
    v = {"buffer": 0, "byteOffset": len(new_bin), "byteLength": len(b)}
    if stride: v["byteStride"] = stride
    if target: v["target"] = target
    new_bin.extend(b); new_views.append(v); return len(new_views) - 1

done = {}
for mi, m in enumerate(gltf["meshes"]):
    c, s = mesh_q[mi]
    for p in m["primitives"]:
        at = p["attributes"]
        pa = at["POSITION"]
        if ("P", pa) not in done:
            q = np.round((read(pa).astype(np.float64) - c) / s * 32767).clip(-32767, 32767).astype(np.int16)
            buf = np.zeros((len(q), 4), np.int16); buf[:, :3] = q
            a = acc[pa]
            a.update(bufferView=add_view(buf.tobytes(), 8, 34962), componentType=5122, normalized=True, byteOffset=0,
                     min=[int(x) for x in q.min(0)], max=[int(x) for x in q.max(0)])
            done[("P", pa)] = 1
        na = at.get("NORMAL")
        if na is not None and ("N", na) not in done:
            nrm = read(na).astype(np.float64)
            nrm /= np.maximum(np.linalg.norm(nrm, axis=1, keepdims=True), 1e-12)
            q = np.round(nrm * 127).clip(-127, 127).astype(np.int8)
            buf = np.zeros((len(q), 4), np.int8); buf[:, :3] = q
            a = acc[na]
            a.update(bufferView=add_view(buf.tobytes(), 4, 34962), componentType=5120, normalized=True, byteOffset=0)
            a.pop("min", None); a.pop("max", None)
            done[("N", na)] = 1

# every other accessor (indices, ...) is copied as is, tightly packed
for ai, a in enumerate(acc):
    if ("P", ai) in done or ("N", ai) in done or "bufferView" not in a: continue
    v = views[a["bufferView"]]
    arr = read(ai)
    a["bufferView"] = add_view(arr.tobytes(), None, v.get("target")); a["byteOffset"] = 0
gltf["bufferViews"] = new_views
gltf["buffers"] = [{"byteLength": len(new_bin)}]

# nodes: move each mesh onto a child node that carries its dequantisation
nodes = gltf["nodes"]
for ni in range(len(nodes)):
    n = nodes[ni]
    if "mesh" not in n: continue
    c, s = mesh_q[n["mesh"]]
    child = {"name": n.get("name", "") + "_q", "mesh": n.pop("mesh"),
             "translation": [float(x) for x in c], "scale": [s, s, s]}
    nodes.append(child)
    n.setdefault("children", []).append(len(nodes) - 1)
for k in ("extensionsUsed", "extensionsRequired"):
    gltf[k] = sorted(set(gltf.get(k, [])) | {"KHR_mesh_quantization"})

js = json.dumps(gltf, separators=(",", ":")).encode()
js += b" " * (-len(js) % 4)
while len(new_bin) % 4: new_bin.append(0)
out = struct.pack("<III", 0x46546C67, 2, 12 + 8 + len(js) + 8 + len(new_bin))
out += struct.pack("<II", len(js), 0x4E4F534A) + js + struct.pack("<II", len(new_bin), 0x004E4942) + bytes(new_bin)
open(dst, "wb").write(out)
print(f"QUANT {len(data)/1e6:.2f} MB -> {len(out)/1e6:.2f} MB")
