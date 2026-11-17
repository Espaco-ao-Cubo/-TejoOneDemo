#!/usr/bin/env python3
"""Pack a .glb into tejo_model.js (gzip + base64), the file the page loads.
    python3 tools/pack_model.py tejo_web.glb tejo_model.js
Script-tag loading works everywhere, even when index.html is opened straight from disk."""
import base64, gzip, sys
src = sys.argv[1] if len(sys.argv) > 1 else "tejo_web.glb"
dst = sys.argv[2] if len(sys.argv) > 2 else "tejo_model.js"
data = base64.b64encode(gzip.compress(open(src, "rb").read(), 9)).decode()
with open(dst, "w") as f:
    f.write("/* TejoOne 1U model: gzipped GLB (KHR_mesh_quantization), base64. Built with tools/pack_model.py */\n")
    f.write(f'window.TEJO_GLB_GZ_B64="{data}";\n')
print(f"{dst}: {len(data) / 1e6:.2f} MB")
