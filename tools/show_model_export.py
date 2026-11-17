"""blender -b TejoOne_organized.blend -P show_model_export.py -- <out.glb>

Lightweight copy of the TejoOne model for the web viewer.
  * Identical parts (the ~40 M3 screws, rods, springs, ...) share one mesh, which the glTF
    file stores once, so small parts keep their full CAD detail at almost no size cost.
  * Every unique part is trimmed gently (see SOFT and CAP below); the page's exploded view
    shows the inside too, so nothing is treated as hidden.
  * Only what is visible in the .blend is exported: an excluded or hidden collection
    (for example 06_RBF) is left out.
  * The satellite is turned so the camera lens faces +X and the top panel +Z, then centred
    (after glTF's Y-up conversion the web page turns it back with one +90 deg X rotation).
"""
import sys, math, re, bpy
from collections import defaultdict
from mathutils import Vector

argv = sys.argv[sys.argv.index("--") + 1:]
OUT = argv[0]

meshes = [o for o in bpy.data.objects if o.type == "MESH" and o.visible_get()]
def tris(me): return sum(len(p.vertices) - 2 for p in me.polygons)

# --- 1. identical parts share one mesh --------------------------------------------
def base(n):
    n = n.split("/")[-1]; n = re.sub(r"\.\d{3}$", "", n); return re.sub(r"-\d+$", "", n)
def sig(o):
    me, v = o.data, o.data.vertices
    pick = lambda i: tuple(round(c, 6) for c in v[i].co)
    return (base(o.name), len(v), len(me.polygons), pick(0), pick(len(v) // 2), pick(len(v) - 1),
            tuple(m.name if m else "" for m in me.materials))
groups = defaultdict(list)
for o in meshes:
    if o.modifiers or o.data.shape_keys or len(o.data.vertices) == 0:
        groups[("solo", o.name)].append(o)
    else:
        groups[sig(o)].append(o)
for objs in groups.values():
    for o in objs[1:]:
        o.data = objs[0].data
before = sum(tris(o.data) for o in meshes)
print(f"MODEL parts {len(meshes)}, unique meshes {len(groups)}, tris drawn {before}")

# --- 2. trim every unique mesh a little, big ones more ------------------------------
# The page has an exploded view, so inside parts are seen too: nothing is treated as hidden.
# Up to SOFT triangles a part is kept as is; above that, only a quarter of the extra
# triangles stay, and no single part keeps more than CAP.
SOFT, CAP = 3000, 12000
dg = bpy.context.evaluated_depsgraph_get()
for objs in groups.values():
    o = objs[0]; t = tris(o.data)
    target = min(t, SOFT + 0.25 * max(0, t - SOFT), CAP)
    if target >= t * 0.97: continue
    m = o.modifiers.new("web_decimate", "DECIMATE")
    m.decimate_type = "COLLAPSE"; m.ratio = target / t; m.use_collapse_triangulate = True
    dg.update()
    new = bpy.data.meshes.new_from_object(o.evaluated_get(dg))
    o.modifiers.remove(m)
    for x in objs: x.data = new
    dg = bpy.context.evaluated_depsgraph_get()
unique_after = sum(tris(objs[0].data) for objs in groups.values())
print(f"MODEL unique tris stored {unique_after}, tris drawn {sum(tris(o.data) for o in meshes)}")

# --- 3. lens to +X, then centre ------------------------------------------------------
root = bpy.data.objects["TejoOne_1U"]
root.rotation_mode = "XYZ"
root.rotation_euler.z += math.radians(90)
bpy.context.view_layer.update()
pts = [o.matrix_world @ Vector(c) for o in meshes for c in o.bound_box]
lo = Vector((min(p.x for p in pts), min(p.y for p in pts), min(p.z for p in pts)))
hi = Vector((max(p.x for p in pts), max(p.y for p in pts), max(p.z for p in pts)))
root.location -= (lo + hi) / 2
bpy.context.view_layer.update()

bpy.ops.export_scene.gltf(
    filepath=OUT, export_format="GLB", export_apply=False, use_visible=True,
    export_cameras=False, export_lights=False, export_animations=False,
    export_texcoords=False, export_extras=False,
)
print("MODEL saved", OUT)
