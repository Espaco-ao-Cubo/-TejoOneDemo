"""Run with:  blender -b --factory-startup -P blender_organize.py -- <in.glb> <out_dir>

Imports the SolidWorks glTF export, recenters the satellite on the world origin,
groups every top-level component under a category empty + collection, and saves
both a .blend and a re-exported .glb that keep that hierarchy.
"""
import sys, os, re
import bpy
from mathutils import Vector

argv = sys.argv[sys.argv.index("--") + 1:]
SRC, OUT = argv[0], argv[1]
os.makedirs(OUT, exist_ok=True)

CATEGORIES = [  # (collection/empty name, regex on top-level SolidWorks name)
    ("01_Structure",        r"^Structure"),
    ("02_Solar_Panels",     r"^ra-hw-tejo-solar"),
    ("03_Payload_Optics",   r"^(STR_w_baffle|Baffle_principais|camera_face|camara_support|top_baffle_support|bottom_baffle_support)"),
    ("04_Electronics",      r"^(SM-AMUN-PSU|top_plate_OBC|bottom_plate_OBC|SSQ-|DCCM37|umbilical_connector|MTQ)"),
    ("05_Antenna",          r"^ESUANT"),
    ("06_RBF",              r"^RBF_"),
    ("07_Fasteners",        r"^(A2M|Nut|989|912A2|7475|7884|Spacer)"),
]
FALLBACK = "08_Other"

# --- clean scene and import ---------------------------------------------------
bpy.ops.wm.read_factory_settings(use_empty=True)
bpy.ops.import_scene.gltf(filepath=SRC)
scene = bpy.context.scene

for ob in [o for o in scene.objects if o.type == "CAMERA"]:
    bpy.data.objects.remove(ob, do_unlink=True)

root = next(o for o in scene.objects if o.parent is None and o.name.startswith("newnew_tejo_one"))
root.name = "TejoOne_1U"

# --- recenter -----------------------------------------------------------------
bpy.context.view_layer.update()
def world_bbox(objs):
    lo = Vector((1e9,) * 3); hi = Vector((-1e9,) * 3)
    for o in objs:
        for c in o.bound_box:
            p = o.matrix_world @ Vector(c)
            lo = Vector(map(min, lo, p)); hi = Vector(map(max, hi, p))
    return lo, hi
meshes = [o for o in scene.objects if o.type == "MESH"]
lo, hi = world_bbox(meshes)
center = (lo + hi) / 2
print(f"before: center={tuple(round(v,4) for v in center)} m  size={tuple(round(v,4) for v in hi-lo)} m")
root.location = root.location - center   # root has no parent, so location is world space
bpy.context.view_layer.update()

# --- category empties + collections ------------------------------------------
master = bpy.data.collections.new("TejoOne_1U")
scene.collection.children.link(master)
def move_tree(ob, coll):
    for c in list(ob.users_collection):
        c.objects.unlink(ob)
    coll.objects.link(ob)
    for ch in ob.children:
        move_tree(ch, coll)

move_tree(root, master)
groups = {}
def group(name):
    if name not in groups:
        coll = bpy.data.collections.new(name)
        master.children.link(coll)
        emp = bpy.data.objects.new(name, None)
        emp.empty_display_size = 0.02
        coll.objects.link(emp)
        emp.parent = root
        bpy.context.view_layer.update()   # refresh emp.matrix_world before children use it
        groups[name] = (coll, emp)
    return groups[name]

counts = {}
for ch in sorted(root.children, key=lambda o: o.name):
    raw = ch.name
    cat = next((n for n, rx in CATEGORIES if re.search(rx, raw)), FALLBACK)
    coll, emp = group(cat)
    mw = ch.matrix_world.copy()
    ch.parent = emp
    ch.matrix_parent_inverse.identity()
    ch.matrix_basis = emp.matrix_world.inverted() @ mw   # keep exact placement
    move_tree(ch, coll)
    counts[cat] = counts.get(cat, 0) + 1

# readable names for the repeated sub-assemblies Blender suffixed with .001 etc.
for o in scene.objects:
    o.name = re.sub(r"^ra-hw-tejo-solar-side-normal_1-1u-v1a-step\.step", "Solar_Side", o.name)
    o.name = re.sub(r"^ra-hw-tejo-solar-top-rbf-1u-v1a-step\.step", "Solar_Top_RBF", o.name)

bpy.context.view_layer.update()
lo, hi = world_bbox([o for o in scene.objects if o.type == "MESH"])
print(f"after:  center={tuple(round(v,4) for v in (lo+hi)/2)} m  size={tuple(round(v,4) for v in hi-lo)} m")
print("objects:", len(scene.objects), "meshes:", sum(o.type == 'MESH' for o in scene.objects))
print("unparented besides root:", [o.name for o in scene.objects if o.parent is None and o is not root])
for k in sorted(counts): print(f"  {k}: {counts[k]} components")

# --- save ---------------------------------------------------------------------
scene.unit_settings.system = "METRIC"
scene.unit_settings.length_unit = "MILLIMETERS"
blend = os.path.join(OUT, "TejoOne_organized.blend")
bpy.ops.wm.save_as_mainfile(filepath=blend)
glb = os.path.join(OUT, "TejoOne_organized.glb")
bpy.ops.export_scene.gltf(filepath=glb, export_format="GLB")
print("saved", blend); print("saved", glb)

# --- quick preview render -------------------------------------------------------
try:
    scene.render.engine = "BLENDER_WORKBENCH"
    scene.display.shading.color_type = "MATERIAL"
    scene.render.resolution_x, scene.render.resolution_y = 1200, 900
    cam = bpy.data.objects.new("PreviewCam", bpy.data.cameras.new("PreviewCam"))
    scene.collection.objects.link(cam)
    cam.location = (0.28, -0.28, 0.2)
    d = Vector((0, 0, 0)) - cam.location
    cam.rotation_euler = d.to_track_quat("-Z", "Y").to_euler()
    cam.data.lens = 50
    scene.camera = cam
    scene.render.filepath = os.path.join(OUT, "preview.png")
    bpy.ops.render.render(write_still=True)
    bpy.data.objects.remove(cam, do_unlink=True)
    print("preview ok")
except Exception as e:
    print("preview failed:", e)
