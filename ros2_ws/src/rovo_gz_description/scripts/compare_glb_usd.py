"""Compare body.glb (URDF chassis) vs V4.usdc (full CAD assembly).

Loads both into a Blender scene, aligns USD to GLB coords using the
chassis as a reference, then dumps centroid + bbox of every USD mesh
that is NOT in the GLB part list — these are the sensor-stack parts we
need to position in URDF.

Usage:
  ./blender-3.3.21-linux-x64/blender --background \\
      --python rovo_gz_description/scripts/compare_glb_usd.py
"""
import bpy
import mathutils
from collections import defaultdict

GLB = "/home/godfather/Work/Claude/Rovo_Model/body.glb"
USD = "/home/godfather/Work/Claude/Rovo_Model/V4/V4.usdc"

GLB_PART_NAMES = {
    "body", "camera_holder", "lightbody", "pipes", "wheel_holder",
    "wheelTrack_new", "wheelTrack_new.001",
    "wl", "wl1", "wl2", "wl3", "wl4", "wl5", "wl6",
    "wr", "wr1", "wr2", "wr3", "wr4", "wr5", "wr6",
    "Plane",
}


def world_bbox(obj):
    """Return (min_corner, max_corner, dim, centroid) in world space."""
    coords = [obj.matrix_world @ mathutils.Vector(c) for c in obj.bound_box]
    xs = [c.x for c in coords]; ys = [c.y for c in coords]; zs = [c.z for c in coords]
    mn = mathutils.Vector((min(xs), min(ys), min(zs)))
    mx = mathutils.Vector((max(xs), max(ys), max(zs)))
    dim = mx - mn
    cen = (mn + mx) * 0.5
    return mn, mx, dim, cen


def scene_bbox(objects):
    if not objects:
        return None
    mn = mathutils.Vector(( 1e9,  1e9,  1e9))
    mx = mathutils.Vector((-1e9, -1e9, -1e9))
    for o in objects:
        omn, omx, _, _ = world_bbox(o)
        for i in range(3):
            mn[i] = min(mn[i], omn[i])
            mx[i] = max(mx[i], omx[i])
    return mn, mx, (mx - mn), (mn + mx) * 0.5


def main():
    bpy.ops.wm.read_factory_settings(use_empty=True)

    # ---- Load GLB ----
    bpy.ops.import_scene.gltf(filepath=GLB)
    glb_objs = [o for o in bpy.data.objects if o.type == "MESH"]
    # Exclude Plane (ground) from chassis bbox
    glb_chassis = [o for o in glb_objs if o.name != "Plane"]
    glb_mn, glb_mx, glb_dim, glb_cen = scene_bbox(glb_chassis)
    print(f"GLB_CHASSIS bbox min={tuple(round(v,3) for v in glb_mn)} "
          f"max={tuple(round(v,3) for v in glb_mx)} "
          f"dim={tuple(round(v,3) for v in glb_dim)} "
          f"centroid={tuple(round(v,3) for v in glb_cen)}")

    # Mark GLB objects so we can exclude after USD import
    glb_obj_names = {o.name for o in glb_objs}

    # ---- Load USD into the same scene ----
    bpy.ops.wm.usd_import(filepath=USD)
    all_objs = [o for o in bpy.data.objects if o.type == "MESH"]
    usd_objs = [o for o in all_objs if o.name not in glb_obj_names]
    if not usd_objs:
        print("ERROR: USD import produced no new mesh objects")
        return

    usd_mn, usd_mx, usd_dim, usd_cen = scene_bbox(usd_objs)
    print(f"USD_ALL bbox min={tuple(round(v,3) for v in usd_mn)} "
          f"max={tuple(round(v,3) for v in usd_mx)} "
          f"dim={tuple(round(v,3) for v in usd_dim)} "
          f"centroid={tuple(round(v,3) for v in usd_cen)}")

    # ---- Derive scale from chassis dims ----
    # GLB chassis roughly 1.31 × 1.46 × 0.90 m
    # USD assembly bbox in mm → divide by 1000 to get m
    # Use the largest dimension ratio as scale
    scale_candidates = [
        glb_dim[i] / usd_dim[i] if usd_dim[i] > 1e-6 else 0
        for i in range(3)
    ]
    print(f"SCALE_CANDIDATES (glb/usd) per-axis = {scale_candidates}")
    # Pick median if values disagree
    scales = sorted(s for s in scale_candidates if s > 0)
    s = scales[len(scales) // 2] if scales else 1.0
    print(f"CHOSEN_SCALE = {s:.6f}  (≈ 0.001 means mm→m)")

    # ---- Apply scale + translation to align USD centroid to GLB centroid ----
    for o in usd_objs:
        o.scale = (s, s, s)
    bpy.context.view_layer.update()
    usd_mn2, usd_mx2, usd_dim2, usd_cen2 = scene_bbox(usd_objs)
    delta = glb_cen - usd_cen2
    for o in usd_objs:
        o.location += delta
    bpy.context.view_layer.update()
    usd_mn3, usd_mx3, usd_dim3, usd_cen3 = scene_bbox(usd_objs)
    print(f"USD_ALIGNED centroid={tuple(round(v,3) for v in usd_cen3)} "
          f"dim={tuple(round(v,3) for v in usd_dim3)}")

    # ---- Heuristic sensor identification ----
    # Buckets by characteristic dims:
    #   small puck   (5-12 cm any dim, height < 5 cm) → GPS candidate
    #   cylinder    (8-12 cm diameter, 5-10 cm tall, roughly equal X/Y) → velodyne
    #   small box   (1-3 cm thick, 8-12 cm width)    → realsense
    #   flat plate  (>15 cm × >15 cm × <3 cm)         → mount plate
    print("==SENSOR_CANDIDATES==")
    for o in usd_objs:
        if o.name in glb_obj_names:
            continue
        omn, omx, dim, cen = world_bbox(o)
        # Filter out tiny noise (bolts) and giant chassis carry-overs
        if max(dim) < 0.02 or max(dim) > 1.5:
            continue
        # Filter by Z above chassis top (z > 0.5 m) — sensor stack is up high
        if cen.z < 0.4:
            continue
        sx, sy, sz = sorted([dim.x, dim.y, dim.z])
        label = "?"
        if 0.03 <= sz <= 0.10 and 0.07 <= sx <= 0.13 and 0.07 <= sy <= 0.13 and abs(dim.x - dim.y) < 0.03:
            label = "VELODYNE?"
        elif 0.005 <= sx <= 0.04 and 0.05 <= sy <= 0.12 and 0.005 <= sz <= 0.04:
            label = "REALSENSE?"
        elif sz <= 0.03 and 0.04 <= sx <= 0.10 and 0.04 <= sy <= 0.10:
            label = "GPS?"
        elif sz <= 0.03 and sx >= 0.15 and sy >= 0.15:
            label = "MOUNT_PLATE?"
        print(f"CAND {label:14s} name={o.name:20s} cen=({cen.x:+.3f},{cen.y:+.3f},{cen.z:+.3f}) dim=({dim.x:.3f},{dim.y:.3f},{dim.z:.3f})")

    # ---- Also: top-Z USD parts (most likely sensors) ----
    print("==TOP_Z_USD_PARTS==")
    parts = []
    for o in usd_objs:
        omn, omx, dim, cen = world_bbox(o)
        if max(dim) < 0.02 or max(dim) > 1.0:
            continue
        parts.append((cen.z, o.name, cen, dim))
    parts.sort(reverse=True)
    for z, name, cen, dim in parts[:20]:
        print(f"TOPZ name={name:20s} cen=({cen.x:+.3f},{cen.y:+.3f},{cen.z:+.3f}) dim=({dim.x:.3f},{dim.y:.3f},{dim.z:.3f})")

    print("DONE")


if __name__ == "__main__":
    main()
