"""Headless Blender OBJ→DAE converter for Rovo sensor meshes.

Usage:
  blender --background --python convert_sensor_obj.py

Reads OBJ + MTL from /home/godfather/Work/Claude/Rovo_Model/sensor_model/
and writes DAE into rovo_gz_description/meshes/sensor/ with sanitized
filenames (spaces and '+' replaced with '_').
"""
import bpy
import os
import re

SRC = "/home/godfather/Work/Claude/Rovo_Model/sensor_model"
DST = "/home/godfather/Work/Claude/Rovo_Model/rovo_gz_description/meshes/sensor"

os.makedirs(DST, exist_ok=True)


def sanitize(name: str) -> str:
    """OBJ filename → DAE filename. Replace spaces, '+', and other shell-
    unfriendly chars with underscores."""
    base = os.path.splitext(name)[0]
    base = re.sub(r"[ +]+", "_", base)
    base = re.sub(r"[^A-Za-z0-9_\-]", "", base)
    return base + ".dae"


def convert(obj_path: str, dae_path: str) -> None:
    bpy.ops.wm.read_factory_settings(use_empty=True)
    bpy.ops.wm.obj_import(filepath=obj_path)
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.wm.collada_export(
        filepath=dae_path,
        selected=True,
        apply_modifiers=True,
        triangulate=True,
        export_mesh_type_selection="view",
        use_texture_copies=True,
    )
    print(f"CONVERTED {obj_path} → {dae_path}")


def main() -> None:
    for fname in sorted(os.listdir(SRC)):
        if not fname.lower().endswith(".obj"):
            continue
        src = os.path.join(SRC, fname)
        dst = os.path.join(DST, sanitize(fname))
        convert(src, dst)
    print("DONE")


if __name__ == "__main__":
    main()
