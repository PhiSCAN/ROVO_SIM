#!/usr/bin/env python3
"""Split shinjuku_visual.glb into an N-metre grid of separate GLB tiles.

Why: gz/ogre2 drops a single oversized off-centre mesh at close range
(shinjuku = one 1 km monolith, 2757 submeshes under ONE cull unit). The
working worlds avoid this: autoware centres its one mesh on the origin,
warehouse splits everything into many small models. This tool reproduces the
warehouse structure for shinjuku -- it bins the GLB's existing 2757 submeshes
by world centroid into a grid and writes each non-empty cell as its own GLB,
which the SDF then references as a separate <visual> (separate cull unit).

No triangle cutting: submeshes are kept whole and re-exported with their
original node transforms and materials, so every tile stays aligned with the
single collision STL and no re-posing is needed.

Usage:
    python3 tile_split.py [--tile 100] [--src ...glb] [--out ...dir]
Prints the SDF <visual> block for the generated tiles on stdout.
"""
import argparse
import os
from collections import defaultdict

import numpy as np
import trimesh


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    meshes = os.path.normpath(os.path.join(here, "..", "meshes"))
    ap = argparse.ArgumentParser()
    ap.add_argument("--tile", type=float, default=100.0, help="tile size (m)")
    ap.add_argument("--src", default=os.path.join(meshes, "shinjuku_visual.glb"))
    ap.add_argument("--out", default=os.path.join(meshes, "shinjuku_tiles"))
    ap.add_argument("--uri-prefix", default="rovo_worlds/meshes/shinjuku_tiles")
    args = ap.parse_args()

    print(f"loading {args.src} ...")
    scene = trimesh.load(args.src, process=False)
    if not isinstance(scene, trimesh.Scene):
        raise SystemExit("source is not a multi-mesh scene")

    # world-space centroid per geometry node
    nodes = []  # (node_name, geom_name, transform, world_centroid)
    cents = []
    for node in scene.graph.nodes_geometry:
        T, gname = scene.graph[node]
        g = scene.geometry[gname]
        cw = (T @ np.append(g.centroid, 1.0))[:3]
        nodes.append((node, gname, T))
        cents.append(cw)
    cents = np.array(cents)
    mn = cents.min(0)

    # bin by floor((c - min) / tile) on X,Y
    bins = defaultdict(list)
    for i, (node, gname, T) in enumerate(nodes):
        ix = int((cents[i, 0] - mn[0]) // args.tile)
        iy = int((cents[i, 1] - mn[1]) // args.tile)
        bins[(ix, iy)].append((node, gname, T))

    os.makedirs(args.out, exist_ok=True)
    # clear stale tiles
    for f in os.listdir(args.out):
        if f.startswith("tile_") and f.endswith(".glb"):
            os.remove(os.path.join(args.out, f))

    visuals = []
    total = 0
    for (ix, iy), members in sorted(bins.items()):
        tile = trimesh.Scene()
        for node, gname, T in members:
            tile.add_geometry(
                scene.geometry[gname], geom_name=gname,
                node_name=node, transform=T,
            )
        name = f"tile_{ix}_{iy}.glb"
        path = os.path.join(args.out, name)
        tile.export(path)
        sz = os.path.getsize(path)
        total += sz
        visuals.append((ix, iy, name))
        print(f"  {name}: {len(members)} submeshes, {sz/1e6:.1f} MB")

    print(f"\n{len(visuals)} tiles, {total/1e6:.0f} MB total")

    print("\n--- SDF <visual> block (paste into shinjuku_environment link) ---")
    for ix, iy, name in visuals:
        print(
            f'        <visual name="tile_{ix}_{iy}">\n'
            f"          <geometry><mesh>\n"
            f"            <uri>{args.uri_prefix}/{name}</uri>\n"
            f"          </mesh></geometry>\n"
            f"        </visual>"
        )


if __name__ == "__main__":
    main()
