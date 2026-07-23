#!/usr/bin/env python3
"""Bake a ground-only collision heightfield for shinjuku.

Why: the visual city is NOT flat (ground spans ~-2.7..+10 m world), so any
flat collision plane leaves the robot sinking into rising roads or floating
above falling ones. The full 326k-tri city STL as collision pinned physics
to RTF ~0.02 (tracked-robot contacts vs dense noisy mesh). A coarse ground
heightfield keeps real slopes while the robot only ever touches a handful
of large, near-flat triangles.

Method: per visual tile, raycast straight down on a 4 m grid from world
z=+60, keep all hits, restrict to band z in (-3, +30) (excludes the stray
underground geometry at z_min=-23), take the LOWEST hit per cell = the
surface the robot would stand on (catches ground under overpasses; building
plinths become walls, which usefully block driving through blocks). Bin all
samples into a global 4 m grid (min per bin), fill interior holes by
neighbour averaging, fill the outer apron with -0.40 (the measured spawn
road height, matching the old flat plane). Export a binary STL in the
shinjuku_environment LINK frame (world - link pose) so the SDF collision
needs no sub-pose and stays aligned with the visual tiles.

Usage:
  ground_collision.py            # full raycast (~5 min), writes STL
  ground_collision.py cache.npz  # reuse a saved scan (pts: N x [x y z])
"""
import glob
import os
import sys

import numpy as np
import trimesh

HERE = os.path.dirname(os.path.abspath(__file__))
TILES = os.path.join(HERE, "..", "meshes", "shinjuku_tiles", "*.glb")
OUT = os.path.join(HERE, "..", "meshes", "shinjuku_ground_collision.stl")

OFF = np.array([50.28, 9.45, 7.46])   # link pose: world = mesh + OFF
STEP = 4.0                            # m grid
CAST_Z = 60.0                         # world z ray origin
BAND = (-3.0, 30.0)                   # world z hits kept (ground band)
APRON_Z = -0.40                       # fill height outside the city


def scan_tiles():
    pts = []
    for path in sorted(glob.glob(TILES)):
        m = trimesh.load(path, force="mesh")
        if m.is_empty:
            continue
        m = trimesh.Trimesh(vertices=m.vertices + OFF, faces=m.faces,
                            process=False)
        xmin, ymin = m.bounds[0][:2]
        xmax, ymax = m.bounds[1][:2]
        xs = np.arange(xmin + STEP / 2, xmax, STEP)
        ys = np.arange(ymin + STEP / 2, ymax, STEP)
        if len(xs) == 0 or len(ys) == 0:
            continue
        gx, gy = np.meshgrid(xs, ys)
        origins = np.column_stack([gx.ravel(), gy.ravel(),
                                   np.full(gx.size, CAST_Z)])
        dirs = np.tile([0.0, 0.0, -1.0], (len(origins), 1))
        locs, ridx, _ = m.ray.intersects_location(
            origins, dirs, multiple_hits=True)
        if len(locs) == 0:
            continue
        z = locs[:, 2]
        keep = (z > BAND[0]) & (z < BAND[1])
        z, ridx = z[keep], ridx[keep]
        ground = np.full(len(origins), np.nan)
        order = np.argsort(z)[::-1]        # descending: last write = min
        ground[ridx[order]] = z[order]
        ok = ~np.isnan(ground)
        pts.append(np.column_stack(
            [origins[ok, 0], origins[ok, 1], ground[ok]]))
        print(f"  {os.path.basename(path):>14}: {int(ok.sum())} cells")
    return np.vstack(pts)


def main():
    if len(sys.argv) > 1:
        pts = np.load(sys.argv[1])["pts"]
        print(f"loaded cached scan: {len(pts)} samples")
    else:
        pts = scan_tiles()
        print(f"scanned: {len(pts)} samples")

    # Global 4 m bins, min z per bin (per-tile grids overlap and are not
    # mutually aligned, so re-bin instead of trusting sample positions).
    ix = np.floor(pts[:, 0] / STEP).astype(int)
    iy = np.floor(pts[:, 1] / STEP).astype(int)
    x0, y0 = ix.min(), iy.min()
    nx, ny = ix.max() - x0 + 1, iy.max() - y0 + 1
    H = np.full((nx, ny), np.nan)
    order = np.argsort(pts[:, 2])[::-1]    # descending: last write = min
    H[ix[order] - x0, iy[order] - y0] = pts[order, 2]
    print(f"grid {nx} x {ny} cells ({nx*STEP:.0f} x {ny*STEP:.0f} m), "
          f"{np.isnan(H).sum()} holes")

    # Interior holes: average of known 4-neighbours, repeated. Remaining
    # (outer apron, far from any geometry) gets the flat spawn-road height.
    for _ in range(8):
        nanm = np.isnan(H)
        if not nanm.any():
            break
        acc = np.zeros_like(H)
        cnt = np.zeros_like(H)
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            sh = np.full_like(H, np.nan)
            src = H[max(0, -dx):nx - max(0, dx) or None,
                    max(0, -dy):ny - max(0, dy) or None]
            sh[max(0, dx):nx - max(0, -dx) or None,
               max(0, dy):ny - max(0, -dy) or None] = src
            good = ~np.isnan(sh)
            acc[good] += sh[good]
            cnt[good] += 1
        fill = nanm & (cnt > 0)
        H[fill] = acc[fill] / cnt[fill]
    H[np.isnan(H)] = APRON_Z

    # Despike: min-binning latches onto stray low geometry (gutters,
    # drains) making single-cell pits the tracks would dive into. Any cell
    # more than 0.5 m BELOW the median of its 8 neighbours is pulled up to
    # that median. One-sided: hills, plinth walls and kerbs are untouched.
    pad = np.pad(H, 1, mode="edge")
    neigh = np.stack([pad[1 + dx:nx + 1 + dx, 1 + dy:ny + 1 + dy]
                      for dx in (-1, 0, 1) for dy in (-1, 0, 1)
                      if (dx, dy) != (0, 0)])
    med = np.median(neigh, axis=0)
    pit = H < med - 0.5
    H[pit] = med[pit]
    print(f"despiked {int(pit.sum())} pit cells")

    # Wall retraction: cell-centre vertices turn every vertical wall (deck
    # vs road boundaries, ~6 m in places) into a one-cell 4 m ramp whose
    # foot intrudes into the visible road — tracks climb it and the robot
    # flips (seen at world (-46,-107): deck 6.9 west of x=-50, road 0.85
    # east, collision ramp reached x=-46). Snap any cell that has a
    # 4-neighbour more than 1.5 m LOWER down to its lowest such neighbour:
    # the ramp moves one cell INTO the upper deck, its foot lands at the
    # visual wall base and the road stays clear. Kerbs (<1.5 m) and real
    # street slopes are untouched; deck edges give way ~one cell early,
    # which is the safe direction (drop onto road, not flip on road).
    low = np.full_like(H, np.inf)
    for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
        sh = np.full_like(H, np.inf)
        src = H[max(0, -dx):nx - max(0, dx) or None,
                max(0, -dy):ny - max(0, dy) or None]
        sh[max(0, dx):nx - max(0, -dx) or None,
           max(0, dy):ny - max(0, -dy) or None] = src
        low = np.minimum(low, sh)
    wall = H > low + 1.5
    H[wall] = low[wall]
    print(f"retracted {int(wall.sum())} wall cells")

    # Heightfield -> mesh. Vertices at cell centres, world frame.
    xs = (np.arange(nx) + x0) * STEP + STEP / 2
    ys = (np.arange(ny) + y0) * STEP + STEP / 2
    gx, gy = np.meshgrid(xs, ys, indexing="ij")
    verts = np.column_stack([gx.ravel(), gy.ravel(), H.ravel()]) - OFF

    idx = np.arange(nx * ny).reshape(nx, ny)
    a = idx[:-1, :-1].ravel()
    b = idx[1:, :-1].ravel()
    c = idx[:-1, 1:].ravel()
    d = idx[1:, 1:].ravel()
    faces = np.concatenate([np.column_stack([a, b, d]),
                            np.column_stack([a, d, c])])
    mesh = trimesh.Trimesh(vertices=verts, faces=faces, process=False)
    mesh.export(OUT)
    print(f"wrote {OUT}: {len(verts)} verts, {len(faces)} tris, "
          f"link-frame z range [{verts[:, 2].min():.2f}, "
          f"{verts[:, 2].max():.2f}]")


if __name__ == "__main__":
    main()
