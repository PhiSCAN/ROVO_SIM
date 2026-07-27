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
underground geometry at z_min=-23), take the HIGHEST hit per cell. On open
drivable ground (road/deck, clear sky above) the highest hit IS the surface
the robot stands on; a raised plaza reads at its true top instead of being
dragged down to some lower stray hit (the old lowest-hit rule sank the robot
~2 m into deck tops, e.g. world (53,366) true 2.75 read as 0.44). Building
footprints read at their roof (non-drivable, harmless raised blocks). Bin all
samples into a global 4 m grid (MAX per bin), fill interior holes by
neighbour averaging, fill the outer apron with -0.40 (the measured spawn
road height), then FILL-SINKS: raise every enclosed bowl (a patch lower than
its lowest rim out to the map edge) up to that rim, so the robot cannot drop
into any hole/gap on drivable ground; capped at a drivable ceiling so
building-walled voids stay low instead of becoming plateaus. No de-spike: an
audit showed clipping "spikes" (cells taller
than their neighbours) only ever cut NARROW REAL surfaces the robot drives
onto (one-cell decks, ramps, walkways), sinking it 1-12 m; the collision now
equals the visual top at every sampled cell (verified 0 sink). Steep level
boundaries get VERTICAL WALLS (see below) so the tracks stop at a cliff
instead of climbing a ramp and toppling. Export a binary STL in the
shinjuku_environment LINK frame (world - link pose) so the SDF collision
needs no sub-pose and stays aligned with the visual tiles.

Usage:
  ground_collision.py            # full raycast (~5 min), writes STL
  ground_collision.py cache.npz  # reuse a saved scan (pts: N x [x y z])
"""
import glob
import heapq
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
WALL_TH = 1.0                         # neighbour dz > WALL_TH gets a vertical wall
FILL_CEIL = 10.0                      # fill-sinks only up to this drivable height


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
        order = np.argsort(z)             # ascending: last write = max (highest)
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
        cache = os.path.join(HERE, "shinjuku_scan.npz")
        np.savez_compressed(cache, pts=pts)
        print(f"cached scan -> {cache} (reuse: ground_collision.py {cache})")

    # Global 4 m bins, MAX z per bin (per-tile grids overlap and are not
    # mutually aligned, so re-bin instead of trusting sample positions).
    ix = np.floor(pts[:, 0] / STEP).astype(int)
    iy = np.floor(pts[:, 1] / STEP).astype(int)
    x0, y0 = ix.min(), iy.min()
    nx, ny = ix.max() - x0 + 1, iy.max() - y0 + 1
    H = np.full((nx, ny), np.nan)
    order = np.argsort(pts[:, 2])          # ascending: last write = max
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

    # Fill-sinks (hydrological depression fill, Planchon-Darboux / priority
    # flood). The robot "sinks" wherever it drives onto a cell that sits in a
    # BOWL -- a patch of ground lower than every path out of it. These are
    # holes/gaps in the visual city (sparse or missing tiles), single-ray
    # dropouts, or apron patches enclosed by higher deck: the highest-hit rule
    # read them low, the smooth grid ramps down into them, and the robot drops
    # in and disappears below the surface (e.g. world (366,350) a -1.0 cell in
    # a 3 m deck, (54,210) a -0.7 cell in a 6 m deck). Filling every enclosed
    # bowl up to its lowest rim removes ALL such traps map-wide in one pass:
    # a cell only rises if it is lower than the lowest lip it would have to
    # cross to reach the map edge. Real drivable descents keep their depth --
    # a through-road, a ramp, or a courtyard with any opening DRAINS to the
    # edge, so its spill height is its own height and it is left untouched;
    # only true pockets fill. The fill never rises above the rim, so it only
    # flattens holes (smoother tracks), never buries a step. Cap the accepted
    # fill at FILL_CEIL: a void ringed solely by tall BUILDINGS (roof rim ~27
    # m) would otherwise fill into a giant fake plateau, but the robot can
    # never drive into a fully walled void, so leave those low and only close
    # bowls whose rim is real drivable ground. Map edge is the only drain.
    edge = np.zeros((nx, ny), bool)
    edge[0, :] = edge[-1, :] = edge[:, 0] = edge[:, -1] = True
    W = np.full((nx, ny), np.inf)
    done = np.zeros((nx, ny), bool)
    pq = []
    ei, ej = np.where(edge)
    for i, j in zip(ei.tolist(), ej.tolist()):
        W[i, j] = H[i, j]
        done[i, j] = True
        heapq.heappush(pq, (H[i, j], i, j))
    while pq:
        z, i, j = heapq.heappop(pq)
        for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            a, b = i + di, j + dj
            if a < 0 or b < 0 or a >= nx or b >= ny or done[a, b]:
                continue
            nz = H[a, b] if H[a, b] > z else z
            W[a, b] = nz
            done[a, b] = True
            heapq.heappush(pq, (nz, a, b))
    accept = W <= FILL_CEIL
    n_fill = int(((W - H > 1e-6) & accept).sum())
    H = np.where(accept, W, H)
    print(f"fill-sinks: raised {n_fill} bowl cells to their rim "
          f"(<= {FILL_CEIL:.0f} m)")

    # NOTE: no spike-clip. An earlier de-spike (pull cells >SPIKE above their
    # neighbour max down) was meant to drop trees/poles caught as the highest
    # hit, but an audit showed it only ever clipped NARROW REAL surfaces the
    # robot drives onto (one-cell decks, ramps, walkways) -> the robot reached
    # them and sank 1-12 m. All 95 sink cells were clip-induced, none from
    # hole-fill. Keeping the true highest-hit height there costs at most a
    # minor bump if a lone tree overhangs a road; that is far better than a
    # multi-metre sink on a surface the robot actually uses.

    # Heightfield -> SMOOTH shared-vertex mesh + vertical walls at big steps.
    #
    # Smooth continuous surface (one vertex per cell centre, quads between)
    # gives smooth track motion -- a flat-top per-cell mesh turns every
    # few-cm cell-to-cell height change on flat road into a tiny vertical
    # step the tracks bump and deflect on (no smooth movement). The price of
    # smoothness is a mild interpolation sag between cell centres near a
    # level step (~0.2-0.3 m dip), a 4 m-resolution limit; the big sinks are
    # already gone (highest-hit, no clip). At real level boundaries
    # (neighbour dz > WALL_TH) add a VERTICAL wall so the tracks stop at the
    # cliff instead of climbing the smoothed ramp and toppling.
    xs = (np.arange(nx) + x0) * STEP + STEP / 2
    ys = (np.arange(ny) + y0) * STEP + STEP / 2
    gx, gy = np.meshgrid(xs, ys, indexing="ij")
    verts = np.column_stack([gx.ravel(), gy.ravel(), H.ravel()]) - OFF

    idx = np.arange(nx * ny).reshape(nx, ny)
    a = idx[:-1, :-1].ravel(); b = idx[1:, :-1].ravel()
    c = idx[:-1, 1:].ravel(); d = idx[1:, 1:].ravel()
    faces = [np.column_stack([a, b, d]), np.column_stack([a, d, c])]

    Hf = verts[:, 2].reshape(nx, ny)
    CXv = verts[:, 0].reshape(nx, ny)
    CYv = verts[:, 1].reshape(nx, ny)
    s = STEP / 2.0
    extra_v, extra_f = [], []

    def add_walls(hi, lo, xlo, ylo, xhi, yhi):
        n0 = nx * ny + sum(len(v) for v in extra_v)
        zlo = np.minimum(Hf[hi], Hf[lo]); zhi = np.maximum(Hf[hi], Hf[lo])
        p = len(np.atleast_1d(zlo))
        v = np.empty((p * 4, 3))
        v[0::4] = np.column_stack([xlo, ylo, zlo])
        v[1::4] = np.column_stack([xhi, yhi, zlo])
        v[2::4] = np.column_stack([xhi, yhi, zhi])
        v[3::4] = np.column_stack([xlo, ylo, zhi])
        base = n0 + np.arange(p) * 4
        extra_v.append(v)
        extra_f.append(np.column_stack([base, base + 1, base + 2]))
        extra_f.append(np.column_stack([base, base + 2, base + 3]))

    m = np.abs(Hf[:-1, :] - Hf[1:, :]) > WALL_TH        # +x steps
    if m.any():
        ii, jj = np.where(m)
        xb = (CXv[ii, jj] + CXv[ii + 1, jj]) / 2
        add_walls((ii, jj), (ii + 1, jj),
                  xb, CYv[ii, jj] - s, xb, CYv[ii, jj] + s)
    m = np.abs(Hf[:, :-1] - Hf[:, 1:]) > WALL_TH        # +y steps
    if m.any():
        ii, jj = np.where(m)
        yb = (CYv[ii, jj] + CYv[ii, jj + 1]) / 2
        add_walls((ii, jj), (ii, jj + 1),
                  CXv[ii, jj] - s, yb, CXv[ii, jj] + s, yb)

    if extra_v:
        verts = np.vstack([verts] + extra_v)
        faces = faces + extra_f
    n_walls = sum(len(f) for f in extra_f) // 2
    faces = np.concatenate(faces)
    mesh = trimesh.Trimesh(vertices=verts, faces=faces, process=False)
    mesh.export(OUT)
    print(f"wrote {OUT}: {len(verts)} verts, {len(faces)} tris "
          f"({n_walls} step walls), link-frame z range "
          f"[{verts[:, 2].min():.2f}, {verts[:, 2].max():.2f}]")


if __name__ == "__main__":
    main()
