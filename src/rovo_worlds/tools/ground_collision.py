#!/usr/bin/env python3
"""Bake a ground-only collision heightfield for shinjuku.

Why: the visual city is NOT flat (ground spans ~-2.7..+10 m world), so any
flat collision plane leaves the robot sinking into rising roads or floating
above falling ones. The full 326k-tri city STL as collision pinned physics
to RTF ~0.02 (tracked-robot contacts vs dense noisy mesh). A coarse ground
heightfield keeps real slopes while the robot only ever touches a handful
of large, near-flat triangles.

Method: area-sample the walkable-ish (|normal.z| > 0.7) faces of every visual
tile, ~4 samples/m2, and bin them into a 4 m grid by 0.5 m height bands,
recording which of each cell's 16 sub-cells a band occupies. Contiguous bands
merge into candidate SURFACES carrying a real coverage fraction, so a cell
knows the difference between a road under it and a 1.2 m railing clipping its
corner. Restrict candidates to the drivable band z in (-3, +30) (excludes the
stray underground geometry at z_min=-23), drop anything roofed over by a
>=80%-coverage slab (that is a building interior, not a street), then choose
one surface per cell by REACHABILITY: Dijkstra outward from the spawn cell,
ordered by accumulated |dz|, taking at each neighbour the candidate closest in
height within one CLIMB step. Cells the grow never reaches keep their highest
candidate, so buildings stay solid blocks the robot cannot drive into.

This replaced a single downward raycast per cell that kept the HIGHEST hit:
55% of cells were then decided by one ray, so any object that ray clipped
became a solid 4x4 m block on open road -- an 8 m tower at world (146,-90)
from a canopy, a 1.25 m wall at (210,-70) from a railing covering 10% of its
cell. Both are invisible to the velodyne (it raycasts the VISUAL), so the
robot drove into walls that were not there and lost odometry. Highest-hit was
itself a fix for lowest-hit, which sank the robot ~2 m into deck tops (world
(53,366) true 2.75 read as 0.44); no cell-local rule can satisfy both, which
is why the choice is now made by connectivity instead of by height.

Then fill interior holes by neighbour averaging, fill the outer apron with
-0.40 (the measured spawn road height), and FILL-SINKS: raise every enclosed
bowl (a patch lower than its lowest rim out to the map edge) up to that rim,
so the robot cannot drop into any hole/gap on drivable ground; capped at a
drivable ceiling so building-walled voids stay low instead of becoming
plateaus. No de-spike: an audit showed clipping "spikes" (cells taller than
their neighbours) only ever cut NARROW REAL surfaces the robot drives onto
(one-cell decks, ramps, walkways), sinking it 1-12 m. Steep level boundaries
get VERTICAL WALLS and lose their ramp (see below) so the tracks stop at a
cliff instead of climbing it. Export a binary STL in the shinjuku_environment
LINK frame (world - link pose) so the SDF collision needs no sub-pose and
stays aligned with the visual tiles.

Usage:
  ground_collision.py             # full tile scan (~25 s), writes STL
  ground_collision.py layers.npz  # reuse a saved layer scan
"""
import collections
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
APRON_Z = -0.40                       # fill height outside the city
WALL_TH = 1.0                         # neighbour dz > WALL_TH gets a vertical wall
FILL_CEIL = 10.0                     # fill-sinks only up to this drivable height

# Layer scan / surface selection.
#
# The old scan cast ONE ray per 4 m cell and kept its highest hit, so 55% of
# cells were decided by a single ray: any object that ray clipped -- a canopy
# 8 m up, a 1.2 m wide railing -- was promoted to a solid 4x4 m block sitting
# on open road. The robot then hit a wall that the velodyne (which raycasts
# the VISUAL) cannot see, deflected, and lost odometry.
#
# Instead measure how much of each cell a surface actually COVERS: sample the
# horizontal-ish faces by area, bin into 0.5 m height bands, and record which
# of the cell's 16 sub-cells each band occupies. Contiguous bands merge into
# one candidate surface with a real coverage fraction.
#
# Choosing among candidates cannot be done cell-locally -- that is the trap
# this file fell into twice. Highest-hit puts the canopy on the road; lowest
# hit puts the street under a raised deck (it once dragged deck tops 2 m down,
# e.g. world (53,366) true 2.75); median splits the difference and is wrong
# for both. The surface the robot drives on is defined by CONNECTIVITY, not by
# height: region-grow from the spawn cell and, at each neighbour, take the
# candidate closest in height to the cell we came from, within one CLIMB step.
# Street stays street, a deck reached by a ramp stays a deck, and a canopy 8 m
# overhead is never within a step of anything. Cells the grow never reaches
# (building interiors, sealed voids) keep their highest candidate, so
# buildings stay solid and the robot cannot drive into one.
SUB = 4                               # 4x4 = 16 sub-cells per cell (1 m each)
DENS = 4.0                            # surface samples per m2
BAND = 0.5                            # height band for grouping samples
Z_LO, Z_HI = -10.0, 32.0              # band range
# Drivable band. Shinjuku has stray geometry down to z=-23 and a genuine
# sub-street level; the original scan excluded both with a hard (-3, 30) hit
# filter and dropping it let the grow settle 13785 cells onto underground
# floors (world (-366,404) went 26.29 -> -7.23). Candidates outside this band
# are not surfaces the robot is ever meant to stand on.
GROUND_LO, GROUND_HI = -3.0, 30.0
FLAT_N = 0.7                          # |normal.z| above this = walkable-ish
MIN_COV = 0.25                        # candidate must cover this much of a cell
# A surface is only standable if it is not roofed over. A canopy covers part
# of a cell (the one at world (146,-90) covers 38%); a building's floor slab
# covers all of it. So a candidate with ANY >=CEIL_COV surface above it is
# ineligible -- without this the grow walked into ground floors and put the
# robot inside buildings the old mesh correctly kept solid (world (-318,144):
# four stacked 100%-coverage slabs). The topmost candidate always survives,
# so buildings still read as solid.
#
# CEIL_GAP is only a tolerance for "above", NOT a headroom allowance: an
# earlier 1.5 m value excluded slabs high overhead while ADMITTING the ones
# 0.25-1.5 m up, which is the worst case -- the robot cannot fit under them
# and ends up standing 1.5 m below a road that covers it, buried in the
# surface (2291 such cells map-wide). Keep it just large enough that one
# surface's own residue cannot roof itself.
CEIL_COV = 0.8
CEIL_GAP = 0.15
# Lattice reclaim: a road only counts as one worth reopening if it covers
# this much of its cell BY AREA. Occupancy is not enough here -- the whole
# point of the pass is that occupancy overstates sparse surfaces.
LATTICE_ROAD_COV = 0.5
# Covered-street reclaim. A street running under a building podium or a
# skywalk is roofed by a slab that IS solid (area coverage ~1.0), so the
# lattice test above cannot help it, and the deck sits above OVERPASS_MAX so
# the underpass reclaim will not either -- world (82,120): a 6.03 road with
# 11 m of headroom, roofed at 17.17 and 27.01, sealed to 27.01 across a
# 2-cell run with open road at the same height on both ends.
#
# Height cannot separate this from a building ground floor, so do not try:
# gate on the STRUCTURE instead. A covered street is a SHORT sealed run
# bridging two stretches of road that sit at the SAME level; a building
# footprint is wider than the run bound and rarely has matching road on both
# opposite sides. These bounds are deliberately tighter than the underpass
# reclaim's (which additionally has the OVERPASS_MAX height cap to lean on).
COVERED_SPAN_MAX = 2                  # max sealed cells between the two roads
COVERED_ROAD_COV = 0.9                # the road must really be a road, by area
COVERED_LEVEL_TOL = 0.3               # the two roads must be the same level
MAX_REOPEN_ROUNDS = 8                 # fixed-point cap for the reopen passes
# Multi-level bake. Everything above picks ONE surface per cell, and no
# threshold can fix a two-level street: whichever level wins, the other
# becomes a wall (from below) or a pit (from above). world (-9,81) is the
# case -- a -0.8 street running 32 m east under a 6.06 deck, both real, both
# drivable. Widening the underpass reclaim to cover it would punch the deck
# into a 6.9 m hole, which is the regression 6712451 fixed.
#
# So emit BOTH. An STL has no trouble with overhangs; the heightfield is this
# script's own convention, not a gz limit. The deck keeps the heightfield and
# the street under it gets its own flat tiles, joined to the open ground at
# each end. Two rules make it safe: a lower surface must be REACHABLE (the
# 2-layer flood below, so a building's basement slab never appears), and the
# cliff wall at the deck edge is lifted to leave the mouth open instead of
# curtaining it off -- without that the tiles exist but nothing can drive in.
MULTI_ROAD_COV = 0.5                  # lower surface must be real, by area
LOWER_EDGE_WALL = 1.0                 # curb where a lower tile runs out
# An OVERPASS is a road with a bridge over it: the ray finds a low road, real
# headroom, then a single thin deck. That deck covers the road just like a
# building floor does, so the CEIL_COV roof test above deletes the road and
# leaves only the deck -- the underpass fills solid to the deck and walls off
# an open road the robot must drive down (world (-137,60): a 0.5 m road roofed
# by a 6.5 m deck read as a 6.5 m wall). Keep a roofed candidate when the ONLY
# thing over it is a single slab clearing HEADROOM (a deck the robot fits
# under); still delete it when the roof is close (buried, cannot fit), is a
# STACK of slabs (a building's floors, never drive-through), or sits above
# OVERPASS_MAX (a building's single roof slab, not a bridge -- without this
# cap a lone 25 m roof read as an overpass and opened the building footprint).
# A deck's own bottom and top faces are two candidates ~1 m apart, so merge
# slabs within SLAB_TH before counting them as one structure.
HEADROOM = 2.5                       # a lone deck this far up still clears
OVERPASS_MAX = 12.0                  # a deck higher than this is a building roof
SLAB_TH = 2.0                        # slabs within this merge into one deck
OVERLAP_TH = 0.5                      # sub-cell overlap that splits two bands
CLIMB = WALL_TH                       # max step the region-grow may cross
RECLAIM_D = 6                         # search radius (cells) for a spanned road
SPAN_MAX = 4                          # max deck run (cells) between two roads
SEED_XY = (0.099, 467.590)            # spawn: known street level
CX0, CY0 = -175, -155                 # grid origin in cell units (generous)
NXC, NYC = 340, 350


NB = int(round((Z_HI - Z_LO) / BAND))
POPCNT = np.array([bin(i).count("1") for i in range(1 << (SUB * SUB))],
                  dtype=np.uint8)


def scan_layers():
    """Area-sample every tile's walkable-ish faces into per-cell height bands.

    Returns (occ, zsum, zcnt), each shaped (NXC, NYC, NB). occ[i,j,b] is a
    bitmask of which of the cell's 16 sub-cells that band occupies, so the
    coverage of a surface is a real area fraction rather than a ray count.
    """
    occ = np.zeros(NXC * NYC * NB, dtype=np.uint16)
    zsum = np.zeros(NXC * NYC * NB, dtype=np.float64)
    zcnt = np.zeros(NXC * NYC * NB, dtype=np.int32)
    for path in sorted(glob.glob(TILES)):
        m = trimesh.load(path, force="mesh")
        if m.is_empty:
            continue
        m = trimesh.Trimesh(vertices=m.vertices + OFF, faces=m.faces,
                            process=False)
        flat = np.abs(m.face_normals[:, 2]) > FLAT_N
        if not flat.any():
            continue
        surf = trimesh.Trimesh(vertices=m.vertices, faces=m.faces[flat],
                               process=False)
        n = int(surf.area * DENS)
        if n < 1:
            continue
        p, _ = trimesh.sample.sample_surface(surf, n)
        ci = np.floor(p[:, 0] / STEP).astype(int) - CX0
        cj = np.floor(p[:, 1] / STEP).astype(int) - CY0
        b = np.floor((p[:, 2] - Z_LO) / BAND).astype(int)
        ok = ((ci >= 0) & (ci < NXC) & (cj >= 0) & (cj < NYC)
              & (b >= 0) & (b < NB))
        if not ok.any():
            continue
        p, ci, cj, b = p[ok], ci[ok], cj[ok], b[ok]
        sx = np.clip(((p[:, 0] - (ci + CX0) * STEP) / (STEP / SUB)),
                     0, SUB - 1).astype(int)
        sy = np.clip(((p[:, 1] - (cj + CY0) * STEP) / (STEP / SUB)),
                     0, SUB - 1).astype(int)
        bit = (np.uint16(1) << (sx * SUB + sy).astype(np.uint16))
        idx = (ci * NYC + cj) * NB + b
        np.bitwise_or.at(occ, idx, bit)
        np.add.at(zsum, idx, p[:, 2])
        np.add.at(zcnt, idx, 1)
        print(f"  {os.path.basename(path):>14}: {n} samples")
    shape = (NXC, NYC, NB)
    return occ.reshape(shape), zsum.reshape(shape), zcnt.reshape(shape)


def cell_candidates(occ_c, zsum_c, zcnt_c):
    """Merge contiguous occupied bands of one cell into candidate surfaces.

    Returns a list of (height, coverage, area_coverage) ordered low to high.
    Coverage is the fraction of the cell's 16 sub-cells the merged surface
    occupies; area_coverage is the fraction of the cell its samples actually
    cover by AREA. The two differ for a sparse surface: a band fills a
    sub-cell if ANY sample lands in it, so beams, railings and canopy ribs
    spread across a cell score a full 1.0 occupancy on a fraction of the area.
    Occupancy is what the grow wants (it asks "is there something to stand
    on"); the ceiling test wants area (it asks "am I roofed in").
    """
    bands = np.flatnonzero(occ_c)
    zb = Z_LO + (bands + 0.5) * BAND
    bands = bands[(zb >= GROUND_LO) & (zb <= GROUND_HI)]
    if not len(bands):
        return []
    # Contiguity alone does NOT mean one surface. A road 1.16 m above a lower
    # deck lands in consecutive 0.5 m bands, and merging them averaged the two
    # into one candidate half a metre BELOW the road -- the robot then rested
    # on collision that sat under the visual road and looked sunk into it
    # (world (174,331): bands 4.17/4.83/5.33 fused to 4.79 vs a 5.33 road).
    # Tell them apart by WHICH sub-cells each band occupies: one sloped
    # surface puts a given sub-cell at exactly one height, so adjacent bands
    # of it are disjoint, while two stacked surfaces occupy the same sub-cells
    # twice. Overlap therefore means "separate surface", so split there.
    out = []
    run = [bands[0]]
    run_mask = int(occ_c[bands[0]])
    for b in bands[1:]:
        mb = int(occ_c[b])
        inter = POPCNT[run_mask & mb]
        smaller = min(POPCNT[run_mask], POPCNT[mb])
        if b == run[-1] + 1 and (smaller == 0
                                 or inter / float(smaller) <= OVERLAP_TH):
            run.append(b)
            run_mask |= mb
        else:
            out.append(run)
            run = [b]
            run_mask = mb
    out.append(run)
    res = []
    for run in out:
        mask = np.bitwise_or.reduce(occ_c[run])
        cov = POPCNT[mask] / float(SUB * SUB)
        cnt = zcnt_c[run].sum()
        if cnt == 0:
            continue
        # Samples are area-uniform at DENS per m2 (scan_layers sizes each
        # tile's draw by surface area), so the sample count over the run is a
        # direct area measure. Clip: a surface steeper than the FLAT_N cut
        # still has more area than footprint, and two coincident faces (a
        # deck's top and underside) both land in the run.
        acov = min(cnt / DENS / (STEP * STEP), 1.0)
        res.append((zsum_c[run].sum() / cnt, cov, acov))
    return res


def _standable(z, allc):
    """True if a robot can stand on surface z, not roofed by a building.

    Open sky above -> yes. A STACK of covering slabs (>=2 after merging a
    deck's twin faces) -> a building, no. A single covering slab -> yes only
    if it clears HEADROOM (an overpass deck the robot drives under), no if it
    is close overhead (the robot would be buried under it).
    """
    above = sorted(z2 for z2, cov2, _acov2 in allc
                   if z2 > z + CEIL_GAP and cov2 >= CEIL_COV)
    if not above:
        return True
    groups = 1 + sum(hi - lo > SLAB_TH for lo, hi in zip(above, above[1:]))
    if groups >= 2:
        return False
    return above[0] - z > HEADROOM and above[-1] <= OVERPASS_MAX


def pick_reachable(occ, zsum, zcnt):
    """Choose one surface per cell by growing outward from the spawn cell."""
    cands = {}
    for i, j in zip(*np.nonzero(occ.any(axis=2))):
        allc = cell_candidates(occ[i, j], zsum[i, j], zcnt[i, j])
        c = [(z, cov) for z, cov, _acov in allc
             if cov >= MIN_COV and _standable(z, allc)]
        if c:
            cands[(i, j)] = c
    H = np.full((NXC, NYC), np.nan)

    si = int(np.floor(SEED_XY[0] / STEP)) - CX0
    sj = int(np.floor(SEED_XY[1] / STEP)) - CY0
    if (si, sj) not in cands:
        raise SystemExit(f"ERROR: spawn cell {SEED_XY} has no candidate "
                         f"surface; cannot seed the region grow")
    # Seed on the lowest substantial surface at spawn: that is the street.
    seed = min((c for c in cands[(si, sj)] if c[1] >= 0.5),
               key=lambda c: c[0], default=cands[(si, sj)][0])

    # Dijkstra on accumulated |dz| from the seed, NOT plain BFS. Shinjuku has
    # a lower level under the streets, joined to them by real ramps, so many
    # cells are legitimately reachable at two heights and a FIFO grow lets
    # whichever it happens to touch first win -- that flooded the lower level
    # outward and dropped world (0,0) from 6.06 to -0.61, a 6.7 m pit in an
    # open road. Ordering by total climb means a cell reachable both as street
    # (flat, near-zero cost) and via a 6 m ramp down keeps the street; the
    # descent still wins wherever it is genuinely the only way in.
    pq = [(0.0, si, sj, seed[0])]
    while pq:
        cost, i, j, h = heapq.heappop(pq)
        if not np.isnan(H[i, j]):
            continue
        H[i, j] = h
        for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            a, b = i + di, j + dj
            if not (0 <= a < NXC and 0 <= b < NYC):
                continue
            if not np.isnan(H[a, b]) or (a, b) not in cands:
                continue
            for z, _cov in cands[(a, b)]:
                d = abs(z - h)
                if d <= CLIMB:
                    heapq.heappush(pq, (cost + d, a, b, z))

    # Anything the grow could not reach (building interiors, sealed voids)
    # keeps its highest surface, so it stays a solid block.
    reached = np.isfinite(H)
    n_reach = int(reached.sum())
    for (i, j), c in cands.items():
        if np.isnan(H[i, j]):
            H[i, j] = max(c, key=lambda t: t[0])[0]

    # Underpass reclaim. The grow assigns ONE surface per cell, so where a
    # flyover deck's flat region reaches the cells over a road more cheaply
    # than the road does, the deck wins and the underpass reads as a solid
    # wall the robot cannot pass (world (-45,95): a -0.66 m road under a 6 m
    # deck read as 6.06, a phantom wall across an open street). Pull such a
    # cell down to its road candidate -- but ONLY when the deck is a narrow
    # SPAN bridging a road: the GROW has road on both opposite sides (x or y)
    # AND the deck run between those roads is at most SPAN_MAX cells wide. A
    # flyover span over a street is thin; a real deck/plaza the robot drives
    # ON is wide, so it keeps its height (verified: 366,350 / 303,353 /
    # 142,366, and the wide deck at 176,325, untouched). Only drops between
    # HEADROOM and OVERPASS_MAX qualify, so a tall building over a passage is
    # never punched through.
    #
    # Two traps this must avoid, both of which flipped the robot by punching a
    # road pit into a solid deck: (1) mutating H in place let a reclaimed
    # cell's new road height seed its neighbours and the pocket cascaded
    # inward -- so the span test reads a FROZEN copy Hg; (2) merely finding
    # road SOMEWHERE on both sides fires inside a wide plaza flanked by streets
    # -- so the run width between the two roads is bounded by SPAN_MAX.
    Hg = H.copy()

    def _road_dist(i, j, zr, di, dj):
        for s in range(1, RECLAIM_D + 1):
            a, b = i + di * s, j + dj * s
            if not (0 <= a < NXC and 0 <= b < NYC):
                return None
            if np.isfinite(Hg[a, b]) and abs(Hg[a, b] - zr) <= CLIMB:
                return s
        return None

    n_reclaim = 0
    for (i, j), c in cands.items():
        h = Hg[i, j]
        lows = [z for z, _ in c if HEADROOM < h - z <= OVERPASS_MAX]
        if not lows:
            continue
        zr = max(lows)
        for (da, db), (ea, eb) in (((1, 0), (-1, 0)), ((0, 1), (0, -1))):
            dp = _road_dist(i, j, zr, da, db)
            dm = _road_dist(i, j, zr, ea, eb)
            if dp is not None and dm is not None and dp + dm - 1 <= SPAN_MAX:
                H[i, j] = zr
                n_reclaim += 1
                break
    # Lattice reclaim. _standable rejects a road roofed by a surface whose
    # sub-cell OCCUPANCY reaches CEIL_COV, and occupancy cannot distinguish a
    # solid slab from beams spanning the same cell: 16 ribs light all 16
    # sub-cells exactly as a floor does. Such a road is dropped from cands, so
    # the grow never reaches it, the cell falls back to its highest candidate
    # and fills solid -- a wall across a street with open sky through it
    # (world (82,118): a 6.03 road under a structure scoring 1.00 occupancy on
    # 0.77 of the cell by area, sealed to 27.01, and the robot wedged INSIDE
    # the block, unable to drive either way).
    #
    # Re-test those roofs by AREA and pull the road back. Deliberately a
    # post-pass, not a fix inside _standable: feeding area coverage to the
    # grow re-routes the Dijkstra globally (measured: 1853 cells move, 692 of
    # them UPWARD, i.e. new walls elsewhere). Here the grow is already final,
    # only cells it left SOLID are touched, and they can only come DOWN, so
    # nothing that currently drives can regress. Reads a FROZEN Hl for the
    # same reason the underpass reclaim reads Hg -- a cell reclaimed here must
    # not seed the next one and let a pocket cascade inward.
    # A covered street is reopened by the second test below. Both run to a
    # FIXED POINT: opening one cell of a passage gives its neighbour the
    # support the tests ask for, and a single sweep only ever peels off the
    # first cell -- that left a wall 4 m further down the same street (the
    # 27.01 block at world (82,120) survived while (82,118) opened). Each
    # round reads a frozen snapshot, so within a round no cell can seed
    # another; progress happens between rounds, never mid-sweep.
    open_ = reached.copy()
    Hpre = H.copy()
    reopened = []
    n_lattice = n_covered = 0
    for _round in range(MAX_REOPEN_ROUNDS):
        Hf, openf = H.copy(), open_.copy()

        def _spanned(i, j, zr):
            """Road at zr on both sides, sealed run between them bounded.

            "Road" is any cell whose chosen surface sits within one CLIMB step
            of zr, exactly as the underpass reclaim's _road_dist defines it --
            NOT "cell the grow reached". The street on the far side of a
            sealed run is usually cut off from the spawn region by that very
            run, so it is never grow-reached; requiring that found no anchor
            and the test could not fire at all. A block at a different height
            fails the height match anyway, which is what keeps this honest.
            """
            def reach(di, dj):
                for s in range(1, RECLAIM_D + 1):
                    a, b = i + di * s, j + dj * s
                    if not (0 <= a < NXC and 0 <= b < NYC):
                        return None, None
                    if np.isfinite(Hf[a, b]) and abs(Hf[a, b] - zr) <= CLIMB:
                        return s, Hf[a, b]
                return None, None
            for (da, db), (ea, eb) in (((1, 0), (-1, 0)), ((0, 1), (0, -1))):
                dp, hp = reach(da, db)
                dm, hm = reach(ea, eb)
                if (dp is not None and dm is not None
                        and dp + dm - 1 <= COVERED_SPAN_MAX
                        and abs(hp - hm) <= COVERED_LEVEL_TOL):
                    return True
            return False

        changed = 0
        for (i, j), _c in cands.items():
            if openf[i, j]:
                continue
            allc = cell_candidates(occ[i, j], zsum[i, j], zcnt[i, j])
            pick = kind = None
            for z, cov, acov in allc:
                if cov < MIN_COV or acov < LATTICE_ROAD_COV:
                    continue
                if _standable(z, allc):    # not the case these passes are for
                    continue
                if Hf[i, j] - z <= CLIMB:
                    continue               # not sealed above it anyway
                over = sorted(z2 for z2, _c2, _a2 in allc if z2 > z + CEIL_GAP)
                if over and over[0] - z <= HEADROOM:
                    continue               # cannot fit under whatever is up there
                solid = [z2 for z2, _c2, acov2 in allc
                         if z2 > z + CEIL_GAP and acov2 >= CEIL_COV]
                if not solid:
                    # Roofed only by sparse structure. Needs TWO open
                    # neighbours at road level: one means a dead-end notch cut
                    # into a deck, which is how the previous reclaim flipped
                    # the robot -- it drives in, no way out, no floor either
                    # side.
                    if sum(np.isfinite(Hf[i + di, j + dj])
                           and abs(Hf[i + di, j + dj] - z) <= CLIMB
                           for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1))
                           if 0 <= i + di < NXC and 0 <= j + dj < NYC) >= 2:
                        if pick is None or z > pick:
                            pick, kind = z, "lattice"
                elif acov >= COVERED_ROAD_COV and _spanned(i, j, z):
                    # Roofed by a genuinely solid slab, but the cell bridges
                    # two stretches of road at one level over a short run:
                    # a street under a podium, not a floor inside a building.
                    if pick is None or z > pick:
                        pick, kind = z, "covered"
            if pick is None:
                continue
            H[i, j] = pick
            open_[i, j] = True
            reopened.append((i, j))
            changed += 1
            if kind == "lattice":
                n_lattice += 1
            else:
                n_covered += 1
        if not changed:
            break

    # Reconnect, radius 1. A cell fenced in by the run just reopened may hold
    # a perfectly standable road that the grow simply never arrived at, so the
    # solid-fill gave it its highest candidate instead: world (82,126) sat at
    # 27.01 with a clean 6.03 road and no roof over it, walling the far end of
    # the street this pass just opened. Neither test above touches it -- both
    # only consider candidates _standable REJECTED.
    #
    # Strictly one cell deep, and deliberately not part of the fixed-point
    # loop. Letting this flood iterate re-grows the map through every pocket
    # the grow could not reach and drops 2899 cells, which is exactly the
    # "buildings stay solid because the grow never got in" guarantee above.
    # One ring finishes a passage without reopening districts.
    n_join = 0
    joined = []
    for i, j in reopened:
        h = H[i, j]
        for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            a, b = i + di, j + dj
            if not (0 <= a < NXC and 0 <= b < NYC) or (a, b) not in cands:
                continue
            for z, _cov in cands[(a, b)]:
                if abs(z - h) <= CLIMB and H[a, b] - z > CLIMB:
                    H[a, b] = z
                    n_join += 1
                    joined.append((a, b))
                    break

    # Every cell these passes lowered must end up with at least TWO
    # same-level neighbours -- a road you can drive through, not a notch cut
    # into a deck with a 15-27 m drop on three sides (which is how the earlier
    # reclaim flipped the robot). The lattice test checks this up front, but
    # the covered-street and rejoin tests cannot: they run before the rest of
    # the passage exists. So check once at the end and put anything still
    # stranded back where it was, repeating because reverting one cell can
    # strand its neighbour.
    touched = list(dict.fromkeys(reopened + joined))
    n_revert = 0
    while True:
        stranded = [(i, j) for i, j in touched
                    if np.isfinite(H[i, j]) and sum(
                        np.isfinite(H[i + di, j + dj])
                        and abs(H[i + di, j + dj] - H[i, j]) <= CLIMB
                        for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1))
                        if 0 <= i + di < NXC and 0 <= j + dj < NYC) < 2]
        if not stranded:
            break
        for i, j in stranded:
            H[i, j] = Hpre[i, j]
            touched.remove((i, j))
            n_revert += 1
    print(f"reopened {n_lattice} cells roofed only by sparse structure, "
          f"{n_covered} covered-street cells spanned by a solid deck, "
          f"{n_join} fenced-in roads rejoined, {n_revert} reverted as dead ends")

    print(f"surfaces: {len(cands)} cells with candidates, "
          f"{n_reach} reached by grow, {len(cands) - n_reach} left solid, "
          f"{n_reclaim} underpass cells reclaimed")
    return H


def lower_surfaces(occ, zsum, zcnt, H):
    """Second drivable surface per cell, under the one the heightfield kept.

    Reachability is decided on a TWO-LAYER graph: a node is (cell, surface),
    not a cell, and neighbouring nodes join when their heights are within one
    CLIMB step. Flooding that from the spawn keeps a street that runs under a
    deck (it rejoins open ground at both ends) and drops a slab sealed inside
    a building (nothing at its level to step onto). A cell-level flood cannot
    make that distinction -- it is the same connectivity argument the main
    grow rests on, one dimension up.
    """
    surf = {}
    for i, j in zip(*np.nonzero(occ.any(axis=2))):
        if not np.isfinite(H[i, j]):
            continue
        allc = cell_candidates(occ[i, j], zsum[i, j], zcnt[i, j])
        surf[(i, j)] = [H[i, j]] + [
            z for z, cov, acov in allc
            if cov >= MIN_COV and acov >= MULTI_ROAD_COV
            and z < H[i, j] - HEADROOM and _standable(z, allc)]

    si = int(np.floor(SEED_XY[0] / STEP)) - CX0
    sj = int(np.floor(SEED_XY[1] / STEP)) - CY0
    seen = {(si, sj, 0)}
    dq = collections.deque(seen)
    while dq:
        i, j, k = dq.popleft()
        z = surf[(i, j)][k]
        for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nb = surf.get((i + di, j + dj))
            if nb is None:
                continue
            for k2, z2 in enumerate(nb):
                if abs(z2 - z) <= CLIMB and (i + di, j + dj, k2) not in seen:
                    seen.add((i + di, j + dj, k2))
                    dq.append((i + di, j + dj, k2))

    L = np.full(H.shape, np.nan)
    for i, j, k in seen:
        if k == 0:                     # layer 0 is the heightfield itself
            continue
        z = surf[(i, j)][k]
        if np.isnan(L[i, j]) or z > L[i, j]:
            L[i, j] = z                # highest reachable surface under the deck
    print(f"multi-level: {int(np.isfinite(L).sum())} cells carry a reachable "
          f"second surface under the heightfield")
    return L


def main():
    cache = os.path.join(HERE, "shinjuku_layers.npz")
    if len(sys.argv) > 1:
        d = np.load(sys.argv[1])
        occ, zsum, zcnt = d["occ"], d["zsum"], d["zcnt"]
        print(f"loaded cached layer scan: {sys.argv[1]}")
    else:
        occ, zsum, zcnt = scan_layers()
        np.savez_compressed(cache, occ=occ, zsum=zsum, zcnt=zcnt)
        print(f"cached scan -> {cache} (reuse: ground_collision.py {cache})")

    H = pick_reachable(occ, zsum, zcnt)
    L = lower_surfaces(occ, zsum, zcnt, H)

    # Crop the generous allocation to the cells that actually carry geometry.
    live = np.isfinite(H)
    ii, jj = np.where(live)
    lo_i, hi_i, lo_j, hi_j = ii.min(), ii.max() + 1, jj.min(), jj.max() + 1
    H = H[lo_i:hi_i, lo_j:hi_j]
    L = L[lo_i:hi_i, lo_j:hi_j]
    x0, y0 = CX0 + lo_i, CY0 + lo_j
    nx, ny = H.shape
    print(f"grid {nx} x {ny} cells ({nx*STEP:.0f} x {ny*STEP:.0f} m), "
          f"{np.isnan(H).sum()} holes")

    # Interior holes: average of known 4-neighbours, repeated. Remaining
    # (outer apron, far from any geometry) gets the flat spawn-road height.
    holes = np.isnan(H)                   # cells the grow gave no surface
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
    # flood), applied to HOLE cells only (see the accept mask below). An
    # apron/unreachable pocket enclosed by higher ground is a bowl the robot
    # would drop into and disappear; filling every enclosed hole up to its
    # lowest rim removes such traps map-wide in one pass:
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
    # Only raise HOLE cells (apron / no grow surface). A cell the region-grow
    # already put on a real drivable surface must never be lifted: the layer
    # grow now resolves genuine low ground correctly, so filling a real sunken
    # lot or road to its rim just floats the robot ~1 m above the visible
    # ground it is driving on (world (-47,91): a -0.66 m road raised to 0.27,
    # 6698 float cells map-wide). Restricting the fill to holes removes every
    # float and reopens zero sinks (all old sink spots are grow-assigned).
    accept = (W <= FILL_CEIL) & holes
    n_fill = int(((W - H > 1e-6) & accept).sum())
    H = np.where(accept, W, H)
    print(f"fill-sinks: raised {n_fill} hole cells to their rim "
          f"(<= {FILL_CEIL:.0f} m)")

    # NOTE: no spike-clip. An earlier de-spike (pull cells >SPIKE above their
    # neighbour max down) was meant to drop trees/poles caught as the highest
    # hit, but an audit showed it only ever clipped NARROW REAL surfaces the
    # robot drives onto (one-cell decks, ramps, walkways) -> the robot reached
    # them and sank 1-12 m. All 95 sink cells were clip-induced, none from
    # hole-fill. Keeping the true highest-hit height there costs at most a
    # minor bump if a lone tree overhangs a road; that is far better than a
    # multi-metre sink on a surface the robot actually uses.

    # Heightfield -> SMOOTH shared-vertex mesh, with each cliff quad replaced
    # by flat sub-quads + vertical walls.
    #
    # Smooth continuous surface (one vertex per cell centre, quads between)
    # gives smooth track motion -- a flat-top per-cell mesh turns every
    # few-cm cell-to-cell height change on flat road into a tiny vertical
    # step the tracks bump and deflect on (no smooth movement). The price of
    # smoothness is a mild interpolation sag between cell centres near a
    # level step (~0.2-0.3 m dip), a 4 m-resolution limit; the big sinks are
    # already gone (highest-hit, no clip).
    #
    # A quad whose 4 corner cells span more than WALL_TH is a real level
    # boundary and CANNOT stay smooth: interpolating a >1 m drop across one
    # 4 m quad yields a 40-84 deg ramp the tracks climb. An earlier version
    # added a vertical wall there but LEFT the ramp in place, so every cliff
    # carried two coincident surfaces -- the contact set flipped between a
    # horizontal-ish and a vertical normal at the same XY each step, spiking
    # the IMU and wrecking odometry (~10k such sites map-wide). Those quads
    # are now REPLACED: 4 flat sub-quads (one per corner cell, covering that
    # corner's quadrant) joined by vertical walls on the internal seams. So
    # near a cliff every face is horizontal or vertical, and everywhere else
    # slope is bounded by atan(WALL_TH * sqrt(2) / STEP) = 19.5 deg (the bound
    # is diagonal: one triangle can carry WALL_TH/STEP in x and in y at once,
    # so it is not 14 deg). Stepped quads meet
    # neighbouring stepped quads exactly (shared corner heights); against a
    # smooth quad the seam is a zero-width vertical crack of at most WALL_TH,
    # which no downward contact can fall through.
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

    z00 = Hf[:-1, :-1]; z10 = Hf[1:, :-1]
    z01 = Hf[:-1, 1:];  z11 = Hf[1:, 1:]
    span = (np.maximum(np.maximum(z00, z10), np.maximum(z01, z11))
            - np.minimum(np.minimum(z00, z10), np.minimum(z01, z11)))
    stepq = span > WALL_TH
    smooth = ~stepq.ravel()
    faces = [f[smooth] for f in faces]

    extra_v, extra_f = [], []
    nv = [nx * ny]

    def _quad(p0, p1, p2, p3):
        """Append a quad (4 own vertices, 2 tris) per row of the inputs."""
        p = len(p0)
        if p == 0:
            return
        v = np.empty((p * 4, 3))
        v[0::4] = p0; v[1::4] = p1; v[2::4] = p2; v[3::4] = p3
        base = nv[0] + np.arange(p) * 4
        nv[0] += p * 4
        extra_v.append(v)
        extra_f.append(np.column_stack([base, base + 1, base + 2]))
        extra_f.append(np.column_stack([base, base + 2, base + 3]))

    def add_flat(xa, xb, ya, yb, z):
        _quad(np.column_stack([xa, ya, z]), np.column_stack([xb, ya, z]),
              np.column_stack([xb, yb, z]), np.column_stack([xa, yb, z]))

    def add_wall(xa, ya, xb, yb, za, zb):
        """Vertical wall along segment (xa,ya)-(xb,yb) spanning za..zb.
        Rows under 1 mm tall are dropped: they carry no contact and STL is
        float32, so a sub-mm wall collapses to a degenerate (NaN-normal) tri."""
        k = np.abs(za - zb) > 1e-3
        xa, ya, xb, yb = xa[k], ya[k], xb[k], yb[k]
        lo = np.minimum(za[k], zb[k]); hi = np.maximum(za[k], zb[k])
        _quad(np.column_stack([xa, ya, lo]), np.column_stack([xb, yb, lo]),
              np.column_stack([xb, yb, hi]), np.column_stack([xa, ya, hi]))

    qx0 = CXv[:-1, :-1][stepq]; qx1 = CXv[1:, :-1][stepq]
    qy0 = CYv[:-1, :-1][stepq]; qy1 = CYv[:-1, 1:][stepq]
    qxm = (qx0 + qx1) / 2; qym = (qy0 + qy1) / 2
    a00 = z00[stepq]; a10 = z10[stepq]; a01 = z01[stepq]; a11 = z11[stepq]

    add_flat(qx0, qxm, qy0, qym, a00)       # corner (i,   j  )
    add_flat(qxm, qx1, qy0, qym, a10)       # corner (i+1, j  )
    add_flat(qx0, qxm, qym, qy1, a01)       # corner (i,   j+1)
    add_flat(qxm, qx1, qym, qy1, a11)       # corner (i+1, j+1)

    # Cliff walls, with the underpass mouth left OPEN. Where the taller corner
    # carries a lower surface at the shorter corner's level, the drop is a
    # deck edge with a street running under it, so the wall starts HEADROOM
    # above that street instead of at it. Walling the full drop is what made
    # the lower street unreachable in the first place -- the tiles below would
    # exist but every way in would be curtained off.
    Lf = L - OFF[2]
    l00 = Lf[:-1, :-1]; l10 = Lf[1:, :-1]
    l01 = Lf[:-1, 1:];  l11 = Lf[1:, 1:]

    def _mouth(zA, zB, LA, LB):
        lo = np.minimum(zA, zB)
        hi = np.maximum(zA, zB)
        ltall = np.where(zA >= zB, LA, LB)
        opened = np.isfinite(ltall) & (np.abs(ltall - lo) <= CLIMB)
        return np.where(opened, np.minimum(lo + HEADROOM, hi), lo), hi

    s00, s10 = l00[stepq], l10[stepq]
    s01, s11 = l01[stepq], l11[stepq]
    w, h = _mouth(a00, a10, s00, s10)
    add_wall(qxm, qy0, qxm, qym, w, h)      # x seam, low  y half
    w, h = _mouth(a01, a11, s01, s11)
    add_wall(qxm, qym, qxm, qy1, w, h)      # x seam, high y half
    w, h = _mouth(a00, a01, s00, s01)
    add_wall(qx0, qym, qxm, qym, w, h)      # y seam, low  x half
    w, h = _mouth(a10, a11, s10, s11)
    add_wall(qxm, qym, qx1, qym, w, h)      # y seam, high x half

    # The lower level itself: flat tiles per quadrant, so a cell's full 4x4 m
    # footprint is covered by the four quads that touch it. Flat rather than
    # smooth-interpolated because these patches start and stop against decks
    # and buildings; an interpolated edge would ramp up into the wall.
    QX0 = CXv[:-1, :-1]; QX1 = CXv[1:, :-1]
    QY0 = CYv[:-1, :-1]; QY1 = CYv[:-1, 1:]
    QXM = (QX0 + QX1) / 2; QYM = (QY0 + QY1) / 2
    n_low = 0
    for lc, xa, xb, ya, yb in ((l00, QX0, QXM, QY0, QYM),
                               (l10, QXM, QX1, QY0, QYM),
                               (l01, QX0, QXM, QYM, QY1),
                               (l11, QXM, QX1, QYM, QY1)):
        m = np.isfinite(lc)
        if m.any():
            add_flat(xa[m], xb[m], ya[m], yb[m], lc[m])
            n_low += int(m.sum())

    # Seams of the lower level. Two tiles at the same level meet flush (the
    # wall collapses under add_wall's 1 mm cut); where a tile ends against
    # something that is NOT at its level -- a pier, a building, the far side
    # of a deck with no street under it -- it gets a curb so the robot cannot
    # drive off the edge into the void under the deck.
    for la, lb, za, zb, xa, ya, xb, yb in (
            (l00, l10, z00, z10, QXM, QY0, QXM, QYM),
            (l01, l11, z01, z11, QXM, QYM, QXM, QY1),
            (l00, l01, z00, z01, QX0, QYM, QXM, QYM),
            (l10, l11, z10, z11, QXM, QYM, QX1, QYM)):
        ea = np.where(np.isfinite(la), la, za)
        eb = np.where(np.isfinite(lb), lb, zb)
        joined = np.abs(ea - eb) <= CLIMB
        m = (np.isfinite(la) | np.isfinite(lb)) & joined
        if m.any():
            add_wall(xa[m], ya[m], xb[m], yb[m], ea[m], eb[m])
        for lc in (la, lb):
            m = np.isfinite(lc) & ~joined
            if m.any():
                add_wall(xa[m], ya[m], xb[m], yb[m],
                         lc[m], lc[m] + LOWER_EDGE_WALL)

    if extra_v:
        verts = np.vstack([verts] + extra_v)
        faces = faces + extra_f
    faces = np.concatenate(faces)
    mesh = trimesh.Trimesh(vertices=verts, faces=faces, process=False)

    # Invariant: no climbable ramp survives. Every face is either near-vertical
    # (a step wall) or within atan(WALL_TH / STEP) of horizontal.
    # Worst case is a diagonal gradient (e.g. z00=0, z10=WALL_TH, z11=0 gives
    # WALL_TH/STEP in both x and y), hence the sqrt(2).
    up = np.abs(mesh.face_normals[:, 2])
    ramp = np.degrees(np.arccos(np.clip(up[up > 0.1], 0, 1)))
    limit = np.degrees(np.arctan(WALL_TH * np.sqrt(2) / STEP))
    if ramp.size and ramp.max() > limit + 1.0:
        raise SystemExit(f"ERROR: max ground slope {ramp.max():.1f} deg "
                         f"exceeds limit {limit:.1f} deg")

    mesh.export(OUT)
    print(f"wrote {OUT}: {len(verts)} verts, {len(faces)} tris "
          f"({int(stepq.sum())} cliff quads rebuilt as flat+wall, "
          f"{n_low} lower-level tiles, "
          f"max ground slope {ramp.max():.1f} deg <= {limit:.1f}), "
          f"link-frame z range "
          f"[{verts[:, 2].min():.2f}, {verts[:, 2].max():.2f}]")


if __name__ == "__main__":
    main()
