#!/usr/bin/env python3
"""Fuse a classified LAZ ground cloud and georeferenced DTM into Gazebo STL.

LAS classification 2 provides terrain detail; the DTM is the continuous base
surface and fills areas without ground returns. Both inputs must use the same
projected CRS and vertical units. Requires laspy[lazrs], numpy, rasterio,
scipy, and pyproj.
"""

import argparse
import math
import os
import struct
import sys

import laspy
import numpy as np
import rasterio
from rasterio.enums import Resampling


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("point_cloud", help="Input classified .las/.laz file")
    parser.add_argument("dem", help="Georeferenced single-band DTM GeoTIFF")
    parser.add_argument("output", help="Output binary STL path")
    parser.add_argument("--cell-size", type=float, default=0.5,
                        help="Output mesh spacing in meters (default: native DTM resolution, 0.5)")
    parser.add_argument("--chunk-size", type=int, default=2_000_000,
                        help="LAZ points decoded per chunk (default: 2 million)")
    parser.add_argument("--max-fill-distance", type=float, default=6.0,
                        help="Bridge only unsupported gaps up to this distance in meters (default: 6)")
    args = parser.parse_args()
    if args.cell_size <= 0 or args.max_fill_distance < 0:
        parser.error("--cell-size must be positive and --max-fill-distance nonnegative")

    with rasterio.open(args.dem) as dem:
        if dem.count != 1:
            raise SystemExit("DTM must have one elevation band")
        if dem.crs is None:
            raise SystemExit("DTM has no CRS")
        dem_crs = dem.crs
        bounds = dem.bounds
        target_nx = max(2, round((bounds.right - bounds.left) / args.cell_size))
        target_ny = max(2, round((bounds.top - bounds.bottom) / args.cell_size))
        dx = (bounds.right - bounds.left) / target_nx
        dy = (bounds.top - bounds.bottom) / target_ny
        dem_values = dem.read(
            1,
            out_shape=(target_ny, target_nx),
            resampling=Resampling.average,
            masked=True,
        )
        dem_heights = np.asarray(dem_values.filled(np.nan), dtype=np.float64)

    with laspy.open(args.point_cloud) as cloud:
        point_crs = cloud.header.parse_crs()
        if point_crs is None:
            raise SystemExit("Point cloud has no CRS")
        point_epsg = point_crs.to_epsg()
        dem_epsg = dem_crs.to_epsg()
        if ((point_epsg is not None and dem_epsg is not None and point_epsg != dem_epsg) or
                (point_epsg is None and dem_epsg is None and point_crs != dem_crs)):
            raise SystemExit(f"CRS mismatch: point cloud is {point_crs}; DTM is {dem_crs}")

        sums = np.zeros((target_ny, target_nx), dtype=np.float64)
        counts = np.zeros((target_ny, target_nx), dtype=np.uint32)
        ground_count = 0
        print(f"DTM: {target_nx} x {target_ny} cells at "
              f"{dx:.3f} x {dy:.3f} m; LAZ: {cloud.header.point_count:,} points",
              file=sys.stderr, flush=True)
        for chunk_number, points in enumerate(cloud.chunk_iterator(args.chunk_size), 1):
            ground = points.classification == 2
            if not np.any(ground):
                continue
            x = np.asarray(points.x[ground])
            y = np.asarray(points.y[ground])
            z = np.asarray(points.z[ground])
            cols = np.floor((x - bounds.left) / dx).astype(np.int64)
            rows = np.floor((bounds.top - y) / dy).astype(np.int64)
            valid = ((cols >= 0) & (cols < target_nx) &
                     (rows >= 0) & (rows < target_ny))
            flat = rows[valid] * target_nx + cols[valid]
            sums += np.bincount(flat, weights=z[valid],
                                minlength=target_nx * target_ny).reshape(target_ny, target_nx)
            counts += np.bincount(flat, minlength=target_nx * target_ny).reshape(
                target_ny, target_nx).astype(np.uint32)
            ground_count += int(np.count_nonzero(valid))
            if chunk_number % 10 == 0:
                print(f"Decoded {min(chunk_number * args.chunk_size, cloud.header.point_count):,} / "
                      f"{cloud.header.point_count:,}; in-extent ground returns {ground_count:,}",
                      file=sys.stderr, flush=True)

    cloud_valid = counts > 0
    if not np.any(cloud_valid):
        raise SystemExit("No class-2 ground returns overlap the DTM")
    cloud_heights = np.full((target_ny, target_nx), np.nan, dtype=np.float64)
    cloud_heights[cloud_valid] = sums[cloud_valid] / counts[cloud_valid]
    dem_valid = np.isfinite(dem_heights)
    overlap = cloud_valid & dem_valid
    if not np.any(overlap):
        raise SystemExit("The DTM and point cloud have no overlapping valid cells")

    # Remove a possible vertical-datum offset robustly, then let dense ground
    # returns add detail while the DTM supplies a continuous base and fallback.
    differences = cloud_heights[overlap] - dem_heights[overlap]
    vertical_offset = float(np.median(differences))
    p05, p95 = np.percentile(differences, [5, 95])
    alpha = counts.astype(np.float64) / (counts.astype(np.float64) + 32.0)
    alpha = np.minimum(alpha, 0.95)
    heights = dem_heights.copy()
    heights[overlap] += alpha[overlap] * (
        cloud_heights[overlap] - vertical_offset - dem_heights[overlap]
    )
    cloud_only = cloud_valid & ~dem_valid
    heights[cloud_only] = cloud_heights[cloud_only] - vertical_offset
    valid = dem_valid | cloud_valid
    if not np.any(valid):
        raise SystemExit("No valid elevations in the fused surface")

    # Keep the navigable footprint compact. Retain the principal contiguous
    # terrain footprint and bridge only small gaps; distant unsupported regions
    # must not turn into flat, invented terrain.
    rows, cols = np.nonzero(valid)
    row_min, row_max = int(rows.min()), int(rows.max())
    col_min, col_max = int(cols.min()), int(cols.max())
    heights = heights[row_min:row_max + 1, col_min:col_max + 1]
    valid = valid[row_min:row_max + 1, col_min:col_max + 1]
    try:
        from scipy.ndimage import distance_transform_edt, label
    except ImportError as exc:
        raise SystemExit("Footprint filtering and gap filling require scipy") from exc
    components, component_count = label(valid)
    component_sizes = np.bincount(components.ravel())
    component_sizes[0] = 0
    main_component = int(np.argmax(component_sizes))
    valid = components == main_component
    if component_count > 1:
        print(f"Keeping largest of {component_count} valid-data components",
              file=sys.stderr, flush=True)
    if np.any(~valid):
        distances, nearest = distance_transform_edt(
            ~valid, sampling=(dy, dx), return_distances=True, return_indices=True
        )
        heights[~valid] = heights[tuple(axis[~valid] for axis in nearest)]
        missing = int(np.count_nonzero(~valid))
        unsupported_max_distance = float(distances[~valid].max())
        supported = valid | (distances <= args.max_fill_distance)
    else:
        missing = 0
        unsupported_max_distance = 0.0
        supported = valid

    ny, nx = heights.shape
    full_center_col = (target_nx - 1) / 2
    full_center_row = (target_ny - 1) / 2
    center_col = full_center_col - col_min
    center_row = full_center_row - row_min
    c0, r0 = math.floor(center_col), math.floor(center_row)
    c1, r1 = min(c0 + 1, nx - 1), min(r0 + 1, ny - 1)
    fx, fy = center_col - c0, center_row - r0
    origin_z = float(
        heights[r0, c0] * (1 - fx) * (1 - fy) +
        heights[r0, c1] * fx * (1 - fy) +
        heights[r1, c0] * (1 - fx) * fy +
        heights[r1, c1] * fx * fy
    )
    heights -= origin_z

    # Binary STL is compact and Gazebo supports it for static triangle meshes.
    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
    p00_valid = supported[:-1, :-1]
    p10_valid = supported[:-1, 1:]
    p01_valid = supported[1:, :-1]
    p11_valid = supported[1:, 1:]
    tri_a_valid = p00_valid & p01_valid & p11_valid
    tri_b_valid = p00_valid & p11_valid & p10_valid
    triangle_count = int(np.count_nonzero(tri_a_valid) + np.count_nonzero(tri_b_valid))
    with open(args.output, "wb") as mesh:
        mesh.write(b"ROVO fused LAZ ground returns and DTM".ljust(80, b"\0"))
        mesh.write(struct.pack("<I", triangle_count))
        for row in range(ny - 1):
            y0 = (full_center_row - (row + row_min)) * dy
            y1 = y0 - dy
            for col in range(nx - 1):
                x0 = ((col + col_min) - full_center_col) * dx
                x1 = x0 + dx
                p00 = (x0, y0, float(heights[row, col]))
                p10 = (x1, y0, float(heights[row, col + 1]))
                p01 = (x0, y1, float(heights[row + 1, col]))
                p11 = (x1, y1, float(heights[row + 1, col + 1]))
                # Raster rows run southward; this winding keeps normals upward.
                triangles = []
                if tri_a_valid[row, col]:
                    triangles.append((p00, p01, p11))
                if tri_b_valid[row, col]:
                    triangles.append((p00, p11, p10))
                for a, b, c in triangles:
                    ab = np.subtract(b, a)
                    ac = np.subtract(c, a)
                    normal = np.cross(ab, ac)
                    normal /= np.linalg.norm(normal)
                    mesh.write(struct.pack("<12fH", *normal, *a, *b, *c, 0))

    center_x = (bounds.left + bounds.right) / 2
    center_y = (bounds.bottom + bounds.top) / 2
    print(f"Wrote {args.output}: {nx} x {ny} vertices, {triangle_count:,} triangles; "
          f"{ground_count:,} ground points; DTM/cloud overlap offset {vertical_offset:.3f} m "
          f"(5-95% {p05:.3f}..{p95:.3f} m); nearest-filled {missing:,} grid cells "
          f"for interpolation; unsupported cells left outside mesh (maximum gap "
          f"{unsupported_max_distance:.1f} m; bridge limit {args.max_fill_distance:.1f} m); "
          f"world origin UTM {center_x:.3f}, {center_y:.3f}, z={origin_z:.3f} m",
          flush=True)


if __name__ == "__main__":
    main()
