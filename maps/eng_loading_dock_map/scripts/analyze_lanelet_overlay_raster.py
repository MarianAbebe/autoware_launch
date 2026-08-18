#!/usr/bin/env python3
"""Raster-based lanelet2 <-> PCD overlay check (read-only, offline).

Builds a top-down occupancy raster of the point cloud, classifies cells as
drivable ground vs obstacle, then scores how well the lanelet2 road network
sits on drivable ground. Scans 2-D offsets to find the best-fit displacement.

Outputs:
  intermediate/georef/lanelet_overlay_raster.json
  results/overlay_<tag>.png   (top-down PCD + OSM + GNSS track)
"""

from __future__ import annotations

import json
import math
import re
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

MAP_ROOT = Path("/home/agxorin/autoware_data/maps/eng_loading_dock_map")
OSM = MAP_ROOT / "lanelet2_map.osm"
ARRAYS = MAP_ROOT / "intermediate/georef/georef_arrays.npz"
RESULTS = MAP_ROOT / "results"
OUT = MAP_ROOT / "intermediate/georef/lanelet_overlay_raster.json"

CLOUDS = {
    "rebuilt_current": MAP_ROOT / "pointcloud_map.pcd",
    "pre_rebuild_georef": MAP_ROOT / "pointcloud_map_georeferenced.pcd",
}

LAT0, LON0 = 51.08080935188234, -114.1331143659806
A = 6378137.0
F = 1.0 / 298.257223563
E2 = F * (2.0 - F)
CELL = 0.5


def ecef(lat, lon, h):
    la, lo = np.radians(lat), np.radians(lon)
    sp, cp, sl, cl = np.sin(la), np.cos(la), np.sin(lo), np.cos(lo)
    n = A / np.sqrt(1.0 - E2 * sp * sp)
    return np.column_stack([(n + h) * cp * cl, (n + h) * cp * sl, (n * (1 - E2) + h) * sp])


def local_cartesian(lat, lon):
    lat = np.asarray(lat, dtype=np.float64)
    lon = np.asarray(lon, dtype=np.float64)
    p = ecef(lat, lon, np.zeros_like(lat))
    p0 = ecef(np.array([LAT0]), np.array([LON0]), np.zeros(1))[0]
    la, lo = math.radians(LAT0), math.radians(LON0)
    sp, cp, sl, cl = math.sin(la), math.cos(la), math.sin(lo), math.cos(lo)
    rot = np.array([[-sl, cl, 0], [-sp * cl, -sp * sl, cp], [cp * cl, cp * sl, sp]])
    return (p - p0) @ rot.T


def read_pcd_xyz(path: Path) -> np.ndarray:
    raw = path.read_bytes()
    marker = b"DATA binary\n"
    i = raw.find(marker)
    header = raw[:i].decode("ascii", "replace")
    fields = re.search(r"FIELDS (.+)", header).group(1).split()
    sizes = [int(v) for v in re.search(r"SIZE (.+)", header).group(1).split()]
    counts = [int(v) for v in re.search(r"COUNT (.+)", header).group(1).split()]
    n = int(re.search(r"POINTS (\d+)", header).group(1))
    dt = []
    for f, s, c in zip(fields, sizes, counts):
        dt.append((f, f"<f{s}") if f in "xyz" else (f"p{len(dt)}", f"V{s * c}"))
    a = np.frombuffer(raw, dtype=np.dtype(dt), count=n, offset=i + len(marker))
    return np.column_stack([a["x"], a["y"], a["z"]]).astype(np.float64)


def parse_osm():
    nodes: dict[int, tuple[float, float]] = {}
    ways: dict[int, list[int]] = {}
    lanelets: list[tuple[int, int]] = []
    for _e, el in ET.iterparse(str(OSM), events=("end",)):
        if el.tag == "node":
            nodes[int(el.get("id"))] = (float(el.get("lat")), float(el.get("lon")))
            el.clear()
        elif el.tag == "way":
            ways[int(el.get("id"))] = [int(n.get("ref")) for n in el.findall("nd")]
            el.clear()
        elif el.tag == "relation":
            tags = {t.get("k"): t.get("v") for t in el.findall("tag")}
            if tags.get("type") == "lanelet":
                left = right = None
                for m in el.findall("member"):
                    if m.get("role") == "left":
                        left = int(m.get("ref"))
                    elif m.get("role") == "right":
                        right = int(m.get("ref"))
                if left is not None and right is not None:
                    lanelets.append((left, right))
            el.clear()
    return nodes, ways, lanelets


def densify(poly: np.ndarray, step: float = 0.5) -> np.ndarray:
    if len(poly) < 2:
        return poly
    out = []
    for a, b in zip(poly[:-1], poly[1:]):
        d = np.linalg.norm(b - a)
        k = max(int(d / step), 1)
        out.append(a + np.outer(np.linspace(0, 1, k, endpoint=False), b - a))
    out.append(poly[-1:])
    return np.vstack(out)


def build_raster(pts: np.ndarray, x0, y0, nx, ny):
    ix = ((pts[:, 0] - x0) / CELL).astype(np.int32)
    iy = ((pts[:, 1] - y0) / CELL).astype(np.int32)
    ok = (ix >= 0) & (ix < nx) & (iy >= 0) & (iy < ny)
    ix, iy, z = ix[ok], iy[ok], pts[ok, 2]
    lin = iy * nx + ix
    cnt = np.bincount(lin, minlength=nx * ny)
    zmax = np.full(nx * ny, -np.inf)
    zmin = np.full(nx * ny, np.inf)
    np.maximum.at(zmax, lin, z)
    np.minimum.at(zmin, lin, z)
    cnt = cnt.reshape(ny, nx)
    zmax = zmax.reshape(ny, nx)
    zmin = zmin.reshape(ny, nx)
    occupied = cnt > 0
    height = np.where(occupied, zmax - zmin, 0.0)
    obstacle = occupied & (height > 1.2)
    ground = occupied & (height < 0.4)
    return {"cnt": cnt, "zmax": zmax, "zmin": zmin, "occ": occupied, "obs": obstacle, "grd": ground}


def sample_mask(mask, xy, x0, y0, nx, ny):
    ix = ((xy[:, 0] - x0) / CELL).astype(np.int32)
    iy = ((xy[:, 1] - y0) / CELL).astype(np.int32)
    ok = (ix >= 0) & (ix < nx) & (iy >= 0) & (iy < ny)
    v = np.zeros(len(xy), dtype=bool)
    v[ok] = mask[iy[ok], ix[ok]]
    return v, ok


def scan_offsets(rast, xy, x0, y0, nx, ny, half=40.0, step=1.0):
    best = None
    grid = np.arange(-half, half + 1e-9, step)
    rows = []
    for dx in grid:
        for dy in grid:
            p = xy + np.array([dx, dy])
            grd, ok = sample_mask(rast["grd"], p, x0, y0, nx, ny)
            obs, _ = sample_mask(rast["obs"], p, x0, y0, nx, ny)
            occ, _ = sample_mask(rast["occ"], p, x0, y0, nx, ny)
            n_in = int(ok.sum())
            if n_in < 0.2 * len(xy):
                continue
            f_grd = float(grd.sum()) / n_in
            f_obs = float(obs.sum()) / n_in
            f_occ = float(occ.sum()) / n_in
            score = f_grd - f_obs
            rows.append((dx, dy, score, f_grd, f_obs, f_occ, n_in))
            if best is None or score > best[2]:
                best = rows[-1]
    return best, rows


def score_at(rast, xy, x0, y0, nx, ny, dx=0.0, dy=0.0):
    p = xy + np.array([dx, dy])
    grd, ok = sample_mask(rast["grd"], p, x0, y0, nx, ny)
    obs, _ = sample_mask(rast["obs"], p, x0, y0, nx, ny)
    occ, _ = sample_mask(rast["occ"], p, x0, y0, nx, ny)
    n_in = max(int(ok.sum()), 1)
    return {
        "dx_m": float(dx),
        "dy_m": float(dy),
        "n_sampled": int(len(xy)),
        "n_inside_raster": int(ok.sum()),
        "frac_ground": float(grd.sum()) / n_in,
        "frac_obstacle": float(obs.sum()) / n_in,
        "frac_any_return": float(occ.sum()) / n_in,
        "score": float(grd.sum() - obs.sum()) / n_in,
    }


def main() -> None:
    nodes, ways, lanelets = parse_osm()
    ids = np.fromiter(nodes.keys(), dtype=np.int64)
    ll = np.array([nodes[i] for i in ids])
    enu = local_cartesian(ll[:, 0], ll[:, 1])
    pos = {int(i): enu[k, :2] for k, i in enumerate(ids)}

    centerlines = []
    for lw, rw in lanelets:
        lp = np.array([pos[r] for r in ways.get(lw, []) if r in pos])
        rp = np.array([pos[r] for r in ways.get(rw, []) if r in pos])
        if len(lp) < 2 or len(rp) < 2:
            continue
        n = max(len(lp), len(rp), 2)
        li = np.linspace(0, 1, n)
        lpd = np.column_stack(
            [np.interp(li, np.linspace(0, 1, len(lp)), lp[:, d]) for d in range(2)]
        )
        rpd = np.column_stack(
            [np.interp(li, np.linspace(0, 1, len(rp)), rp[:, d]) for d in range(2)]
        )
        centerlines.append(densify(0.5 * (lpd + rpd), 0.5))
    center = np.vstack(centerlines) if centerlines else np.zeros((0, 2))

    boundary = []
    for lw, rw in lanelets:
        for w in (lw, rw):
            p = np.array([pos[r] for r in ways.get(w, []) if r in pos])
            if len(p) >= 2:
                boundary.append(densify(p, 0.5))
    boundary = np.vstack(boundary) if boundary else np.zeros((0, 2))

    arr = np.load(ARRAYS)
    gnss = arr["gnss_enu"][:, :2]

    result = {
        "projector": "LocalCartesian (exact, GeographicLib convention)",
        "origin": {"lat": LAT0, "lon": LON0, "alt_m": 0.0},
        "cell_m": CELL,
        "n_lanelets": len(lanelets),
        "n_centerline_samples_total": int(len(center)),
        "clouds": {},
    }

    for tag, path in CLOUDS.items():
        if not path.exists():
            continue
        pts = read_pcd_xyz(path)
        x0, y0 = pts[:, 0].min() - 1, pts[:, 1].min() - 1
        x1, y1 = pts[:, 0].max() + 1, pts[:, 1].max() + 1
        nx = int((x1 - x0) / CELL) + 1
        ny = int((y1 - y0) / CELL) + 1
        rast = build_raster(pts, x0, y0, nx, ny)

        inbox = (
            (center[:, 0] > x0 - 40)
            & (center[:, 0] < x1 + 40)
            & (center[:, 1] > y0 - 40)
            & (center[:, 1] < y1 + 40)
        )
        c_near = center[inbox]

        entry = {
            "pcd": str(path),
            "n_points": int(len(pts)),
            "bbox_min": pts.min(0).tolist(),
            "bbox_max": pts.max(0).tolist(),
            "raster_shape": [int(ny), int(nx)],
            "n_cells_occupied": int(rast["occ"].sum()),
            "n_cells_ground": int(rast["grd"].sum()),
            "n_cells_obstacle": int(rast["obs"].sum()),
            "n_centerline_samples_near": int(len(c_near)),
            "control_gnss_track_at_zero": score_at(rast, gnss, x0, y0, nx, ny),
            "lanelet_centerline_at_zero": (
                score_at(rast, c_near, x0, y0, nx, ny) if len(c_near) else None
            ),
        }
        if len(c_near) >= 50:
            best, rows = scan_offsets(rast, c_near, x0, y0, nx, ny, half=40.0, step=1.0)
            if best:
                entry["lanelet_centerline_best_offset"] = score_at(
                    rast, c_near, x0, y0, nx, ny, best[0], best[1]
                )
            rows_arr = np.array([r[:3] for r in rows])
            entry["offset_scan_score_stats"] = {
                "n_offsets": int(len(rows_arr)),
                "score_at_zero": float(
                    rows_arr[np.argmin(np.hypot(rows_arr[:, 0], rows_arr[:, 1])), 2]
                ),
                "score_max": float(rows_arr[:, 2].max()),
                "score_p50": float(np.median(rows_arr[:, 2])),
            }
        gbest, _ = scan_offsets(rast, gnss, x0, y0, nx, ny, half=20.0, step=1.0)
        if gbest:
            entry["control_gnss_track_best_offset"] = score_at(
                rast, gnss, x0, y0, nx, ny, gbest[0], gbest[1]
            )
        result["clouds"][tag] = entry

        # figure
        try:
            import matplotlib

            matplotlib.use("Agg")
            import matplotlib.pyplot as plt

            fig, ax = plt.subplots(figsize=(11, 11), dpi=120)
            dens = np.log1p(rast["cnt"])
            ax.imshow(
                dens,
                origin="lower",
                extent=[x0, x0 + nx * CELL, y0, y0 + ny * CELL],
                cmap="Greys",
                vmax=np.percentile(dens[dens > 0], 98) if (dens > 0).any() else 1,
            )
            if len(boundary):
                b = boundary[
                    (boundary[:, 0] > x0 - 30)
                    & (boundary[:, 0] < x1 + 30)
                    & (boundary[:, 1] > y0 - 30)
                    & (boundary[:, 1] < y1 + 30)
                ]
                ax.scatter(b[:, 0], b[:, 1], s=0.6, c="tab:blue", label="lanelet boundaries")
            if len(c_near):
                ax.scatter(
                    c_near[:, 0], c_near[:, 1], s=0.6, c="tab:green", label="lanelet centerlines"
                )
            ax.plot(gnss[:, 0], gnss[:, 1], "-", c="tab:red", lw=1.4, label="NovAtel GNSS track")
            ax.set_xlim(x0 - 20, x1 + 20)
            ax.set_ylim(y0 - 20, y1 + 20)
            ax.set_aspect("equal")
            ax.set_xlabel("East [m] (LocalCartesian)")
            ax.set_ylabel("North [m] (LocalCartesian)")
            ax.set_title(f"{tag}: PCD vs lanelet2_map.osm vs GNSS")
            ax.legend(loc="upper right", markerscale=12)
            RESULTS.mkdir(parents=True, exist_ok=True)
            fig.savefig(RESULTS / f"overlay_{tag}.png", bbox_inches="tight")
            plt.close(fig)
            entry["figure"] = str(RESULTS / f"overlay_{tag}.png")
        except Exception as exc:  # noqa: BLE001
            entry["figure_error"] = repr(exc)

    OUT.write_text(json.dumps(result, indent=2))
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
