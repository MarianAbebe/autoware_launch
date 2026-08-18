#!/usr/bin/env python3
"""Quantify lanelet2 road-corridor placement against the georeferenced PCD.

Robust version of the raster overlay test:
  * local ground surface via min-filter, so drivable/obstacle classification
    tolerates the vertical smear left by KISS drift
  * corridor samples restricted to a core region well inside the PCD footprint,
    so the offset scan compares a constant sample set
  * offset scan +-20 m at 0.5 m, peak location and sharpness reported

Outputs intermediate/georef/lanelet_corridor_fit.json and zoom figures.
"""

from __future__ import annotations

import json
import math
import re
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from scipy.ndimage import minimum_filter, uniform_filter

MAP_ROOT = Path("/home/agxorin/autoware_data/maps/eng_loading_dock_map")
OSM = MAP_ROOT / "lanelet2_map.osm"
ARRAYS = MAP_ROOT / "intermediate/georef/georef_arrays.npz"
RESULTS = MAP_ROOT / "results"
OUT = MAP_ROOT / "intermediate/georef/lanelet_corridor_fit.json"

CLOUDS = {
    "pre_rebuild_georef": MAP_ROOT / "pointcloud_map_georeferenced.pcd",
    "rebuilt_current": MAP_ROOT / "pointcloud_map.pcd",
}

LAT0, LON0 = 51.08080935188234, -114.1331143659806
A = 6378137.0
F = 1.0 / 298.257223563
E2 = F * (2.0 - F)
CELL = 0.5
SCAN_HALF = 20.0
SCAN_STEP = 0.5


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


def resample(poly: np.ndarray, n: int) -> np.ndarray:
    d = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(poly, axis=0), axis=1))])
    if d[-1] <= 0:
        return np.repeat(poly[:1], n, axis=0)
    u = np.linspace(0, d[-1], n)
    return np.column_stack([np.interp(u, d, poly[:, k]) for k in range(2)])


def corridor_samples(lanelets, ways, pos, along_step=1.0, n_across=9):
    """Sample the full drivable surface of every lanelet."""
    pts = []
    for lw, rw in lanelets:
        lp = np.array([pos[r] for r in ways.get(lw, []) if r in pos])
        rp = np.array([pos[r] for r in ways.get(rw, []) if r in pos])
        if len(lp) < 2 or len(rp) < 2:
            continue
        length = max(
            np.linalg.norm(np.diff(lp, axis=0), axis=1).sum(),
            np.linalg.norm(np.diff(rp, axis=0), axis=1).sum(),
        )
        n_along = max(int(length / along_step) + 1, 2)
        ls = resample(lp, n_along)
        rs = resample(rp, n_along)
        for w in np.linspace(0.12, 0.88, n_across):
            pts.append(ls * (1 - w) + rs * w)
    return np.vstack(pts) if pts else np.zeros((0, 2))


class Raster:
    def __init__(self, pts: np.ndarray):
        self.x0 = pts[:, 0].min() - 1.0
        self.y0 = pts[:, 1].min() - 1.0
        self.nx = int((pts[:, 0].max() + 1.0 - self.x0) / CELL) + 1
        self.ny = int((pts[:, 1].max() + 1.0 - self.y0) / CELL) + 1
        ix = ((pts[:, 0] - self.x0) / CELL).astype(np.int32)
        iy = ((pts[:, 1] - self.y0) / CELL).astype(np.int32)
        lin = iy * self.nx + ix
        cnt = np.bincount(lin, minlength=self.nx * self.ny).astype(np.float32)
        zmin = np.full(self.nx * self.ny, np.inf, dtype=np.float64)
        zmax = np.full(self.nx * self.ny, -np.inf, dtype=np.float64)
        np.minimum.at(zmin, lin, pts[:, 2])
        np.maximum.at(zmax, lin, pts[:, 2])
        self.cnt = cnt.reshape(self.ny, self.nx)
        self.zmin = zmin.reshape(self.ny, self.nx)
        self.zmax = zmax.reshape(self.ny, self.nx)
        self.occ = self.cnt > 0

        # local ground: min of zmin over an 11x11 (5.5 m) window, then smoothed
        zfill = np.where(self.occ, self.zmin, 1e6)
        gmin = minimum_filter(zfill, size=11, mode="nearest")
        gmin = np.where(gmin > 1e5, np.nan, gmin)
        valid = np.isfinite(gmin)
        filled = np.where(valid, np.nan_to_num(gmin), 0.0)
        num = uniform_filter(filled, size=9, mode="nearest")
        den = uniform_filter(valid.astype(np.float64), size=9, mode="nearest")
        self.ground = np.where(den > 1e-6, num / np.maximum(den, 1e-9), np.nan)

        above_min = self.zmin - self.ground
        above_max = self.zmax - self.ground
        self.drivable = self.occ & np.isfinite(self.ground) & (above_max < 0.6)
        self.tall = self.occ & np.isfinite(self.ground) & (above_min > 2.0)

    def idx(self, xy):
        ix = ((xy[:, 0] - self.x0) / CELL).astype(np.int64)
        iy = ((xy[:, 1] - self.y0) / CELL).astype(np.int64)
        ok = (ix >= 0) & (ix < self.nx) & (iy >= 0) & (iy < self.ny)
        return ix, iy, ok

    def sample(self, mask, xy):
        ix, iy, ok = self.idx(xy)
        v = np.zeros(len(xy), dtype=bool)
        v[ok] = mask[iy[ok], ix[ok]]
        return v, ok


def score(rast: Raster, xy: np.ndarray, dx=0.0, dy=0.0) -> dict:
    p = xy + np.array([dx, dy])
    drv, ok = rast.sample(rast.drivable, p)
    tall, _ = rast.sample(rast.tall, p)
    occ, _ = rast.sample(rast.occ, p)
    n = max(int(ok.sum()), 1)
    n_occ = max(int(occ.sum()), 1)
    return {
        "dx_m": float(dx),
        "dy_m": float(dy),
        "n_in_bounds": int(ok.sum()),
        "frac_covered": float(occ.sum()) / n,
        "frac_drivable": float(drv.sum()) / n,
        "frac_tall": float(tall.sum()) / n,
        "drivable_given_covered": float(drv.sum()) / n_occ,
        "tall_given_covered": float(tall.sum()) / n_occ,
        "score": float(drv.sum() - tall.sum()) / n,
    }


def main() -> None:
    nodes, ways, lanelets = parse_osm()
    ids = np.fromiter(nodes.keys(), dtype=np.int64)
    ll = np.array([nodes[i] for i in ids])
    enu = local_cartesian(ll[:, 0], ll[:, 1])
    pos = {int(i): enu[k, :2] for k, i in enumerate(ids)}
    corr = corridor_samples(lanelets, ways, pos)

    gnss = np.load(ARRAYS)["gnss_enu"][:, :2]

    result = {
        "projector": "LocalCartesian exact, origin lat/lon from georef report, h0=0",
        "cell_m": CELL,
        "scan_half_m": SCAN_HALF,
        "scan_step_m": SCAN_STEP,
        "n_lanelets": len(lanelets),
        "n_corridor_samples_all": int(len(corr)),
        "clouds": {},
    }

    for tag, path in CLOUDS.items():
        if not path.exists():
            continue
        pts = read_pcd_xyz(path)
        rast = Raster(pts)

        # core region: inside the footprint with a SCAN_HALF margin, and where
        # the cloud actually has data, so the offset scan is a fair comparison
        margin = SCAN_HALF + 2.0
        core = (
            (corr[:, 0] > rast.x0 + margin)
            & (corr[:, 0] < rast.x0 + rast.nx * CELL - margin)
            & (corr[:, 1] > rast.y0 + margin)
            & (corr[:, 1] < rast.y0 + rast.ny * CELL - margin)
        )
        c = corr[core]

        entry = {
            "pcd": str(path),
            "n_points": int(len(pts)),
            "n_corridor_samples_core": int(len(c)),
            "cells": {
                "occupied": int(rast.occ.sum()),
                "drivable": int(rast.drivable.sum()),
                "tall": int(rast.tall.sum()),
            },
            "gnss_control_at_zero": score(rast, gnss),
        }

        if len(c) >= 100:
            grid = np.arange(-SCAN_HALF, SCAN_HALF + 1e-9, SCAN_STEP)
            surf = np.zeros((len(grid), len(grid)))
            for i, dy in enumerate(grid):
                for j, dx in enumerate(grid):
                    s = score(rast, c, dx, dy)
                    surf[i, j] = s["score"]
            k = np.unravel_index(np.argmax(surf), surf.shape)
            best_dx, best_dy = float(grid[k[1]]), float(grid[k[0]])
            entry["at_zero_offset"] = score(rast, c)
            entry["best_offset"] = score(rast, c, best_dx, best_dy)
            zero_i = int(np.argmin(np.abs(grid)))
            entry["offset_scan"] = {
                "best_dx_m": best_dx,
                "best_dy_m": best_dy,
                "best_offset_magnitude_m": float(math.hypot(best_dx, best_dy)),
                "score_at_zero": float(surf[zero_i, zero_i]),
                "score_at_best": float(surf[k]),
                "score_min": float(surf.min()),
                "score_p50": float(np.median(surf)),
                "score_improvement_best_minus_zero": float(surf[k] - surf[zero_i, zero_i]),
                "peak_sharpness_frac_of_grid_within_1pct_of_peak": float(
                    np.mean(surf > surf[k] - 0.01 * abs(surf[k]))
                ),
            }
            np.save(RESULTS / f"corridor_score_surface_{tag}.npy", surf)

            try:
                import matplotlib

                matplotlib.use("Agg")
                import matplotlib.pyplot as plt

                fig, axes = plt.subplots(1, 2, figsize=(17, 8), dpi=120)
                ax = axes[0]
                ext = [rast.x0, rast.x0 + rast.nx * CELL, rast.y0, rast.y0 + rast.ny * CELL]
                img = np.zeros(rast.occ.shape + (3,))
                img[rast.occ] = [0.85, 0.85, 0.85]
                img[rast.drivable] = [0.55, 0.75, 1.0]
                img[rast.tall] = [0.25, 0.25, 0.25]
                ax.imshow(img, origin="lower", extent=ext)
                ax.scatter(c[:, 0], c[:, 1], s=0.5, c="tab:green", label="lanelet corridor (core)")
                ax.plot(gnss[:, 0], gnss[:, 1], c="tab:red", lw=1.3, label="GNSS track")
                ax.set_aspect("equal")
                ax.set_xlabel("East [m]")
                ax.set_ylabel("North [m]")
                ax.set_title(f"{tag}: drivable(blue)/tall(dark) vs lanelet corridor")
                ax.legend(loc="upper right", markerscale=10)

                ax2 = axes[1]
                im = ax2.imshow(
                    surf,
                    origin="lower",
                    extent=[-SCAN_HALF, SCAN_HALF, -SCAN_HALF, SCAN_HALF],
                    cmap="viridis",
                )
                ax2.plot(0, 0, "w+", ms=14, mew=2, label="zero offset")
                ax2.plot(best_dx, best_dy, "r x", ms=12, mew=2, label="best offset")
                ax2.set_xlabel("dx [m]")
                ax2.set_ylabel("dy [m]")
                ax2.set_title("corridor drivability score vs 2-D shift")
                ax2.legend()
                fig.colorbar(im, ax=ax2, shrink=0.8)
                fig.savefig(RESULTS / f"corridor_fit_{tag}.png", bbox_inches="tight")
                plt.close(fig)
                entry["figure"] = str(RESULTS / f"corridor_fit_{tag}.png")
            except Exception as exc:  # noqa: BLE001
                entry["figure_error"] = repr(exc)

        result["clouds"][tag] = entry

    OUT.write_text(json.dumps(result, indent=2))
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
