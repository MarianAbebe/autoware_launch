#!/usr/bin/env python3
"""Lanelet2 corridor placement vs PCD, v2 (control-validated classifier).

v1 used a min-filter ground estimate and a 0.6 m drivability threshold, which
the ~0.6 m ground smear in this cloud defeats (the driven GNSS track scored 0 %
drivable). v2 estimates ground as a smoothed 20th-percentile surface on a 2 m
grid and classifies with counts in absolute height bands, then validates the
classifier on the GNSS track before trusting the lanelet offset scan.
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np
from scipy.ndimage import uniform_filter

sys.path.insert(0, str(Path(__file__).parent))
from analyze_lanelet_corridor_fit import (  # noqa: E402
    ARRAYS,
    LAT0,
    LON0,
    RESULTS,
    corridor_samples,
    local_cartesian,
    parse_osm,
    read_pcd_xyz,
)

MAP_ROOT = Path("/home/agxorin/autoware_data/maps/eng_loading_dock_map")
OUT = MAP_ROOT / "intermediate/georef/lanelet_corridor_fit_v2.json"
CLOUDS = {
    "pre_rebuild_georef": MAP_ROOT / "pointcloud_map_georeferenced.pcd",
    "rebuilt_current": MAP_ROOT / "pointcloud_map.pcd",
}
CELL = 0.5
GCELL = 2.0
LOW_BAND = 1.0
HIGH_BAND = 2.5
SCAN_HALF = 20.0
SCAN_STEP = 0.5


class Grid:
    def __init__(self, pts: np.ndarray):
        self.x0 = pts[:, 0].min() - 2.0
        self.y0 = pts[:, 1].min() - 2.0
        self.nx = int((pts[:, 0].max() + 2.0 - self.x0) / CELL) + 1
        self.ny = int((pts[:, 1].max() + 2.0 - self.y0) / CELL) + 1
        gnx = int(self.nx * CELL / GCELL) + 2
        gny = int(self.ny * CELL / GCELL) + 2

        gx = ((pts[:, 0] - self.x0) / GCELL).astype(np.int64)
        gy = ((pts[:, 1] - self.y0) / GCELL).astype(np.int64)
        glin = gy * gnx + gx
        order = np.lexsort((pts[:, 2], glin))
        gl = glin[order]
        z = pts[order, 2]
        starts = np.searchsorted(gl, np.arange(gnx * gny), side="left")
        ends = np.searchsorted(gl, np.arange(gnx * gny), side="right")
        n = ends - starts
        pick = starts + (0.20 * np.maximum(n - 1, 0)).astype(np.int64)
        ground = np.full(gnx * gny, np.nan)
        has = n > 0
        ground[has] = z[pick[has]]
        ground = ground.reshape(gny, gnx)

        valid = np.isfinite(ground)
        num = uniform_filter(np.where(valid, ground, 0.0), size=5, mode="nearest")
        den = uniform_filter(valid.astype(np.float64), size=5, mode="nearest")
        gsm = np.where(den > 1e-9, num / np.maximum(den, 1e-12), np.nan)
        # fill remaining holes with a wider average
        num2 = uniform_filter(np.where(np.isfinite(gsm), np.nan_to_num(gsm), 0.0), 15, mode="nearest")
        den2 = uniform_filter(np.isfinite(gsm).astype(np.float64), 15, mode="nearest")
        self.ground_coarse = np.where(
            np.isfinite(gsm), gsm, np.where(den2 > 1e-9, num2 / np.maximum(den2, 1e-12), np.nan)
        )
        self.gnx, self.gny = gnx, gny

        gz = self.ground_at(pts[:, 0], pts[:, 1])
        rel = pts[:, 2] - gz
        ix = ((pts[:, 0] - self.x0) / CELL).astype(np.int64)
        iy = ((pts[:, 1] - self.y0) / CELL).astype(np.int64)
        lin = iy * self.nx + ix
        ok = np.isfinite(rel)
        n_all = np.bincount(lin[ok], minlength=self.nx * self.ny)
        n_low = np.bincount(lin[ok & (rel < LOW_BAND)], minlength=self.nx * self.ny)
        n_high = np.bincount(lin[ok & (rel > HIGH_BAND)], minlength=self.nx * self.ny)
        self.n_all = n_all.reshape(self.ny, self.nx)
        self.n_low = n_low.reshape(self.ny, self.nx)
        self.n_high = n_high.reshape(self.ny, self.nx)
        self.occ = self.n_all > 0
        self.drivable = (self.n_low >= 3) & (self.n_high == 0)
        self.tall = self.n_high >= 3

    def ground_at(self, x, y):
        gx = np.clip(((x - self.x0) / GCELL).astype(np.int64), 0, self.gnx - 1)
        gy = np.clip(((y - self.y0) / GCELL).astype(np.int64), 0, self.gny - 1)
        return self.ground_coarse[gy, gx]

    def sample(self, mask, xy):
        ix = ((xy[:, 0] - self.x0) / CELL).astype(np.int64)
        iy = ((xy[:, 1] - self.y0) / CELL).astype(np.int64)
        ok = (ix >= 0) & (ix < self.nx) & (iy >= 0) & (iy < self.ny)
        v = np.zeros(len(xy), dtype=bool)
        v[ok] = mask[iy[ok], ix[ok]]
        return v, ok


def score(g: Grid, xy, dx=0.0, dy=0.0) -> dict:
    p = xy + np.array([dx, dy])
    drv, ok = g.sample(g.drivable, p)
    tall, _ = g.sample(g.tall, p)
    occ, _ = g.sample(g.occ, p)
    n = max(int(ok.sum()), 1)
    nc = max(int(occ.sum()), 1)
    return {
        "dx_m": float(dx),
        "dy_m": float(dy),
        "n_in_bounds": int(ok.sum()),
        "frac_covered": float(occ.sum()) / n,
        "frac_drivable": float(drv.sum()) / n,
        "frac_tall": float(tall.sum()) / n,
        "drivable_given_covered": float(drv.sum()) / nc,
        "tall_given_covered": float(tall.sum()) / nc,
        "score_given_covered": float(drv.sum() - tall.sum()) / nc,
    }


def main() -> None:
    nodes, ways, lanelets = parse_osm()
    ids = np.fromiter(nodes.keys(), dtype=np.int64)
    ll = np.array([nodes[i] for i in ids])
    enu = local_cartesian(ll[:, 0], ll[:, 1])
    pos = {int(i): enu[k, :2] for k, i in enumerate(ids)}
    corr = corridor_samples(lanelets, ways, pos)
    gnss = np.load(ARRAYS)["gnss_enu"][:, :2]

    out = {
        "classifier": {
            "ground": "20th-pct z on 2 m grid, 5x5 smoothed",
            "low_band_m": LOW_BAND,
            "high_band_m": HIGH_BAND,
            "drivable": "n(z<g+1.0)>=3 and n(z>g+2.5)==0",
            "tall": "n(z>g+2.5)>=3",
        },
        "clouds": {},
    }

    for tag, path in CLOUDS.items():
        pts = read_pcd_xyz(path)
        g = Grid(pts)
        ctrl = score(g, gnss)
        margin = SCAN_HALF + 2.0
        core = (
            (corr[:, 0] > g.x0 + margin)
            & (corr[:, 0] < g.x0 + g.nx * CELL - margin)
            & (corr[:, 1] > g.y0 + margin)
            & (corr[:, 1] < g.y0 + g.ny * CELL - margin)
        )
        c = corr[core]

        grid = np.arange(-SCAN_HALF, SCAN_HALF + 1e-9, SCAN_STEP)
        surf = np.full((len(grid), len(grid)), -np.inf)
        for i, dy in enumerate(grid):
            for j, dx in enumerate(grid):
                s = score(g, c, dx, dy)
                surf[i, j] = s["score_given_covered"] * min(s["frac_covered"] / 0.10, 1.0)
        k = np.unravel_index(np.argmax(surf), surf.shape)
        bdx, bdy = float(grid[k[1]]), float(grid[k[0]])
        zi = int(np.argmin(np.abs(grid)))

        entry = {
            "pcd": str(path),
            "n_points": int(len(pts)),
            "cells": {
                "occupied": int(g.occ.sum()),
                "drivable": int(g.drivable.sum()),
                "tall": int(g.tall.sum()),
            },
            "control_gnss_track_at_zero": ctrl,
            "n_corridor_core": int(len(c)),
            "lanelet_at_zero": score(g, c),
            "lanelet_at_best": score(g, c, bdx, bdy),
            "offset_scan": {
                "best_dx_m": bdx,
                "best_dy_m": bdy,
                "best_offset_magnitude_m": float(math.hypot(bdx, bdy)),
                "objective_at_zero": float(surf[zi, zi]),
                "objective_at_best": float(surf[k]),
                "objective_p50": float(np.median(surf)),
                "objective_min": float(surf.min()),
            },
        }
        np.save(RESULTS / f"corridor_score_surface_v2_{tag}.npy", surf)

        try:
            import matplotlib

            matplotlib.use("Agg")
            import matplotlib.pyplot as plt

            fig, ax = plt.subplots(1, 2, figsize=(18, 8.5), dpi=120)
            ext = [g.x0, g.x0 + g.nx * CELL, g.y0, g.y0 + g.ny * CELL]
            img = np.ones(g.occ.shape + (3,))
            img[g.occ] = [0.88, 0.88, 0.88]
            img[g.drivable] = [0.60, 0.80, 1.00]
            img[g.tall] = [0.20, 0.20, 0.20]
            ax[0].imshow(img, origin="lower", extent=ext)
            ax[0].scatter(c[:, 0], c[:, 1], s=0.4, c="tab:green", label="lanelet corridor")
            ax[0].plot(gnss[:, 0], gnss[:, 1], c="tab:red", lw=1.3, label="GNSS track (control)")
            ax[0].set_aspect("equal")
            ax[0].set_xlabel("East [m]")
            ax[0].set_ylabel("North [m]")
            ax[0].set_title(f"{tag}: blue=drivable, dark=tall structure")
            ax[0].legend(loc="upper right", markerscale=12)
            im = ax[1].imshow(
                surf,
                origin="lower",
                extent=[-SCAN_HALF, SCAN_HALF, -SCAN_HALF, SCAN_HALF],
                cmap="viridis",
            )
            ax[1].plot(0, 0, "w+", ms=14, mew=2, label="zero offset")
            ax[1].plot(bdx, bdy, "rx", ms=12, mew=2, label="best offset")
            ax[1].set_xlabel("dx [m]")
            ax[1].set_ylabel("dy [m]")
            ax[1].set_title("corridor drivability objective vs shift")
            ax[1].legend()
            fig.colorbar(im, ax=ax[1], shrink=0.8)
            fig.savefig(RESULTS / f"corridor_fit_v2_{tag}.png", bbox_inches="tight")
            plt.close(fig)
            entry["figure"] = str(RESULTS / f"corridor_fit_v2_{tag}.png")
        except Exception as exc:  # noqa: BLE001
            entry["figure_error"] = repr(exc)

        out["clouds"][tag] = entry

    OUT.write_text(json.dumps(out, indent=2))
    print(json.dumps(out, indent=2))


if __name__ == "__main__":
    main()
