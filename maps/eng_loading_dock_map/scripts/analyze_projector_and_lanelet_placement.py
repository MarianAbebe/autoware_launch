#!/usr/bin/env python3
"""Offline validation of map_projector_info.yaml and lanelet2_map.osm placement.

Read-only w.r.t. the map artifacts. Emits
intermediate/georef/projector_lanelet_placement_analysis.json.

Checks:
  A) Exact GeographicLib-style LocalCartesian vs the spherical ENU approximation
     that georeference_kiss_to_gnss.py used, over the map footprint.
  B) KISS -> GNSS alignment: path lengths, residuals for the transform in the
     report vs the transform the PCD rebuild actually applied, plus a proper
     similarity (Umeyama) solve.
  C) lanelet2_map.osm geographic bounds -> ENU through the projector origin.
  D) Overlay quality of the OSM against the rebuilt PCD and the GNSS track,
     including a 2-D translation search for the best-fit offset.
"""

from __future__ import annotations

import json
import math
import re
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

MAP_ROOT = Path("/home/agxorin/autoware_data/maps/eng_loading_dock_map")
GEOREF_REPORT = MAP_ROOT / "intermediate/georef/georef_alignment_report.json"
GEOREF_ARRAYS = MAP_ROOT / "intermediate/georef/georef_arrays.npz"
REBUILD_VALIDATION = MAP_ROOT / "intermediate/georef/rebuild_validation.json"
OSM = MAP_ROOT / "lanelet2_map.osm"
PCD = MAP_ROOT / "pointcloud_map.pcd"
OUT = MAP_ROOT / "intermediate/georef/projector_lanelet_placement_analysis.json"

WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)


# --------------------------------------------------------------------------- #
# geodesy
# --------------------------------------------------------------------------- #
def lla_to_ecef(lat_deg, lon_deg, h):
    lat = np.radians(np.asarray(lat_deg, dtype=np.float64))
    lon = np.radians(np.asarray(lon_deg, dtype=np.float64))
    h = np.asarray(h, dtype=np.float64)
    sp, cp = np.sin(lat), np.cos(lat)
    sl, cl = np.sin(lon), np.cos(lon)
    n = WGS84_A / np.sqrt(1.0 - WGS84_E2 * sp * sp)
    x = (n + h) * cp * cl
    y = (n + h) * cp * sl
    z = (n * (1.0 - WGS84_E2) + h) * sp
    return np.column_stack([x, y, z])


def local_cartesian(lat, lon, h, lat0, lon0, h0):
    """Exact GeographicLib LocalCartesian / lanelet2 LocalCartesianProjector."""
    p = lla_to_ecef(lat, lon, h)
    p0 = lla_to_ecef([lat0], [lon0], [h0])[0]
    d = p - p0
    lat0r, lon0r = math.radians(lat0), math.radians(lon0)
    sp, cp = math.sin(lat0r), math.cos(lat0r)
    sl, cl = math.sin(lon0r), math.cos(lon0r)
    rot = np.array(
        [
            [-sl, cl, 0.0],
            [-sp * cl, -sp * sl, cp],
            [cp * cl, cp * sl, sp],
        ]
    )
    return d @ rot.T


def spherical_enu(lat, lon, alt, lat0, lon0, alt0):
    """The approximation used inside georeference_kiss_to_gnss.py."""
    lat = np.asarray(lat, dtype=np.float64)
    lon = np.asarray(lon, dtype=np.float64)
    alt = np.asarray(alt, dtype=np.float64)
    r = 6378137.0
    lat0r, lon0r = math.radians(lat0), math.radians(lon0)
    east = (np.radians(lon) - lon0r) * math.cos(lat0r) * r
    north = (np.radians(lat) - lat0r) * r
    return np.column_stack([east, north, alt - alt0])


def enu_to_lla_spherical(east, north, lat0, lon0):
    r = 6378137.0
    lat = np.degrees(np.asarray(north, dtype=np.float64) / r + math.radians(lat0))
    lon = np.degrees(
        np.asarray(east, dtype=np.float64) / (r * math.cos(math.radians(lat0)))
        + math.radians(lon0)
    )
    return lat, lon


def utm_zone_and_band(lat, lon):
    zone = int(math.floor((lon + 180.0) / 6.0)) + 1
    bands = "CDEFGHJKLMNPQRSTUVWX"
    band = bands[int(math.floor((lat + 80.0) / 8.0))]
    return zone, band


def mgrs_100km_square(lat, lon):
    """100 km square ID for the WGS84 UTM zone (enough to detect grid spans)."""
    zone, band = utm_zone_and_band(lat, lon)
    lat_r, lon_r = math.radians(lat), math.radians(lon)
    lon0 = math.radians((zone - 1) * 6 - 180 + 3)
    k0 = 0.9996
    e2 = WGS84_E2
    ep2 = e2 / (1 - e2)
    n = WGS84_A / math.sqrt(1 - e2 * math.sin(lat_r) ** 2)
    t = math.tan(lat_r) ** 2
    c = ep2 * math.cos(lat_r) ** 2
    a_ = math.cos(lat_r) * (lon_r - lon0)
    m = WGS84_A * (
        (1 - e2 / 4 - 3 * e2**2 / 64 - 5 * e2**3 / 256) * lat_r
        - (3 * e2 / 8 + 3 * e2**2 / 32 + 45 * e2**3 / 1024) * math.sin(2 * lat_r)
        + (15 * e2**2 / 256 + 45 * e2**3 / 1024) * math.sin(4 * lat_r)
        - (35 * e2**3 / 3072) * math.sin(6 * lat_r)
    )
    easting = (
        k0
        * n
        * (
            a_
            + (1 - t + c) * a_**3 / 6
            + (5 - 18 * t + t**2 + 72 * c - 58 * ep2) * a_**5 / 120
        )
        + 500000.0
    )
    northing = k0 * (
        m
        + n
        * math.tan(lat_r)
        * (
            a_**2 / 2
            + (5 - t + 9 * c + 4 * c**2) * a_**4 / 24
            + (61 - 58 * t + t**2 + 600 * c - 330 * ep2) * a_**6 / 720
        )
    )
    col_letters = ["ABCDEFGH", "JKLMNPQR", "STUVWXYZ"][(zone - 1) % 3]
    col = col_letters[int(easting // 100000) - 1]
    row_letters = "ABCDEFGHJKLMNPQRSTUV" if zone % 2 else "FGHJKLMNPQRSTUVABCDE"
    row = row_letters[int(northing // 100000) % 20]
    return f"{zone}{band}{col}{row}"


# --------------------------------------------------------------------------- #
# io
# --------------------------------------------------------------------------- #
def read_pcd_xyz(path: Path, stride: int = 1) -> np.ndarray:
    raw = path.read_bytes()
    marker = b"DATA binary\n"
    idx = raw.find(marker)
    if idx < 0:
        raise RuntimeError("only DATA binary supported")
    header = raw[:idx].decode("ascii", "replace")
    fields = re.search(r"FIELDS (.+)", header).group(1).split()
    sizes = [int(v) for v in re.search(r"SIZE (.+)", header).group(1).split()]
    counts = [int(v) for v in re.search(r"COUNT (.+)", header).group(1).split()]
    n = int(re.search(r"POINTS (\d+)", header).group(1))
    dtype = []
    for f, s, c in zip(fields, sizes, counts):
        if f in ("x", "y", "z"):
            dtype.append((f, f"<f{s}"))
        else:
            dtype.append((f"pad_{len(dtype)}", f"V{s * c}"))
    arr = np.frombuffer(raw, dtype=np.dtype(dtype), count=n, offset=idx + len(marker))
    if stride > 1:
        arr = arr[::stride]
    return np.column_stack([arr["x"], arr["y"], arr["z"]]).astype(np.float64)


def parse_osm(path: Path):
    """Return node id -> (lat, lon), plus way-node id sets by role."""
    node_ids, lats, lons = [], [], []
    way_node_ids: set[int] = set()
    lanelet_way_ids: set[int] = set()
    ways: dict[int, list[int]] = {}
    for _event, elem in ET.iterparse(str(path), events=("end",)):
        if elem.tag == "node":
            node_ids.append(int(elem.get("id")))
            lats.append(float(elem.get("lat")))
            lons.append(float(elem.get("lon")))
            elem.clear()
        elif elem.tag == "way":
            wid = int(elem.get("id"))
            refs = [int(nd.get("ref")) for nd in elem.findall("nd")]
            ways[wid] = refs
            way_node_ids.update(refs)
            elem.clear()
        elif elem.tag == "relation":
            tags = {t.get("k"): t.get("v") for t in elem.findall("tag")}
            if tags.get("type") == "lanelet":
                for m in elem.findall("member"):
                    if m.get("type") == "way" and m.get("role") in ("left", "right"):
                        lanelet_way_ids.add(int(m.get("ref")))
            elem.clear()
    ids = np.asarray(node_ids, dtype=np.int64)
    lat = np.asarray(lats, dtype=np.float64)
    lon = np.asarray(lons, dtype=np.float64)
    lanelet_nodes: set[int] = set()
    for wid in lanelet_way_ids:
        lanelet_nodes.update(ways.get(wid, []))
    return ids, lat, lon, way_node_ids, lanelet_nodes


# --------------------------------------------------------------------------- #
# helpers
# --------------------------------------------------------------------------- #
def path_length(p: np.ndarray) -> float:
    return float(np.linalg.norm(np.diff(p, axis=0), axis=1).sum())


def stats(d: np.ndarray) -> dict:
    return {
        "n": int(d.size),
        "mean_m": float(np.mean(d)),
        "median_m": float(np.median(d)),
        "rmse_m": float(np.sqrt(np.mean(d**2))),
        "p95_m": float(np.percentile(d, 95)),
        "max_m": float(np.max(d)),
    }


def umeyama(src: np.ndarray, dst: np.ndarray, with_scale: bool):
    mu_s, mu_d = src.mean(0), dst.mean(0)
    s0, d0 = src - mu_s, dst - mu_d
    cov = d0.T @ s0 / len(src)
    u, sv, vt = np.linalg.svd(cov)
    d = np.eye(3)
    if np.linalg.det(u) * np.linalg.det(vt) < 0:
        d[2, 2] = -1.0
    r = u @ d @ vt
    if with_scale:
        var_s = float(np.mean(np.sum(s0**2, axis=1)))
        s = float(np.trace(np.diag(sv) @ d) / var_s)
    else:
        s = 1.0
    t = mu_d - s * (r @ mu_s)
    return r, t, s


def apply(p: np.ndarray, r: np.ndarray, t: np.ndarray, s: float = 1.0) -> np.ndarray:
    return s * (p @ r.T) + t


def best_2d_offset(src_xy: np.ndarray, tree: cKDTree, half: float, step: float):
    """Coarse-to-fine search for the 2-D shift minimising median NN distance."""
    cx = cy = 0.0
    for scale in (1.0, 0.2, 0.05):
        rng = half * scale
        st = max(step * scale, 0.02)
        grid = np.arange(-rng, rng + 1e-9, st)
        best = (None, None, np.inf)
        for dx in grid:
            for dy in grid:
                d, _ = tree.query(src_xy + np.array([cx + dx, cy + dy]))
                med = float(np.median(d))
                if med < best[2]:
                    best = (cx + dx, cy + dy, med)
        cx, cy, med = best
    return {"dx_m": float(cx), "dy_m": float(cy), "median_nn_m": float(med)}


# --------------------------------------------------------------------------- #
def main() -> None:
    report = json.loads(GEOREF_REPORT.read_text())
    rebuild = json.loads(REBUILD_VALIDATION.read_text())
    org = report["enu_origin_lla"]
    lat0, lon0, alt0 = org["latitude"], org["longitude"], org["altitude"]

    out: dict = {
        "origin": {"lat": lat0, "lon": lon0, "alt_m": alt0},
        "sources": {
            "georef_report": str(GEOREF_REPORT),
            "rebuild_validation": str(REBUILD_VALIDATION),
            "osm": str(OSM),
            "pcd": str(PCD),
        },
    }

    # ---------------- A: projector frame consistency ----------------------- #
    grid_e, grid_n = np.meshgrid(np.linspace(-120, 140, 27), np.linspace(-120, 130, 26))
    ge, gn = grid_e.ravel(), grid_n.ravel()
    glat, glon = enu_to_lla_spherical(ge, gn, lat0, lon0)
    exact = local_cartesian(glat, glon, np.zeros_like(glat), lat0, lon0, 0.0)
    dxy = np.column_stack([exact[:, 0] - ge, exact[:, 1] - gn])
    dnorm = np.linalg.norm(dxy, axis=1)
    out["A_projector_frame_consistency"] = {
        "description": (
            "Difference between exact LocalCartesian (what Autoware/lanelet2 use) and "
            "the spherical ENU used by georeference_kiss_to_gnss.py, evaluated on a grid "
            "covering the PCD footprint."
        ),
        "grid_extent_m": {"east": [-120, 140], "north": [-120, 130]},
        "dxy_norm_m": stats(dnorm),
        "east_scale_ratio_exact_over_spherical": float(
            np.polyfit(ge[np.abs(ge) > 1], exact[np.abs(ge) > 1, 0], 1)[0]
        ),
        "north_scale_ratio_exact_over_spherical": float(
            np.polyfit(gn[np.abs(gn) > 1], exact[np.abs(gn) > 1, 1], 1)[0]
        ),
        "up_at_footprint_edge_m": float(np.min(exact[:, 2])),
    }

    # ---------------- B: KISS <-> GNSS alignment --------------------------- #
    arr = np.load(GEOREF_ARRAYS)
    kiss = arr["kiss_xyz"]
    gnss = arr["gnss_enu"]
    r_rep = np.asarray(report["R"])
    t_rep = np.asarray(report["t"])

    scale_rebuild = float(rebuild["scale_applied"])
    kiss_scaled = kiss[0] + scale_rebuild * (kiss - kiss[0])

    res_report = np.linalg.norm(apply(kiss, r_rep, t_rep) - gnss, axis=1)
    res_rebuild = np.linalg.norm(apply(kiss_scaled, r_rep, t_rep) - gnss, axis=1)

    r_s, t_s, s_s = umeyama(kiss, gnss, with_scale=True)
    res_sim = np.linalg.norm(apply(kiss, r_s, t_s, s_s) - gnss, axis=1)
    r_r, t_r, _ = umeyama(kiss, gnss, with_scale=False)
    res_rig = np.linalg.norm(apply(kiss, r_r, t_r) - gnss, axis=1)

    # arc length is noise-inflated; decimating suppresses per-step jitter
    decim = {}
    for k in (1, 2, 5, 10, 20, 50):
        decim[str(k)] = {
            "kiss_m": path_length(kiss[::k]),
            "gnss_m": path_length(gnss[::k]),
            "ratio": path_length(kiss[::k]) / max(path_length(gnss[::k]), 1e-9),
        }

    span_kiss = float(np.max(np.linalg.norm(kiss - kiss.mean(0), axis=1)))
    span_gnss = float(np.max(np.linalg.norm(gnss - gnss.mean(0), axis=1)))
    # scale-free comparison: ratio of pairwise-distance RMS
    idx = np.linspace(0, len(kiss) - 1, 400).astype(int)
    dk = np.linalg.norm(kiss[idx][:, None, :] - kiss[idx][None, :, :], axis=-1)
    dg = np.linalg.norm(gnss[idx][:, None, :] - gnss[idx][None, :, :], axis=-1)
    m = dg > 1.0
    pairwise_scale = float(np.sum(dk[m] * dg[m]) / np.sum(dg[m] ** 2))

    out["B_kiss_gnss_alignment"] = {
        "n_matched": int(len(kiss)),
        "residuals_report_transform_unscaled": stats(res_report),
        "residuals_rebuild_transform_scaled": stats(res_rebuild),
        "residuals_optimal_similarity": stats(res_sim),
        "residuals_optimal_rigid": stats(res_rig),
        "optimal_similarity_scale": float(s_s),
        "pairwise_distance_scale_kiss_over_gnss": pairwise_scale,
        "scale_applied_by_rebuild": scale_rebuild,
        "arc_length_by_decimation": decim,
        "radius_of_gyration_max_m": {"kiss": span_kiss, "gnss": span_gnss},
    }

    # ---------------- C: OSM bounds ---------------------------------------- #
    ids, olat, olon, way_nodes, lanelet_nodes = parse_osm(OSM)
    enu_all = local_cartesian(olat, olon, np.zeros_like(olat), lat0, lon0, 0.0)
    enu_sph = spherical_enu(olat, olon, np.zeros_like(olat), lat0, lon0, alt0)

    pcd_min = np.asarray(rebuild["bbox_min_xyz"])
    pcd_max = np.asarray(rebuild["bbox_max_xyz"])
    pad = 20.0
    near = (
        (enu_all[:, 0] > pcd_min[0] - pad)
        & (enu_all[:, 0] < pcd_max[0] + pad)
        & (enu_all[:, 1] > pcd_min[1] - pad)
        & (enu_all[:, 1] < pcd_max[1] + pad)
    )

    corners = [
        (olat.min(), olon.min()),
        (olat.min(), olon.max()),
        (olat.max(), olon.min()),
        (olat.max(), olon.max()),
    ]
    grids = sorted({mgrs_100km_square(a, b) for a, b in corners})

    out["C_osm_bounds"] = {
        "n_nodes": int(len(ids)),
        "n_nodes_referenced_by_ways": int(len(way_nodes)),
        "n_lanelet_boundary_nodes": int(len(lanelet_nodes)),
        "geographic_bounds": {
            "lat_min": float(olat.min()),
            "lat_max": float(olat.max()),
            "lon_min": float(olon.min()),
            "lon_max": float(olon.max()),
        },
        "extent_m": {
            "east": float(enu_all[:, 0].max() - enu_all[:, 0].min()),
            "north": float(enu_all[:, 1].max() - enu_all[:, 1].min()),
        },
        "enu_bounds_localcartesian_full": {
            "min": enu_all.min(0).tolist(),
            "max": enu_all.max(0).tolist(),
        },
        "enu_bounds_localcartesian_near_pcd": {
            "min": enu_all[near].min(0).tolist() if near.any() else None,
            "max": enu_all[near].max(0).tolist() if near.any() else None,
            "n_nodes": int(near.sum()),
        },
        "mgrs_100km_squares_at_corners": grids,
        "spherical_vs_localcartesian_shift_near_pcd_m": (
            stats(np.linalg.norm(enu_all[near, :2] - enu_sph[near, :2], axis=1))
            if near.any()
            else None
        ),
        "pcd_bbox_from_rebuild": {"min": pcd_min.tolist(), "max": pcd_max.tolist()},
        "bbox_overlap_m": {
            "east": float(
                min(pcd_max[0], enu_all[:, 0].max()) - max(pcd_min[0], enu_all[:, 0].min())
            ),
            "north": float(
                min(pcd_max[1], enu_all[:, 1].max()) - max(pcd_min[1], enu_all[:, 1].min())
            ),
        },
    }

    # ---------------- D: overlay quality ----------------------------------- #
    pcd = read_pcd_xyz(PCD, stride=1)
    out["D_overlay"] = {
        "pcd_bbox_measured": {"min": pcd.min(0).tolist(), "max": pcd.max(0).tolist()},
        "pcd_n_points": int(len(pcd)),
    }

    lanelet_mask = np.isin(ids, np.fromiter(lanelet_nodes, dtype=np.int64))
    lane_enu = enu_all[lanelet_mask]
    lane_near = lane_enu[
        (lane_enu[:, 0] > pcd_min[0] - pad)
        & (lane_enu[:, 0] < pcd_max[0] + pad)
        & (lane_enu[:, 1] > pcd_min[1] - pad)
        & (lane_enu[:, 1] < pcd_max[1] + pad)
    ]
    out["D_overlay"]["n_lanelet_nodes_near_pcd"] = int(len(lane_near))

    # densify lanelet boundary polylines is overkill; node spacing is ~1 m here
    if len(lane_near) >= 10:
        tree_lane = cKDTree(lane_near[:, :2])

        gnss_xy = gnss[:, :2]
        d_traj, _ = tree_lane.query(gnss_xy)
        out["D_overlay"]["gnss_track_to_lanelet_nn"] = stats(d_traj)
        out["D_overlay"]["gnss_track_best_offset"] = best_2d_offset(
            gnss_xy, tree_lane, half=40.0, step=2.0
        )

        # ground slab of the PCD ~ drivable surface
        zs = np.percentile(pcd[:, 2], [2, 25])
        ground = pcd[(pcd[:, 2] > zs[0]) & (pcd[:, 2] < zs[1])]
        gsub = ground[:: max(1, len(ground) // 60000)]
        d_g, _ = tree_lane.query(gsub[:, :2])
        out["D_overlay"]["pcd_ground_to_lanelet_nn"] = stats(d_g)
        out["D_overlay"]["pcd_ground_z_window"] = zs.tolist()

        # reverse direction: how much of the lanelet network is covered by PCD
        tree_pcd = cKDTree(pcd[:: max(1, len(pcd) // 400000), :2])
        d_rev, _ = tree_pcd.query(lane_near[:, :2])
        out["D_overlay"]["lanelet_to_pcd_nn"] = stats(d_rev)

    # all way nodes near the dock (drivable-area / building outlines included)
    way_mask = np.isin(ids, np.fromiter(way_nodes, dtype=np.int64))
    wn = enu_all[way_mask]
    wn = wn[
        (wn[:, 0] > pcd_min[0] - pad)
        & (wn[:, 0] < pcd_max[0] + pad)
        & (wn[:, 1] > pcd_min[1] - pad)
        & (wn[:, 1] < pcd_max[1] + pad)
    ]
    out["D_overlay"]["n_way_nodes_near_pcd"] = int(len(wn))
    if len(wn) >= 10:
        tree_w = cKDTree(wn[:, :2])
        d_traj_w, _ = tree_w.query(gnss[:, :2])
        out["D_overlay"]["gnss_track_to_any_way_node_nn"] = stats(d_traj_w)
        out["D_overlay"]["gnss_track_best_offset_any_way"] = best_2d_offset(
            gnss[:, :2], tree_w, half=40.0, step=2.0
        )

    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(json.dumps(out, indent=2))
    print(json.dumps(out, indent=2))


if __name__ == "__main__":
    main()
