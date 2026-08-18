#!/usr/bin/env python3
"""Georeference existing KISS-ICP PCD via rigid align of LO trajectory to NovAtel GNSS.

Does not overwrite pointcloud_map.pcd. Does not write map_projector_info.yaml
unless frames are proven identical (this script never writes it).
"""

from __future__ import annotations

import json
import math
import re
import struct
from pathlib import Path

import numpy as np

MAP_ROOT = Path("/home/agxorin/autoware_data/maps/eng_loading_dock_map")
BAG = Path("/home/agxorin/Desktop/engg_loading_dock_01_gnss")
POSES_TUM = MAP_ROOT / "intermediate/full_run/latest/engg_loading_dock_01_gnss_poses_tum.txt"
PCD_IN = MAP_ROOT / "pointcloud_map.pcd"
PCD_OUT = MAP_ROOT / "pointcloud_map_georeferenced.pcd"
OSM = MAP_ROOT / "finalMap.osm"
OUT_DIR = MAP_ROOT / "intermediate" / "georef"
RESULTS = MAP_ROOT / "results"


def load_tum(path: Path) -> tuple[np.ndarray, np.ndarray]:
    """Return times (N,), xyz (N,3) from TUM trajectory (ignore quaternion for Kabsch on positions)."""
    rows = []
    for line in path.read_text().splitlines():
        if not line.strip() or line.startswith("#"):
            continue
        p = line.split()
        rows.append([float(x) for x in p[:4]])  # t x y z
    arr = np.asarray(rows, dtype=np.float64)
    return arr[:, 0], arr[:, 1:4]


def load_gnss_navsat(bag: Path) -> tuple[np.ndarray, np.ndarray]:
    from rosbags.highlevel import AnyReader
    from rosbags.typesys import Stores, get_typestore

    typestore = get_typestore(Stores.LATEST)
    ts, lla = [], []
    with AnyReader([bag], default_typestore=typestore) as reader:
        conns = [c for c in reader.connections if c.topic == "/novatel/oem7/fix"]
        for conn, _t, raw in reader.messages(connections=conns):
            msg = reader.deserialize(raw, conn.msgtype)
            if msg.status.status < 0:
                continue
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            ts.append(t)
            lla.append([msg.latitude, msg.longitude, msg.altitude])
    return np.asarray(ts, dtype=np.float64), np.asarray(lla, dtype=np.float64)


def lla_to_enu(lat, lon, alt, lat0, lon0, alt0) -> np.ndarray:
    """WGS84 approximate local ENU (meters)."""
    lat = np.asarray(lat, dtype=np.float64)
    lon = np.asarray(lon, dtype=np.float64)
    alt = np.asarray(alt, dtype=np.float64)
    R = 6378137.0
    lat0r = math.radians(lat0)
    lon0r = math.radians(lon0)
    east = (np.radians(lon) - lon0r) * math.cos(lat0r) * R
    north = (np.radians(lat) - lat0r) * R
    up = alt - alt0
    return np.column_stack([east, north, up])


def interp_traj(t_src: np.ndarray, xyz_src: np.ndarray, t_query: np.ndarray) -> np.ndarray:
    """Linear interpolate xyz at query times; NaN outside range."""
    out = np.full((len(t_query), 3), np.nan, dtype=np.float64)
    for d in range(3):
        out[:, d] = np.interp(t_query, t_src, xyz_src[:, d], left=np.nan, right=np.nan)
    # np.interp doesn't support nan left/right with float nan easily — mask manually
    mask = (t_query < t_src[0]) | (t_query > t_src[-1])
    out[mask] = np.nan
    return out


def kabsch_rigid(A: np.ndarray, B: np.ndarray, with_scale: bool = False):
    """Find R,t (and optional s) minimizing ||s*R*A + t - B||. A,B are Nx3."""
    assert A.shape == B.shape and A.shape[1] == 3
    centroid_A = A.mean(axis=0)
    centroid_B = B.mean(axis=0)
    AA = A - centroid_A
    BB = B - centroid_B
    H = AA.T @ BB
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T
    if with_scale:
        var_A = (AA**2).sum() / A.shape[0]
        s = float(S.sum() / var_A) if var_A > 0 else 1.0
    else:
        s = 1.0
    t = centroid_B - s * (R @ centroid_A)
    return R, t, s


def apply_rigid(X: np.ndarray, R: np.ndarray, t: np.ndarray, s: float = 1.0) -> np.ndarray:
    return (s * (X @ R.T)) + t


def residuals(A: np.ndarray, B: np.ndarray, R, t, s=1.0) -> np.ndarray:
    pred = apply_rigid(A, R, t, s)
    return np.linalg.norm(pred - B, axis=1)


def read_pcd_xyz(path: Path) -> tuple[bytes, np.ndarray]:
    raw = path.read_bytes()
    marker = b"DATA binary\n"
    idx = raw.find(marker)
    if idx < 0:
        raise RuntimeError(f"Unsupported PCD (need DATA binary): {path}")
    header = raw[: idx + len(marker)]
    data = raw[idx + len(marker) :]
    # sample map format: x y z _ as 4 float32 (16 bytes) — matches our writer
    arr = np.frombuffer(data, dtype=np.float32).reshape(-1, 4).copy()
    return header, arr


def write_pcd_xyz(path: Path, xyz: np.ndarray) -> None:
    n = xyz.shape[0]
    cloud = np.zeros((n, 4), dtype=np.float32)
    cloud[:, :3] = xyz.astype(np.float32)
    cloud[:, 3] = 1.0
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z _\n"
        "SIZE 4 4 4 1\n"
        "TYPE F F F U\n"
        "COUNT 1 1 1 4\n"
        f"WIDTH {n}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {n}\n"
        "DATA binary\n"
    ).encode("ascii")
    path.write_bytes(header + cloud.tobytes())


def load_osm_lla_sample(path: Path, max_nodes: int = 200000) -> np.ndarray:
    lats, lons = [], []
    with path.open() as f:
        for line in f:
            m = re.search(r'lat="([^"]+)"\s+lon="([^"]+)"', line)
            if m:
                lats.append(float(m.group(1)))
                lons.append(float(m.group(2)))
                if len(lats) >= max_nodes:
                    break
    return np.column_stack([lats, lons])


def main() -> None:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    RESULTS.mkdir(parents=True, exist_ok=True)

    t_kiss, xyz_kiss = load_tum(POSES_TUM)
    t_gnss, lla = load_gnss_navsat(BAG)

    # Local ENU origin: first GNSS fix (documented, not MGRS)
    lat0, lon0, alt0 = map(float, lla[0])
    xyz_gnss = lla_to_enu(lla[:, 0], lla[:, 1], lla[:, 2], lat0, lon0, alt0)

    # Interpolate GNSS to KISS timestamps
    xyz_gnss_at_kiss = interp_traj(t_gnss, xyz_gnss, t_kiss)
    valid = ~np.isnan(xyz_gnss_at_kiss[:, 0])
    A = xyz_kiss[valid]  # KISS local
    B = xyz_gnss_at_kiss[valid]  # ENU
    t_aligned = t_kiss[valid]

    # Also SE(2)-ish: use XY only for planar rigid (R about Z) — report both
    R3, t3, s1 = kabsch_rigid(A, B, with_scale=False)
    R3s, t3s, s_opt = kabsch_rigid(A, B, with_scale=True)

    # Planar: Kabsch on XY, keep Z via mean offset + optional R from 3D
    A_xy = np.column_stack([A[:, 0], A[:, 1], np.zeros(len(A))])
    B_xy = np.column_stack([B[:, 0], B[:, 1], np.zeros(len(B))])
    R2, t2, _ = kabsch_rigid(A_xy, B_xy, with_scale=False)
    # Apply R2 to full 3D with z translation = mean(B_z - (R2 A)_z)
    A_r2 = apply_rigid(A, R2, np.zeros(3), 1.0)
    t2_full = np.array([t2[0], t2[1], B[:, 2].mean() - A_r2[:, 2].mean()])

    res3 = residuals(A, B, R3, t3, 1.0)
    res3s = residuals(A, B, R3s, t3s, s_opt)
    res2 = residuals(A, B, R2, t2_full, 1.0)

    # Time-resolved error (drift signature)
    order = np.argsort(t_aligned)
    t_rel = t_aligned[order] - t_aligned[order][0]
    res3_sorted = res3[order]

    def chunk_stats(res, t_rel, edges):
        out = []
        for i in range(len(edges) - 1):
            m = (t_rel >= edges[i]) & (t_rel < edges[i + 1])
            if m.sum() == 0:
                continue
            out.append(
                {
                    "t0_s": float(edges[i]),
                    "t1_s": float(edges[i + 1]),
                    "n": int(m.sum()),
                    "rmse_m": float(np.sqrt(np.mean(res[m] ** 2))),
                    "median_m": float(np.median(res[m])),
                    "p95_m": float(np.percentile(res[m], 95)),
                }
            )
        return out

    edges = np.linspace(0, float(t_rel[-1]) + 1e-6, 6)
    chunks = chunk_stats(res3_sorted, t_rel, edges)

    # Path lengths
    def path_len(x):
        return float(np.linalg.norm(np.diff(x, axis=0), axis=1).sum())

    report = {
        "enu_origin_lla": {"latitude": lat0, "longitude": lon0, "altitude": alt0},
        "enu_frame_note": (
            "Local ENU meters east-north-up from first NovAtel fix. "
            "NOT Autoware MGRS (OSM spans 11UPS/11UQS). NOT proven Autoware map frame."
        ),
        "n_kiss_poses": int(len(t_kiss)),
        "n_gnss": int(len(t_gnss)),
        "n_matched": int(valid.sum()),
        "time_span_matched_s": float(t_aligned[-1] - t_aligned[0]) if valid.any() else 0.0,
        "kiss_path_length_m": path_len(A),
        "gnss_path_length_m": path_len(B),
        "path_length_ratio_kiss_over_gnss": path_len(A) / path_len(B) if path_len(B) else None,
        "rigid_se3_no_scale": {
            "R": R3.tolist(),
            "t": t3.tolist(),
            "s": 1.0,
            "rmse_m": float(np.sqrt(np.mean(res3**2))),
            "median_m": float(np.median(res3)),
            "mean_m": float(res3.mean()),
            "p95_m": float(np.percentile(res3, 95)),
            "max_m": float(res3.max()),
        },
        "rigid_se3_with_scale": {
            "R": R3s.tolist(),
            "t": t3s.tolist(),
            "s": float(s_opt),
            "rmse_m": float(np.sqrt(np.mean(res3s**2))),
            "median_m": float(np.median(res3s)),
            "p95_m": float(np.percentile(res3s, 95)),
            "scale_improvement_rmse_m": float(
                np.sqrt(np.mean(res3**2)) - np.sqrt(np.mean(res3s**2))
            ),
        },
        "planar_yaw_xy_no_scale": {
            "R": R2.tolist(),
            "t": t2_full.tolist(),
            "rmse_m": float(np.sqrt(np.mean(res2**2))),
            "median_m": float(np.median(res2)),
            "p95_m": float(np.percentile(res2, 95)),
        },
        "residuals_vs_time_se3": chunks,
        "interpretation": {},
    }

    # Decide if scale required: only if improvement is large AND s far from 1
    scale_needed = abs(s_opt - 1.0) > 0.02 and report["rigid_se3_with_scale"][
        "scale_improvement_rmse_m"
    ] > 0.5
    report["scale_required"] = bool(scale_needed)
    report["chosen_transform"] = "rigid_se3_no_scale"
    report["R"] = R3.tolist()
    report["t"] = t3.tolist()
    report["s"] = 1.0

    rmse = report["rigid_se3_no_scale"]["rmse_m"]
    p95 = report["rigid_se3_no_scale"]["p95_m"]
    # Reasonable global alignment: RMSE of a few meters on ~200m RTK path with known LO drift
    apply_pcd = rmse < 15.0  # allow apply for inspection; flag quality separately
    report["apply_pcd"] = bool(apply_pcd)
    report["global_alignment_quality"] = (
        "good" if rmse < 2.0 and p95 < 5.0 else "moderate" if rmse < 8.0 else "poor"
    )

    # Drift vs global: compare early vs late chunk RMSE
    if len(chunks) >= 2:
        early, late = chunks[0]["rmse_m"], chunks[-1]["rmse_m"]
        report["interpretation"]["early_window_rmse_m"] = early
        report["interpretation"]["late_window_rmse_m"] = late
        report["interpretation"]["rmse_growth_late_minus_early_m"] = late - early
        report["interpretation"]["note"] = (
            "Growth of residual over time indicates internal KISS drift (or lever-arm/"
            "extrinsic effects), not just a bad global SE(3). A low global RMSE can still "
            "leave doubled walls in the PCD."
        )

    # Save residual report BEFORE touching PCD
    report_path = OUT_DIR / "georef_alignment_report.json"
    report_path.write_text(json.dumps(report, indent=2))
    np.savez(
        OUT_DIR / "georef_arrays.npz",
        t_aligned=t_aligned,
        kiss_xyz=A,
        gnss_enu=B,
        kiss_aligned_se3=apply_rigid(A, R3, t3, 1.0),
        residuals_se3=res3,
        R=R3,
        t=t3,
    )
    print("=== ALIGNMENT REPORT (before PCD modify) ===")
    print(json.dumps(report, indent=2))

    if not apply_pcd:
        print("Residuals too large; NOT writing georeferenced PCD.")
        return

    # Transform PCD copy
    _header, cloud = read_pcd_xyz(PCD_IN)
    xyz = cloud[:, :3].astype(np.float64)
    xyz_g = apply_rigid(xyz, R3, t3, 1.0)
    write_pcd_xyz(PCD_OUT, xyz_g)
    print(f"Wrote {PCD_OUT} ({len(xyz_g)} points)")

    # Compare to OSM in same ENU
    osm_lla = load_osm_lla_sample(OSM, max_nodes=250000)
    osm_enu = lla_to_enu(
        osm_lla[:, 0], osm_lla[:, 1], np.zeros(len(osm_lla)), lat0, lon0, alt0
    )
    # Focus OSM near GNSS bbox (+margin)
    margin = 80.0
    bmin = B.min(axis=0) - margin
    bmax = B.max(axis=0) + margin
    osm_near = osm_enu[
        (osm_enu[:, 0] >= bmin[0])
        & (osm_enu[:, 0] <= bmax[0])
        & (osm_enu[:, 1] >= bmin[1])
        & (osm_enu[:, 1] <= bmax[1])
    ]

    pcd_min, pcd_max = xyz_g.min(0), xyz_g.max(0)
    osm_min, osm_max = (
        (osm_near.min(0), osm_near.max(0)) if len(osm_near) else (None, None)
    )

    # Overlap of XY bboxes
    def overlap_1d(a0, a1, b0, b1):
        return max(0.0, min(a1, b1) - max(a0, b0))

    ox = overlap_1d(pcd_min[0], pcd_max[0], osm_min[0], osm_max[0]) if osm_min is not None else 0
    oy = overlap_1d(pcd_min[1], pcd_max[1], osm_min[1], osm_max[1]) if osm_min is not None else 0

    # Distance from transformed traj to nearest OSM node (proxy)
    from scipy.spatial import cKDTree

    tree = cKDTree(osm_near[:, :2]) if len(osm_near) else None
    traj_g = apply_rigid(A, R3, t3, 1.0)
    if tree is not None and len(osm_near):
        d_traj, _ = tree.query(traj_g[:, :2], k=1)
        d_pcd_sample = None
        rng = np.random.default_rng(0)
        sample_idx = rng.choice(len(xyz_g), size=min(5000, len(xyz_g)), replace=False)
        d_pcd, _ = tree.query(xyz_g[sample_idx, :2], k=1)
        d_pcd_sample = {
            "median_m": float(np.median(d_pcd)),
            "p95_m": float(np.percentile(d_pcd, 95)),
        }
        traj_osm = {
            "median_m": float(np.median(d_traj)),
            "p95_m": float(np.percentile(d_traj, 95)),
            "mean_m": float(d_traj.mean()),
        }
    else:
        traj_osm = None
        d_pcd_sample = None

    # Top-down preview ENU
    try:
        res = 0.5
        pts = xyz_g[:, :2]
        mins = pts.min(0) - 5
        maxs = pts.max(0) + 5
        w = int(np.ceil((maxs[0] - mins[0]) / res))
        h = int(np.ceil((maxs[1] - mins[1]) / res))
        img = np.zeros((h, w, 3), dtype=np.uint8)

        def paint(xy, color, rad=0):
            ix = ((xy[:, 0] - mins[0]) / res).astype(int)
            iy = ((xy[:, 1] - mins[1]) / res).astype(int)
            for dx in range(-rad, rad + 1):
                for dy in range(-rad, rad + 1):
                    x = ix + dx
                    y = iy + dy
                    m = (x >= 0) & (x < w) & (y >= 0) & (y < h)
                    img[y[m], x[m]] = color

        # downsample pcd for image
        step = max(1, len(pts) // 400000)
        paint(pts[::step], (220, 220, 220))
        if len(osm_near):
            paint(osm_near[:: max(1, len(osm_near) // 100000), :2], (80, 160, 255))
        paint(traj_g[:, :2], (255, 80, 80), rad=1)
        paint(B[:, :2], (80, 255, 80), rad=1)
        try:
            from PIL import Image

            Image.fromarray(img[::-1]).save(RESULTS / "georef_topdown_enu.png")
        except Exception:
            pass
    except Exception as e:
        print("preview failed", e)

    osm_cmp = {
        "same_frame_as_autoware_map": False,
        "reason": (
            "PCD/traj transformed into ad-hoc ENU from first GNSS fix. "
            "Carter OSM is WGS84 geographic; Autoware would project it via "
            "map_projector_info (MGRS invalid for full OSM; origin-based type unknown). "
            "Spatial overlap in this ENU is a consistency check only — not proof of "
            "shared Autoware map frame. map_projector_info.yaml NOT written."
        ),
        "pcd_bbox_enu": {"min": pcd_min.tolist(), "max": pcd_max.tolist()},
        "osm_near_bbox_enu": {
            "min": osm_min.tolist() if osm_min is not None else None,
            "max": osm_max.tolist() if osm_max is not None else None,
            "n_nodes": int(len(osm_near)),
        },
        "xy_bbox_overlap_m": {"x": float(ox), "y": float(oy)},
        "aligned_traj_to_osm_node_dist": traj_osm,
        "pcd_sample_to_osm_node_dist": d_pcd_sample,
        "original_pcd_preserved": str(PCD_IN),
        "georeferenced_pcd": str(PCD_OUT),
        "ndt_ready": False,
        "ndt_ready_reason": (
            "Global GNSS alignment does not remove internal KISS drift / ghost walls. "
            "Prior visual review showed doubled structures; georef only places the "
            "distorted cloud roughly on Earth."
        ),
    }
    (OUT_DIR / "georef_osm_comparison.json").write_text(json.dumps(osm_cmp, indent=2))
    (RESULTS / "georef_summary.json").write_text(
        json.dumps({"alignment": report, "osm_comparison": osm_cmp}, indent=2)
    )
    print("=== OSM COMPARISON ===")
    print(json.dumps(osm_cmp, indent=2))


if __name__ == "__main__":
    main()
