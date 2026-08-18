#!/usr/bin/env python3
"""Isolated Engineering Loading Dock map build via KISS-ICP.

Does not touch Autoware / sample localization. Reads a ROS 2 bag, runs
KISS-ICP lidar odometry, accumulates a voxelized global map, writes
Autoware-style binary PCD + intermediate poses/metrics.
"""

from __future__ import annotations

import argparse
import json
import os
import time
from pathlib import Path

import numpy as np
import open3d as o3d
from kiss_icp.config import load_config
from kiss_icp.datasets import dataset_factory
from kiss_icp.kiss_icp import KissICP
from kiss_icp.pipeline import OdometryPipeline
from kiss_icp.voxelization import voxel_down_sample
from tqdm import tqdm


def write_autoware_pcd(path: Path, points_xyz: np.ndarray) -> None:
    """Write binary PCD matching sample-map FIELD layout (x y z _)."""
    pts = np.asarray(points_xyz, dtype=np.float32).reshape(-1, 3)
    n = pts.shape[0]
    # 16-byte point: 3x float32 + 4-byte padding (matches sample-map-rosbag)
    cloud = np.zeros((n, 4), dtype=np.float32)
    cloud[:, :3] = pts
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
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "wb") as f:
        f.write(header)
        f.write(cloud.tobytes())


def transform_points(points: np.ndarray, pose: np.ndarray) -> np.ndarray:
    ones = np.ones((points.shape[0], 1), dtype=np.float64)
    homo = np.hstack([points.astype(np.float64), ones])
    return (pose @ homo.T).T[:, :3]


def accumulate_map(
    bag_path: Path,
    topic: str,
    config_path: Path | None,
    out_dir: Path,
    map_voxel: float,
    scan_stride: int,
    max_range: float | None,
    n_scans: int,
    jump: int,
) -> dict:
    out_dir.mkdir(parents=True, exist_ok=True)
    config = load_config(str(config_path) if config_path else None)
    config.out_dir = str(out_dir)

    dataset = dataset_factory(
        dataloader="rosbag",
        data_dir=bag_path,
        topic=topic,
    )

    pipe = OdometryPipeline(
        dataset=dataset,
        config=config_path,
        visualize=False,
        n_scans=n_scans,
        jump=jump,
    )
    # Force out_dir after pipeline constructed (load_config inside may ignore file out_dir)
    pipe.config.out_dir = str(out_dir)
    if max_range is not None:
        pipe.config.data.max_range = float(max_range)

    # Rebuild odometry with possibly updated config
    pipe.odometry = KissICP(config=pipe.config)

    global_cloud = o3d.geometry.PointCloud()
    n_used = 0
    n_raw_points = 0

    first = pipe._first
    last = pipe._last
    t0 = time.perf_counter()

    for idx in tqdm(range(first, last), desc="KISS-ICP+map", unit="scan"):
        raw_frame, timestamps = dataset[idx]
        source, _keypoints = pipe.odometry.register_frame(raw_frame, timestamps)
        pose = pipe.odometry.last_pose
        pipe.poses[idx - first] = pose
        pipe.times[idx - first] = 0.0

        if ((idx - first) % scan_stride) != 0:
            continue

        # Use motion-compensated / range-filtered source in sensor frame
        pts = np.asarray(source, dtype=np.float64)
        if pts.size == 0:
            continue
        if max_range is not None:
            norms = np.linalg.norm(pts, axis=1)
            pts = pts[(norms >= pipe.config.data.min_range) & (norms <= max_range)]
        if pts.size == 0:
            continue

        # Pre-downsample each scan before world transform
        pts = voxel_down_sample(pts, map_voxel)
        world = transform_points(pts, pose)
        n_raw_points += world.shape[0]

        pc = o3d.geometry.PointCloud()
        pc.points = o3d.utility.Vector3dVector(world)
        global_cloud += pc
        n_used += 1

        # Periodic voxelization to bound memory
        if n_used % 50 == 0:
            global_cloud = global_cloud.voxel_down_sample(map_voxel)

    # Final downsample
    global_cloud = global_cloud.voxel_down_sample(map_voxel)
    points = np.asarray(global_cloud.points)

    # Write KISS-ICP pose artifacts
    pipe._create_output_dir()
    pipe._write_result_poses()
    pipe._write_cfg()
    pipe._run_evaluation()
    pipe._write_log()

    pcd_path = out_dir / "pointcloud_map_kiss_frame.pcd"
    write_autoware_pcd(pcd_path, points)

    # Trajectory length for quality report
    translations = pipe.poses[:, :3, 3]
    diffs = np.linalg.norm(np.diff(translations, axis=0), axis=1)
    traj_len = float(diffs.sum()) if len(diffs) else 0.0
    bbox_min = points.min(axis=0).tolist() if len(points) else [0, 0, 0]
    bbox_max = points.max(axis=0).tolist() if len(points) else [0, 0, 0]
    bbox_diag = float(np.linalg.norm(np.array(bbox_max) - np.array(bbox_min))) if len(points) else 0.0

    summary = {
        "bag": str(bag_path),
        "topic": topic,
        "n_scans_odometry": int(last - first),
        "n_scans_in_map": int(n_used),
        "scan_stride": scan_stride,
        "map_voxel_m": map_voxel,
        "max_range_m": pipe.config.data.max_range,
        "n_map_points": int(points.shape[0]),
        "points_before_final_voxel_estimate": int(n_raw_points),
        "trajectory_length_m": traj_len,
        "bbox_min_xyz": bbox_min,
        "bbox_max_xyz": bbox_max,
        "bbox_diagonal_m": bbox_diag,
        "runtime_s": time.perf_counter() - t0,
        "poses_dir": str(pipe.results_dir),
        "pcd_kiss_frame": str(pcd_path),
        "frame_note": (
            "Map is in KISS-ICP odometry frame (first pose ~ identity). "
            "NOT yet georeferenced to GNSS / Lanelet2 / Autoware map frame."
        ),
    }
    with open(out_dir / "build_summary.json", "w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2)
    return summary


def validate_pcd(pcd_path: Path, report_path: Path) -> dict:
    pcd = o3d.io.read_point_cloud(str(pcd_path))
    pts = np.asarray(pcd.points)
    report = {
        "path": str(pcd_path),
        "exists": pcd_path.exists(),
        "size_bytes": pcd_path.stat().st_size if pcd_path.exists() else 0,
        "n_points_open3d": int(pts.shape[0]),
        "empty": bool(pts.size == 0),
    }
    if pts.size:
        report["min_xyz"] = pts.min(axis=0).tolist()
        report["max_xyz"] = pts.max(axis=0).tolist()
        report["mean_xyz"] = pts.mean(axis=0).tolist()
        report["bbox_diagonal_m"] = float(np.linalg.norm(pts.max(0) - pts.min(0)))
        # Rough density: points / bbox volume (avoid div0)
        extents = np.maximum(pts.max(0) - pts.min(0), 1e-3)
        report["points_per_m3"] = float(pts.shape[0] / float(np.prod(extents)))
        # Nearest-neighbor spacing sample
        if pts.shape[0] > 2000:
            sample = pts[np.random.default_rng(0).choice(pts.shape[0], 2000, replace=False)]
        else:
            sample = pts
        pcd_s = o3d.geometry.PointCloud()
        pcd_s.points = o3d.utility.Vector3dVector(sample)
        dists = np.asarray(pcd_s.compute_nearest_neighbor_distance())
        report["nn_distance_mean_m"] = float(dists.mean()) if len(dists) else None
        report["nn_distance_median_m"] = float(np.median(dists)) if len(dists) else None

        # Heuristic quality flags (honest, not polished)
        flags = []
        if report["bbox_diagonal_m"] < 20:
            flags.append("bbox_diagonal_small_for_200m_drive")
        if report["n_points_open3d"] < 50_000:
            flags.append("low_point_count")
        if report.get("nn_distance_median_m", 0) and report["nn_distance_median_m"] > 0.5:
            flags.append("sparse_nn_spacing")
        # Vertical thickness: outdoor maps often tens of meters of Z range
        z_span = float(pts[:, 2].max() - pts[:, 2].min())
        report["z_span_m"] = z_span
        if z_span > 40:
            flags.append("large_z_span_possible_drift_or_tilt")
        report["quality_flags"] = flags
        report["quality_verdict"] = (
            "suspect_poor" if flags else "plausible_first_pass_local_frame"
        )
    else:
        report["quality_flags"] = ["empty_cloud"]
        report["quality_verdict"] = "failed"

    with open(report_path, "w", encoding="utf-8") as f:
        json.dump(report, f, indent=2)
    return report


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", type=Path, required=True)
    ap.add_argument("--topic", default="/velodyne_points")
    ap.add_argument("--config", type=Path, default=None)
    ap.add_argument("--out-dir", type=Path, required=True)
    ap.add_argument("--map-voxel", type=float, default=0.2)
    ap.add_argument("--scan-stride", type=int, default=2)
    ap.add_argument("--max-range", type=float, default=80.0)
    ap.add_argument("--n-scans", type=int, default=-1)
    ap.add_argument("--jump", type=int, default=0)
    ap.add_argument("--final-pcd", type=Path, required=True)
    args = ap.parse_args()

    os.environ.setdefault("kiss_icp_out_dir", str(args.out_dir))

    summary = accumulate_map(
        bag_path=args.bag,
        topic=args.topic,
        config_path=args.config,
        out_dir=args.out_dir,
        map_voxel=args.map_voxel,
        scan_stride=args.scan_stride,
        max_range=args.max_range,
        n_scans=args.n_scans,
        jump=args.jump,
    )

    kiss_pcd = Path(summary["pcd_kiss_frame"])
    # Copy/promote to requested Autoware map path (still kiss frame; documented)
    args.final_pcd.parent.mkdir(parents=True, exist_ok=True)
    args.final_pcd.write_bytes(kiss_pcd.read_bytes())

    report = validate_pcd(args.final_pcd, args.out_dir / "validation_open3d.json")
    print(json.dumps({"summary": summary, "validation": report}, indent=2))


if __name__ == "__main__":
    main()
