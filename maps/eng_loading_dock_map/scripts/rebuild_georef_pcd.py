#!/usr/bin/env python3
"""Rebuild eng loading dock PCD: scale-correct KISS trajectory + GNSS SE(3) georef.

Fixes ~1.45× KISS path-length drift vs GNSS by scaling poses about the first
pose before map accumulation, then applies rigid GNSS alignment from
intermediate/georef/georef_alignment_report.json.

Uses saved KISS poses (no ICP re-run); deskews each scan with stored deltas.
"""

from __future__ import annotations

import json
import shutil
import time
from pathlib import Path

import numpy as np
import open3d as o3d
from kiss_icp.config import load_config
from kiss_icp.datasets import dataset_factory
from kiss_icp.preprocess import get_preprocessor
from kiss_icp.voxelization import voxel_down_sample
from tqdm import tqdm

MAP_ROOT = Path("/home/agxorin/autoware_data/maps/eng_loading_dock_map")
BAG = Path("/home/agxorin/Desktop/engg_loading_dock_01_gnss")
GEOREF_REPORT = MAP_ROOT / "intermediate/georef/georef_alignment_report.json"
KISS_POSES_KITTI = (
    MAP_ROOT / "intermediate/full_run/latest/engg_loading_dock_01_gnss_poses_kitti.txt"
)
CONFIG = MAP_ROOT / "docs/kiss_icp_eng_dock_config.yaml"
PCD_OUT = MAP_ROOT / "pointcloud_map.pcd"
PCD_BACKUP = MAP_ROOT / "pointcloud_map.pcd.pre_rebuild_backup"
VALIDATION = MAP_ROOT / "intermediate/georef/rebuild_validation.json"


def write_autoware_pcd(path: Path, points_xyz: np.ndarray) -> None:
    pts = np.asarray(points_xyz, dtype=np.float32).reshape(-1, 3)
    n = pts.shape[0]
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
    path.write_bytes(header + cloud.tobytes())


def transform_points(points: np.ndarray, pose: np.ndarray) -> np.ndarray:
    ones = np.ones((points.shape[0], 1), dtype=np.float64)
    homo = np.hstack([points.astype(np.float64), ones])
    return (pose @ homo.T).T[:, :3]


def load_poses_kitti(path: Path) -> np.ndarray:
    data = np.loadtxt(path)
    n = data.shape[0]
    poses = np.tile(np.eye(4, dtype=np.float64), (n, 1, 1))
    poses[:, :3, :] = data.reshape(n, 3, 4)
    return poses


def load_georef(report_path: Path) -> tuple[np.ndarray, np.ndarray, float, dict]:
    report = json.loads(report_path.read_text())
    R = np.asarray(report["R"], dtype=np.float64)
    t = np.asarray(report["t"], dtype=np.float64)
    ratio = float(report["path_length_ratio_kiss_over_gnss"])
    scale = 1.0 / ratio if ratio > 0 else 1.0
    return R, t, scale, report


def scale_poses_about_anchor(poses: np.ndarray, scale: float, anchor_idx: int = 0) -> np.ndarray:
    out = poses.copy()
    anchor = poses[anchor_idx, :3, 3].copy()
    for i in range(len(poses)):
        t = poses[i, :3, 3]
        out[i, :3, 3] = anchor + scale * (t - anchor)
    return out


def apply_se3_to_poses(poses: np.ndarray, R: np.ndarray, t: np.ndarray) -> np.ndarray:
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = t
    return np.einsum("ij,njk->nik", T, poses)


def path_length(translations: np.ndarray) -> float:
    if len(translations) < 2:
        return 0.0
    return float(np.linalg.norm(np.diff(translations, axis=0), axis=1).sum())


def rebuild_map(
    *,
    map_voxel: float = 0.2,
    scan_stride: int = 2,
    max_range: float = 80.0,
) -> dict:
    R_geo, t_geo, scale, report = load_georef(GEOREF_REPORT)
    kiss_poses = load_poses_kitti(KISS_POSES_KITTI)
    scaled_poses = scale_poses_about_anchor(kiss_poses, scale, anchor_idx=0)
    georef_poses = apply_se3_to_poses(scaled_poses, R_geo, t_geo)

    config = load_config(str(CONFIG))
    config.data.max_range = float(max_range)
    preprocessor = get_preprocessor(config)
    dataset = dataset_factory(dataloader="rosbag", data_dir=BAG, topic="/velodyne_points")
    n_scans = len(kiss_poses)
    assert len(dataset) >= n_scans, f"bag scans {len(dataset)} < poses {n_scans}"

    global_cloud = o3d.geometry.PointCloud()
    n_used = 0
    t0 = time.perf_counter()

    for pose_idx in tqdm(range(n_scans), desc="rebuild-map", unit="scan"):
        if (pose_idx % scan_stride) != 0:
            continue

        raw_frame, timestamps = dataset[pose_idx]
        if pose_idx == 0:
            delta = np.eye(4, dtype=np.float64)
        else:
            delta = np.linalg.inv(kiss_poses[pose_idx - 1]) @ kiss_poses[pose_idx]

        frame = preprocessor.preprocess(raw_frame, timestamps, delta)
        pts = np.asarray(frame, dtype=np.float64)
        if pts.size == 0:
            continue
        norms = np.linalg.norm(pts, axis=1)
        pts = pts[(norms >= config.data.min_range) & (norms <= max_range)]
        if pts.size == 0:
            continue

        pts = voxel_down_sample(pts, map_voxel)
        world = transform_points(pts, georef_poses[pose_idx])
        pc = o3d.geometry.PointCloud()
        pc.points = o3d.utility.Vector3dVector(world)
        global_cloud += pc
        n_used += 1
        if n_used % 50 == 0:
            global_cloud = global_cloud.voxel_down_sample(map_voxel)

    global_cloud = global_cloud.voxel_down_sample(map_voxel)
    points = np.asarray(global_cloud.points)

    if PCD_OUT.exists():
        shutil.copy2(PCD_OUT, PCD_BACKUP)
    write_autoware_pcd(PCD_OUT, points)

    gnss_len = float(report["gnss_path_length_m"])
    kiss_tr = kiss_poses[:, :3, 3]
    scaled_tr = scaled_poses[:, :3, 3]
    geo_tr = georef_poses[:, :3, 3]

    validation = {
        "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "bag": str(BAG),
        "kiss_poses_kitti": str(KISS_POSES_KITTI),
        "georef_report": str(GEOREF_REPORT),
        "scale_applied": scale,
        "path_length_ratio_kiss_over_gnss_report": float(report["path_length_ratio_kiss_over_gnss"]),
        "kiss_path_length_m": path_length(kiss_tr),
        "scaled_kiss_path_length_m": path_length(scaled_tr),
        "georef_path_length_m": path_length(geo_tr),
        "gnss_path_length_m_report": gnss_len,
        "scaled_to_gnss_ratio": path_length(scaled_tr) / gnss_len if gnss_len else None,
        "n_map_points": int(points.shape[0]),
        "n_scans_in_map": n_used,
        "map_voxel_m": map_voxel,
        "scan_stride": scan_stride,
        "bbox_min_xyz": points.min(axis=0).tolist() if len(points) else None,
        "bbox_max_xyz": points.max(axis=0).tolist() if len(points) else None,
        "runtime_s": time.perf_counter() - t0,
        "backup_pcd": str(PCD_BACKUP) if PCD_BACKUP.exists() else None,
        "output_pcd": str(PCD_OUT),
        "se3_rmse_m_report": report["rigid_se3_no_scale"]["rmse_m"],
        "enu_origin_lla": report["enu_origin_lla"],
    }
    VALIDATION.parent.mkdir(parents=True, exist_ok=True)
    VALIDATION.write_text(json.dumps(validation, indent=2))
    print(json.dumps(validation, indent=2))
    return validation


def main() -> None:
    rebuild_map()


if __name__ == "__main__":
    main()
