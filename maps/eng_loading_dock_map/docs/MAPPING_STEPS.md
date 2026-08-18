# Engineering Loading Dock — KISS-ICP map generation (first pass)

**Date:** 2026-08-12  
**Machine:** Jetson AGX Orin, Ubuntu 22.04, aarch64  
**Constraint:** Isolated under `~/autoware_data/maps/eng_loading_dock_map`.  
No Autoware source, sensor-kit, launch, or sample-map files were modified for this workflow.

## Input

| Item | Path |
|------|------|
| ROS 2 bag | `/home/agxorin/Desktop/engg_loading_dock_01_gnss` |
| LiDAR topic | `/velodyne_points` (`sensor_msgs/PointCloud2`) |
| Frame | `lidar_vlp16_points_link` |

## Workspace layout

```
eng_loading_dock_map/
  venv/                         # isolated Python 3.10 (kiss-icp, open3d, rosbags)
  scripts/run_kiss_icp_and_build_map.py
  docs/                         # this file + configs + pip freeze
  intermediate/full_run/        # poses, metrics, kiss-frame PCD copy, logs
  results/                      # quality JSON + top-down preview
  pointcloud_map.pcd            # Autoware-style binary PCD (KISS-ICP odometry frame)
```

## Steps executed

### 1. Create isolated directories

```bash
MAP_ROOT=/home/agxorin/autoware_data/maps/eng_loading_dock_map
mkdir -p "$MAP_ROOT"/{kiss_icp_ws/src,venv,intermediate,docs,scripts,results}
```

### 2. Create venv and install KISS-ICP + Open3D

```bash
python3 -m venv "$MAP_ROOT/venv"
source "$MAP_ROOT/venv/bin/activate"
python -m pip install --upgrade pip wheel setuptools
pip install "kiss-icp[all]" open3d
# Verified: kiss-icp 1.3.0, open3d 0.18.0
```

### 3. Smoke test (30 scans)

```bash
export kiss_icp_out_dir="$MAP_ROOT/intermediate/smoke_test"
kiss_icp_pipeline --topic /velodyne_points --n-scans 30 \
  /home/agxorin/Desktop/engg_loading_dock_01_gnss
```

Confirmed rosbag dataloader works (~100 Hz odometry on Orin for VLP-16).

### 4. Full map build (odometry + voxel accumulation)

KISS-ICP alone writes **poses**, not a global PCD. The helper script runs KISS-ICP and accumulates deskewed scans into a 0.2 m voxel map, then writes an Autoware-like binary PCD (`FIELDS x y z _`).

```bash
source "$MAP_ROOT/venv/bin/activate"
python "$MAP_ROOT/scripts/run_kiss_icp_and_build_map.py" \
  --bag /home/agxorin/Desktop/engg_loading_dock_01_gnss \
  --topic /velodyne_points \
  --config "$MAP_ROOT/docs/kiss_icp_eng_dock_config.yaml" \
  --out-dir "$MAP_ROOT/intermediate/full_run" \
  --final-pcd "$MAP_ROOT/pointcloud_map.pcd" \
  --map-voxel 0.2 \
  --scan-stride 2 \
  --max-range 80 \
  --n-scans -1
```

**Parameters chosen**

| Param | Value | Reason |
|-------|-------|--------|
| `max_range` | 80 m | VLP-16 practical outdoor range; reduces far noise |
| `map_voxel` | 0.2 m | Autoware-friendly density vs memory |
| `scan_stride` | 2 | Use every 2nd scan (~5 Hz) for map density/runtime tradeoff |
| deskew | true (default) | Use KISS-ICP motion compensation |

Runtime ≈ **222 s** for 4918 odometry frames / 2459 map scans.

### 5. Validation (Open3D)

Automated checks wrote:

- `intermediate/full_run/validation_open3d.json`
- `results/quality_deep.json`
- `results/topdown_preview.png`

## Outputs

| Artifact | Location |
|----------|----------|
| **pointcloud_map.pcd** | `.../eng_loading_dock_map/pointcloud_map.pcd` (~16 MB, ~1.00e6 points) |
| Poses (npy/kitti/tum) | `intermediate/full_run/2026-08-12_21-47-44/` |
| Build summary | `intermediate/full_run/build_summary.json` |
| Pip freeze | `docs/venv_requirements_freeze.txt` |

## Quality assessment (honest)

### What looks quantitatively OK

- Non-empty map: **1,004,344** points after 0.2 m voxelization.
- Spatial extent ~ **260 m** bbox diagonal — consistent with driving in a loading-dock yard.
- KISS net displacement **62.9 m** ≈ GNSS net displacement **62.4 m** (good agreement on endpoints).
- Trajectory Z span only **~4.2 m** (no catastrophic pitch/Z blow-up).
- Local NN median **~0.14 m** (points are dense; density alone is fine).

### What looks visually poor (important)

Top-down preview `results/topdown_preview.png` shows **clear registration drift**:

- Many building/wall edges appear as **parallel ghost lines** (double walls), especially around the perimeter.
- Structure boundaries look **thick / fuzzy**, not sharp single surfaces.
- The LO trajectory overlay is **jagged** relative to what GNSS suggests should be a smoother ~200 m path (KISS path length 295 m vs GNSS 204 m).

This is expected failure mode for **odometry-only** mapping over ~8 minutes with **no loop closure** and a sparse VLP-16. Automated “no vertical double-layer” probes can miss primarily **horizontal** ghosting.

### Other caveats

1. **Frame is NOT Autoware `map` / MGRS / UTM.** Origin is the first LiDAR pose (identity). Carter Lanelet2 will **not** overlay until georeferencing.
2. **GNSS was not fused** into poses used for accumulation.
3. **Extrinsics are unknown** (`sensor_kit_calibration.yaml` still null). Map is in `lidar_vlp16_points_link` odometry, not `base_link`.

**Verdict:** `poor_for_autoware_localization_as_is` — useful as a **pipeline proof / first look**, **not** ready for NDT localization or as a production Autoware `pointcloud_map.pcd`. Next work should add GNSS-constrained poses and/or loop-closure SLAM before treating the map as successful.

## What was NOT done (next steps)

1. Align / scale KISS trajectory to NovAtel `/novatel/oem7/fix` and write `map_projector_info.yaml`.
2. Place Carter `lanelet2_map.osm` beside the georeferenced PCD.
3. Optional: denser rebuild (`--scan-stride 1`, `--map-voxel 0.1`) if NDT needs more detail.
4. Optional: install a loop-closure SLAM if ghosting appears after visual review.

## Safety / isolation notes

- Sample maps under `sample-map-rosbag` / `sample-map-planning` were **not** overwritten.
- Autoware workspace was not used as install target; only an accidental `kiss_icp.yaml` dump in the Autoware cwd during config probe was deleted.
- Activate mapping tools with: `source ~/autoware_data/maps/eng_loading_dock_map/venv/bin/activate`
