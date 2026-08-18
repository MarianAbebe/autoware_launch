# LIO-SAM feasibility — Engineering Loading Dock `_gnss` bag

**Date:** 2026-08-12  
**Decision:** **STOP — do not install/run LIO-SAM on this bag yet.**  
**Reason:** IMU orientation (and other inputs) make an official LIO-SAM result unreliable.  
**Isolation:** No Autoware / sensor-kit / sample-localization changes. KISS-ICP outputs left untouched.

## Dataset vs LIO-SAM requirements

| Input | Bag has | LIO-SAM needs | Status |
|-------|---------|---------------|--------|
| LiDAR PointCloud2 | `/velodyne_points`, VLP-16-like, fields `x,y,z,intensity,ring,time` | Velodyne-style cloud with **ring** + relative **time** | **Mostly OK** (see soft issues) |
| IMU rate | `/xsens/imu/data` ~400 Hz | ≥ ~200 Hz preferred | **OK (rate)** |
| IMU orientation | **quaternion always `[0,0,0,0]`** | **9-axis orientation** (roll/pitch/yaw) for init + GPS heading | **HARD FAIL** |
| IMU↔LiDAR extrinsics | `sensor_kit_calibration.yaml` all **null** | Accurate `extrinsicRot` / `extrinsicRPY` / `extrinsicTrans` | **HARD FAIL** |
| GNSS | `/novatel/oem7/fix` `NavSatFix` status=2 | `gpsTopic` = **`nav_msgs/Odometry`** (from navsat), not raw NavSatFix | **Adapter missing** |
| ROS 2 Humble | Available | Official `ros2` branch supports Humble | **OK** |
| GTSAM | `ros-humble-gtsam` / libgtsam via PPA | Required | Installable, not installed (stopped before) |

## Hard blockers (why we stop)

### 1. No usable 9-axis orientation on any IMU topic

Probed 500–2000 messages each:

| Topic | Orientation |
|-------|-------------|
| `/xsens/imu/data` | **all zeros** |
| `/novatel/oem7/imu/data_raw` | **all zeros** |
| `/mcu_node/data_link/icm/data` | **all zeros** |

Accel/gyro on Xsens look physically plausible (~9.81 m/s² up), but LIO-SAM’s documented contract is:

> “LIO-SAM only works with a 9-axis IMU, which gives roll, pitch, and yaw estimation.”

Without orientation, attitude initialization and IMU-heading GPS init are undefined. Running anyway would be a 6-axis misuse of the stack (unsupported in official LIO-SAM; forks like `liorf` exist but are a different project).

### 2. Unknown LiDAR↔IMU extrinsics

Eng loading dock calibrations are still TODO/null. Wrong extrinsics in LIO-SAM typically produce jumping / inverted gravity / zigzag — exactly the failure modes called out in the upstream README. Guessing identity or default Microstrain extrinsics would not be honest mapping.

### 3. GNSS factor path incomplete on ROS 2

Official ROS 2 branch README states missing features include:

- **“A launch file for the navsat module/GPS factor”**

Config expects `gpsTopic: "odometry/gpsz"` (Odometry), while the bag only has `sensor_msgs/NavSatFix`. Autoware’s own LIO-SAM tutorial converts GNSS via a driver/`use_odometry` path. We would need an isolated `robot_localization` navsat_transform (and TF tree) — doable later, but useless while IMU orientation is empty.

## Soft issues (would matter after blockers are fixed)

1. **Point `time` range** in a sample scan: ~`0.0` … `0.017 s`, not the full `0`–`0.1 s` expected for a 10 Hz VLP-16 sweep. Deskew may be incomplete or the driver time channel may be nonstandard.
2. Cloud is organized **width=16, height=1824** (ring×azimuth). LIO-SAM expects `N_SCAN=16`, `Horizon_SCAN≈1800` — configurable, but verify ring indexing matches their Velodyne path.
3. No `/tf` / `/tf_static` in the bag — any GPS odometry bridge needs a synthetic static TF (lidar/imu/gnss/base_link).

## What “minimal adaptation” would look like (if data were fixed)

Official workflow (isolated ws only):

```bash
mkdir -p ~/autoware_data/maps/eng_loading_dock_map/lio_sam_ws/src
cd ~/autoware_data/maps/eng_loading_dock_map/lio_sam_ws/src
git clone -b ros2 https://github.com/TixiaoShan/LIO-SAM.git
# install libgtsam-dev (PPA) or ros-humble-gtsam — verify CMake find_package
cd .. && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Config adaptations (not applied — blocked):

- `pointCloudTopic: /velodyne_points`
- `imuTopic: /xsens/imu/data`
- `sensor: velodyne`, `N_SCAN: 16`, `Horizon_SCAN: 1824` (or 1800 after verification)
- `gpsTopic:` remapped Odometry from NavSatFix bridge
- Measured extrinsics for Xsens MTi-680G ↔ Velodyne
- Separate output dir from KISS (`lio_sam_results/`, not overwrite `pointcloud_map.pcd`)

## Comparison to KISS-ICP (no new LIO map)

| Metric | KISS-ICP (existing) | LIO-SAM (this attempt) |
|--------|---------------------|-------------------------|
| PCD produced | Yes (~1.0M pts) | **Not run** |
| Trajectory vs GNSS net displacement | ~62.9 m vs ~62.4 m | N/A |
| Path length vs GNSS | 295 m vs 204 m | N/A |
| Wall sharpness / duplicates | **Poor** (ghost walls) | N/A — would not trust without 9-axis + extrinsics |
| GNSS constraints | None | Not usable without Odometry bridge + heading |

## Required data/work before retrying LIO-SAM

1. **Re-record or re-publish Xsens orientation** (AHRS / filter mode) so `sensor_msgs/Imu.orientation` is a valid unit quaternion; confirm covariance policy.
2. **Measure LiDAR↔IMU↔GNSS extrinsics** (or run a lidar–IMU calibrator offline) and fill eng-kit calibration (in a copy used only by the mapping ws — still do not break sample Autoware).
3. Add isolated **NavSatFix → `nav_msgs/Odometry`** + static TF for GPS factors.
4. Then build LIO-SAM in `eng_loading_dock_map/lio_sam_ws`, run bag offline, export PCD beside (not over) KISS results, and compare trajectories/walls again.

## Bottom line

LIO-SAM on ROS 2 Humble **can** consume Velodyne-style clouds and high-rate IMU **in principle**, and GNSS **can** be added via an odometry bridge — but **this bag’s IMUs provide no orientation**, and **extrinsics are unknown**. Installing and forcing a run would produce another PCD without a trustworthy trajectory. Per your instructions: **stopped and reported instead of claiming success.**
