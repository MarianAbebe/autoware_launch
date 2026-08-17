# Engineering Loading Dock Sensor Measurements

This file defines the measurement convention for the eng loading dock replay
sensor calibration. These values affect the static TF chain used by GNSS
initialization and NDT.

## Coordinate Convention

Measure every value in the vehicle `base_link` frame:

- `x`: forward from `base_link`, meters
- `y`: left from `base_link`, meters
- `z`: up from `base_link`, meters
- `yaw`: counter-clockwise about +z, radians or degrees

For `sample_vehicle`, `base_link` is the vehicle reference frame used by
Autoware, not the GNSS antenna and not the LiDAR.

## Required Measurements

Measure these physical points:

- `base_link -> lidar_vlp16_points_link`
  - LiDAR measurement origin, not the top of the housing.
  - Include lidar yaw if the VLP-16 x-axis is not aligned with vehicle x.

- `base_link -> gnss_oem7_link`
  - NovAtel/OEM7 antenna electrical phase center.
  - Do not use only the antenna mount/housing unless that is the best available
    estimate and is documented.

The current replay estimate is:

```text
base_link -> lidar_vlp16_points_link: x=0.0 y=0.0 z=2.05 yaw=0.0
base_link -> gnss_oem7_link:          x=0.0 y=0.0 z=2.05
```

The `z=2.05` estimate comes from the map altitude convention: map `z=0` is near
NovAtel antenna height and point-cloud ground appears near `z=-2.05`.

## Apply Measurements

Dry run first:

```bash
source /opt/ros/humble/setup.bash
source /home/agxorin/autoware_v2/autoware/install/setup.bash

ros2 run eng_loading_dock_sensor_kit_launch apply_measured_sensor_calibration.py \
  --lidar-x 0.0 --lidar-y 0.0 --lidar-z 2.05 --lidar-yaw-deg 0.0 \
  --gnss-x 0.0 --gnss-y 0.0 --gnss-z 2.05
```

Write the YAML after confirming the dry-run output:

```bash
ros2 run eng_loading_dock_sensor_kit_launch apply_measured_sensor_calibration.py \
  --lidar-x 0.0 --lidar-y 0.0 --lidar-z 2.05 --lidar-yaw-deg 0.0 \
  --gnss-x 0.0 --gnss-y 0.0 --gnss-z 2.05 \
  --apply
```

The tool writes:

- `eng_loading_dock_sensor_kit_description/config/sensors_calibration.yaml`
- `eng_loading_dock_sensor_kit_description/config/sensor_kit_calibration.yaml`

It also creates timestamped `.bak_YYYYmmdd_HHMMSS` backups beside both files.

## Validate

Rebuild after changing the install list or when using a non-symlink install:

```bash
cd /home/agxorin/autoware_v2/autoware
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-select \
  eng_loading_dock_sensor_kit_description \
  eng_loading_dock_sensor_kit_launch \
  --symlink-install --cmake-args -DBUILD_TESTING=OFF
```

Check expanded TF values:

```bash
xacro /home/agxorin/autoware_v2/autoware/src/launcher/autoware_launch/tier4_universe_launch/tier4_vehicle_launch/urdf/vehicle.xacro \
  vehicle_model:=sample_vehicle \
  sensor_model:=eng_loading_dock_sensor_kit | \
  grep -A8 'lidar_vlp16_points_joint'
```

Run the replay verification:

```bash
bash /home/agxorin/autoware_data/maps/eng_loading_dock_map/scripts/run_loc_verify.sh
```

Compare:

- `ndt_pose` rate
- `nvtl` median and p95
- EKF-GNSS position median and p95
- RViz visual overlay in `TopDownOrtho`
