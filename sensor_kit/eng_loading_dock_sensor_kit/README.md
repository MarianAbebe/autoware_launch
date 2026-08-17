# Engineering Loading Dock Sensor Kit

This sensor kit is the Autoware sensor-model scaffold for the Engineering Loading Dock platform.
It follows the standard Autoware split between:

- `eng_loading_dock_sensor_kit_launch`: sensing launch entrypoints, driver wrappers, and runtime config/data placeholders.
- `eng_loading_dock_sensor_kit_description`: Xacro frame hierarchy and calibration placeholders.

It is intended to be selected with:

```bash
sensor_model:=eng_loading_dock_sensor_kit
vehicle_model:=ugv_vehicle
```

## Bag-Derived Sensor Inventory

The current package is based on the Engineering Loading Dock rosbag inventory.

| Sensor | Recorded topic | Message type | Frame |
|---|---|---|---|
| LiDAR | `/velodyne_points` | `sensor_msgs/msg/PointCloud2` | `lidar_vlp16_points_link` |
| Camera 0 | `/cam_0/image_raw` | `sensor_msgs/msg/Image` | `cam_0_link` |
| Camera 1 | `/cam_1/image_raw` | `sensor_msgs/msg/Image` | `cam_1_link` |
| IMU | `/xsens/imu/data` | `sensor_msgs/msg/Imu` | `imu_mti680g_link` |
| IMU | `/novatel/oem7/imu/data_raw` | `sensor_msgs/msg/Imu` | `imu_eg320_link` |
| IMU | `/mcu_node/data_link/icm/data` | `sensor_msgs/msg/Imu` | `imu_icm20948_link` |
| GNSS | `/novatel/oem7/fix` | `sensor_msgs/msg/NavSatFix` | `gnss_oem7_link` |
| GNSS | `/zedf9p/fix` | `sensor_msgs/msg/NavSatFix` | `gnss_f9p_link` |

The bag did not contain `/tf`, `/tf_static`, `/clock`, or vehicle-state topics.

## Frame Hierarchy

The description package currently expands to:

```text
base_link
└── sensor_kit_base_link
    ├── cam_0_link
    ├── cam_1_link
    ├── gnss_f9p_link
    ├── gnss_oem7_link
    ├── imu_eg320_link
    ├── imu_icm20948_link
    ├── imu_mti680g_link
    └── lidar_vlp16_points_link
```

All sensor origins are syntactic zero placeholders in Xacro so the URDF remains expandable.
They are not calibration values.

## Current Assumptions

- The LiDAR is likely a Velodyne VLP-16, inferred from topic names and pointcloud ring count.
- Camera driver/vendor, serial numbers, transport settings, and intrinsics are unknown.
- The primary runtime IMU and GNSS source have not been selected yet.
- `sensor_kit_calibration.yaml` and `sensors_calibration.yaml` are consumed by the Xacro files.
- Current LiDAR/GNSS calibration values are estimated for bag replay: horizontal lever arms are still zero, and vertical LiDAR/NovAtel GNSS height is set to 2.05 m from the map altitude convention. Replace these with measured values before deployment.

## Validation Performed

The package has been validated with:

```bash
colcon build \
  --packages-select \
  eng_loading_dock_sensor_kit_description \
  eng_loading_dock_sensor_kit_launch \
  --cmake-args -DBUILD_TESTING=OFF
```

Full vehicle description expansion was also checked through:

```bash
xacro tier4_vehicle_launch/urdf/vehicle.xacro \
  vehicle_model:=ugv_vehicle \
  sensor_model:=eng_loading_dock_sensor_kit
```

`check_urdf` successfully parsed the expanded vehicle URDF and reported the expected `base_link -> sensor_kit_base_link -> sensor frames` tree.

## Remaining Hardware-Dependent Tasks

- Measure `base_link -> sensor_kit_base_link`.
- Measure `sensor_kit_base_link -> each sensor frame`.
- Use `CALIBRATION_MEASUREMENTS.md` and
  `apply_measured_sensor_calibration.py` to apply measured LiDAR/GNSS lever arms.
- Decide whether to represent the raw Velodyne packet frame `lidar_vlp16_packs_link`.
- Select and configure the LiDAR driver.
- Select the primary IMU source and generate valid `imu_corrector` parameters.
- Select the primary GNSS receiver and configure `gnss_poser`.
- Identify camera models, serials, transport configuration, and camera_info publishing path.
- Calibrate camera intrinsics and add valid camera_info YAML files.
- Replace estimated LiDAR/GNSS vertical offsets and zero horizontal lever arms with measured phase-center/sensor-origin values.
