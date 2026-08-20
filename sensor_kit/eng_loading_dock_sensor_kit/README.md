# Engineering Loading Dock Replay Pipeline

This package documents the replay-only Autoware workflow for the Engineering Loading Dock dataset.
It is the primary entry point for developers who need to run, verify, or debug replay of:

- Localization
- Perception
- Planning
- Control

The launch files in this package are thin wrappers around upstream Autoware launch architecture.
They keep the replay graph in one place, disable real vehicle actuation, and reuse the same
sensor-kit and map setup across all replay stages.

## Overview

### Purpose

The goal of the Engineering Loading Dock replay package is to run recorded sensor data through
Autoware without touching physical hardware. It is intended for:

- replay bring-up
- regression testing
- end-to-end pipeline checks
- debugging launch/configuration changes on the Engineering Loading Dock platform

The replay path assumes the dataset provides sensor input, while the launch files provide the
Autoware stack, static sensor model, localization, planning, and control wiring.

### Pipeline relationship

The stages depend on each other in the usual Autoware order:

1. Localization estimates vehicle pose and kinematic state from replayed sensor data and the map.
2. Perception consumes the same replayed sensors and produces detected objects and related outputs.
3. Planning consumes localization, perception, map, and route input to produce a trajectory.
4. Control consumes the planning trajectory and produces control commands, but without vehicle
   interface access in this replay workflow.

### Replay architecture

The Engineering Loading Dock replay launchers all call upstream `autoware.launch.xml` directly.
They do not duplicate the upstream launch graph. Instead, they:

- select `sensor_model:=eng_loading_dock_sensor_kit`
- keep `use_sim_time:=true`
- disable `launch_sensing_driver`
- disable `launch_vehicle_interface`
- override the localization component with the replay NDT component
- enable only the stages required for the chosen replay level

`full_stack_replay_v2.launch.xml` is a thin wrapper around `control_replay_v2.launch.xml` so there
is one canonical end-to-end replay entry point.

### Difference from the standard Autoware logging simulator

Compared with the generic logging simulator flow, this replay stack is narrower and more explicit:

- it is tied to the Engineering Loading Dock sensor kit and map
- it uses a replay-specific localization component and replay parameter set
- it keeps the vehicle interface off, even in planning and control replay
- it exposes one consistent replay path for localization, perception, planning, and control
- it avoids copying the upstream logging-simulator include graph into this package

In short: use the upstream Autoware architecture, but with the Engineering Loading Dock overrides
documented here.

## Repository Layout

All replay launch files live in:

`sensor_kit/eng_loading_dock_sensor_kit/eng_loading_dock_sensor_kit_launch/launch/`

### `localization_replay_v2.launch.xml`

- What it launches:
  - upstream `autoware.launch.xml`
  - vehicle, map, sensing, and localization
  - replay localization component only
  - no perception, planning, or control
- Intended use:
  - localization-only replay validation
  - map and TF bring-up
  - pose estimation and EKF checks before enabling downstream stages
- Dependencies:
  - `eng_loading_dock_sensor_kit_launch`
  - `eng_loading_dock_sensor_kit_description`
  - `autoware_launch`
  - Eng Loading Dock lanelet and pointcloud maps
  - replay bag with sensor topics
- When to use:
  - first stage of replay bring-up
  - when debugging localization without downstream noise

### `perception_replay_v3.launch.xml`

- What it launches:
  - upstream `autoware.launch.xml`
  - vehicle, map, sensing, localization, and perception
  - replay localization component
  - no planning or control
- Intended use:
  - perception replay checks on top of a known-good localization flow
  - sensor preprocessing and object pipeline validation
- Dependencies:
  - same localization dependencies as above
  - upstream perception stack
  - replay bag with the sensor topics required by the perception mode
- When to use:
  - after localization is stable
  - before moving to planning or control

### `planning_replay_v2.launch.xml`

- What it launches:
  - upstream `autoware.launch.xml`
  - vehicle, map, sensing, localization, perception, and planning
  - system stack is enabled so routing and operation-mode interfaces exist
  - control is disabled
- Intended use:
  - mission-planning and trajectory-generation validation
  - route and planning-state verification
- Dependencies:
  - localization and perception dependencies
  - planning stack
  - system/AD API support for route and operation-mode interfaces
  - replay bag plus a route set through the exposed interfaces
- When to use:
  - after localization and perception are already working
  - when checking that planning publishes trajectory output

### `control_replay_v2.launch.xml`

- What it launches:
  - upstream `autoware.launch.xml`
  - vehicle, map, sensing, localization, perception, planning, and control
  - system stack is enabled
  - vehicle interface remains disabled
- Intended use:
  - control-stack validation on top of a published planning trajectory
  - command-generation checks without any physical actuation
- Dependencies:
  - localization, perception, and planning dependencies
  - control stack
  - system/AD API support
  - replay bag plus a valid route
- When to use:
  - when trajectory publication is already verified
  - when validating control outputs without a vehicle interface

### `full_stack_replay_v2.launch.xml`

- What it launches:
  - the full replay stack through `control_replay_v2.launch.xml`
  - localization, perception, planning, and control
- Intended use:
  - single entry point for end-to-end replay demonstrations
  - developer handoff and integration checks
- Dependencies:
  - same as `control_replay_v2.launch.xml`
- When to use:
  - when you want one launch file for the whole replay pipeline

## Setup

### Source ROS and the workspace

```bash
source /opt/ros/humble/setup.bash
source <AUTOWARE_WS>/install/setup.bash
```

If you keep your own overlay workspace, source it after the ROS base install and before launching.

### Required packages

At minimum, the workspace must provide:

- `autoware_launch`
- `eng_loading_dock_sensor_kit_launch`
- `eng_loading_dock_sensor_kit_description`
- the upstream Autoware localization, perception, planning, control, and system launch stacks
- the usual Autoware AD API and lifecycle support packages

This package also relies on standard ROS 2 utilities such as `xacro`, `rclpy`, and `topic_tools`.

### Required map

The map path must point to a directory that contains the Eng Loading Dock map files:

- `lanelet2_map.osm`
- `pointcloud_map.pcd`

Use:

`<MAP_PATH>`

If your files use different names, pass `lanelet2_map_file` and `pointcloud_map_file` explicitly.

### Required rosbag

Use the Engineering Loading Dock replay bag:

`<BAG_PATH>`

The bag is expected to provide the sensor data used by localization and perception.
It does not need to provide TF, static TF, or `/clock`, because the replay launch and bag playback
cover those roles separately.

### Common placeholders

- `<AUTOWARE_WS>`: workspace root that contains `install/`
- `<MAP_PATH>`: directory containing the lanelet2 and pointcloud map files
- `<BAG_PATH>`: rosbag directory to replay
- `<DATA_PATH>`: Autoware artifact/model directory used by planning and control launch files
- `<VEHICLE_ID>`: vehicle-specific identifier if your environment requires one

## Launch Commands

Run each launch file from a shell that has ROS and the workspace sourced.

### Localization

```bash
ros2 launch eng_loading_dock_sensor_kit_launch localization_replay_v2.launch.xml \
  map_path:=<MAP_PATH> \
  vehicle_model:=ugv_vehicle \
  vehicle_id:=<VEHICLE_ID> \
  rviz:=false
```

### Perception

```bash
ros2 launch eng_loading_dock_sensor_kit_launch perception_replay_v3.launch.xml \
  map_path:=<MAP_PATH> \
  vehicle_model:=ugv_vehicle \
  vehicle_id:=<VEHICLE_ID> \
  rviz:=false
```

### Planning

```bash
ros2 launch eng_loading_dock_sensor_kit_launch planning_replay_v2.launch.xml \
  map_path:=<MAP_PATH> \
  vehicle_model:=ugv_vehicle \
  vehicle_id:=<VEHICLE_ID> \
  data_path:=<DATA_PATH> \
  planning_module_preset:=default \
  control_module_preset:=default \
  rviz:=false
```

### Control

```bash
ros2 launch eng_loading_dock_sensor_kit_launch control_replay_v2.launch.xml \
  map_path:=<MAP_PATH> \
  vehicle_model:=ugv_vehicle \
  vehicle_id:=<VEHICLE_ID> \
  data_path:=<DATA_PATH> \
  planning_module_preset:=default \
  control_module_preset:=default \
  rviz:=false
```

### Full stack

```bash
ros2 launch eng_loading_dock_sensor_kit_launch full_stack_replay_v2.launch.xml \
  map_path:=<MAP_PATH> \
  vehicle_model:=ugv_vehicle \
  vehicle_id:=<VEHICLE_ID> \
  data_path:=<DATA_PATH> \
  planning_module_preset:=default \
  control_module_preset:=default \
  rviz:=false
```

## Rosbag Replay

Start the bag in a second terminal after sourcing the same ROS environment:

```bash
ros2 bag play <BAG_PATH> --clock -r 1.0
```

Notes:

- `--clock` is required because the replay launch files use `use_sim_time:=true`.
- `-r 1.0` is the recommended default rate for first-pass validation.
- If you want to hold the system before playback, use `--start-paused` and then resume once the
  launch graph is up.
- If your bag is large, keep playback on the same machine as the launch stack to avoid I/O noise
  during debugging.

## Verification

The checks below are the fastest way to confirm each stage is healthy.

### Localization

Expected nodes:

- nodes under `/localization/pose_estimator`
- nodes under `/localization/pose_twist_fusion_filter`
- nodes under `/localization/util`

Expected topics:

- `/localization/pose_estimator/pose_with_covariance`
- `/localization/kinematic_state`
- `/localization/pose_with_covariance`
- `/localization/util/downsample/pointcloud`
- `/tf`
- `/tf_static`

Expected TF frames:

- `map`
- `odom`
- `base_link`
- `sensor_kit_base_link`
- sensor frames such as `lidar_vlp16_points_link`, `cam_0_link`, `cam_1_link`, and the IMU/GNSS
  frames used by the sensor kit

Expected services:

- localization trigger and initialization services under `/localization`
- verify with `ros2 service list | rg 'localization|initialize|trigger'`

Expected lifecycle state:

- active

### Perception

Expected nodes:

- nodes under `/perception`
- pointcloud processing components in the shared pointcloud container

Expected topics:

- perception object and obstacle outputs under `/perception`
- the sensor topics coming from the replay bag
- `/tf`
- `/tf_static`

Expected TF frames:

- same static frames as localization
- no new TF frames should be required just for replay perception

Expected services:

- perception node services only if the chosen perception modules expose them
- there should be no dependence on vehicle-interface services

Expected lifecycle state:

- active

### Planning

Expected nodes:

- nodes under `/planning`
- mission-planning and scenario-planning components
- system/AD API support nodes when `launch_system=true`

Expected topics:

- `/planning/trajectory`
- `/planning/mission_planning/route`
- `/planning/route_state`

Expected TF frames:

- same as localization
- planning should consume the existing `map`, `odom`, and `base_link` tree

Expected services:

- route services such as `clear_route`, `set_lanelet_route`, and `set_waypoint_route`
- operation-mode and related system services under `/system` or `/api`
- verify the exact service names with `ros2 service list | rg 'route|operation_mode'`

Expected lifecycle state:

- active

### Control

Expected nodes:

- nodes under `/control`
- the control stack and its helpers
- system/AD API support nodes when `launch_system=true`

Expected topics:

- `/control/command/control_cmd`
- `/control/command/gear_cmd`
- `/control/command/turn_indicators_cmd`
- `/control/command/hazard_lights_cmd`
- `/control/command/emergency_cmd`
- `/control/current_gate_mode`
- `/system/operation_mode/state`

Expected TF frames:

- same as localization
- control should not introduce new frames

Expected services:

- operation-mode and engage/emergency-related services under `/system` or `/api`
- verify the exact names with `ros2 service list | rg 'engage|emergency|operation_mode'`

Expected lifecycle state:

- active

### Full stack

When `full_stack_replay_v2.launch.xml` is running, the combined graph should expose the expected
localization, perception, planning, and control outputs at the same time. If only one stage is
present, you likely launched a narrower file instead of the full stack.

## Debugging

### Localization not initializing

- Confirm the bag is playing with `--clock`.
- Confirm `use_sim_time` is enabled by the launch file you started.
- Confirm `<MAP_PATH>` points to the lanelet2 and pointcloud map directory.
- Confirm the replay localization component and the Eng Loading Dock NDT parameter files are being
  loaded.
- Check `ros2 node list` for `/localization/pose_estimator` and
  `/localization/pose_twist_fusion_filter`.

### Missing TF

- Check that localization is active.
- Check that the bag is replaying sensor data.
- Check `ros2 run tf2_ros tf2_echo map base_link`.
- Check `ros2 run tf2_ros tf2_echo base_link sensor_kit_base_link`.
- If static frames are missing, verify the sensor-kit description package was built and sourced.

### Planning not publishing trajectory

- Confirm `planning_replay_v2.launch.xml` or `full_stack_replay_v2.launch.xml` was used.
- Confirm localization and perception are already healthy.
- Confirm a route has been set through the route services.
- Check `ros2 topic echo /planning/route_state`.
- Check `ros2 topic hz /planning/trajectory`.
- If no route exists, planning may stay alive but remain idle.

### Control not publishing commands

- Confirm `control_replay_v2.launch.xml` or `full_stack_replay_v2.launch.xml` was used.
- Confirm planning is publishing `/planning/trajectory`.
- Confirm the vehicle interface is still disabled in replay and that is intentional.
- Check `ros2 topic hz /control/command/control_cmd`.
- Check `ros2 topic echo /control/current_gate_mode`.

### Missing route

- This is usually a workflow issue, not a launch failure.
- Use the AD API route services exposed by the planning/system stack.
- Verify the route-related service names with `ros2 service list | rg 'route'`.

### Missing map

- Confirm `<MAP_PATH>` contains both `lanelet2_map.osm` and `pointcloud_map.pcd`.
- If the filenames differ, pass `lanelet2_map_file:=...` and `pointcloud_map_file:=...`.
- A wrong map path will often surface as launch errors or a silent lack of localization progress.

### DDS/network issues

- Treat middleware, socket, and network failures as environment problems first.
- Do not assume they are caused by the replay launch files.
- If the XML is valid and packages are present, but the runtime dies in middleware setup, stop and
  debug the environment separately.

### RViz issues

- RViz is optional and defaults to `false` in these replay launch files.
- If RViz fails, leave it off while validating the replay pipeline.
- Do not treat rendering failures as replay failures.

### Missing parameters

- Confirm the selected planning and control presets exist in your Autoware branch.
- Confirm `<DATA_PATH>` is valid if the launch file expects model or artifact paths.
- Rebuild and resource the workspace after changing launch or config files.

### Launch include failures

- Verify the package that owns the include is installed and sourced.
- Verify the include path points at the local replay launch file, not an old logging-simulation path.
- Check for stale argument names after an upstream Autoware branch update.

### Missing packages

- Rebuild the workspace.
- Source the ROS base and workspace overlay again.
- Check that `autoware_launch`, `eng_loading_dock_sensor_kit_launch`, and
  `eng_loading_dock_sensor_kit_description` are visible to `ros2 pkg list`.

## Useful ROS Commands

```bash
ros2 node list
ros2 topic list
ros2 topic hz /planning/trajectory
ros2 topic echo /planning/route_state
ros2 service list
ros2 lifecycle nodes
ros2 lifecycle get /<node_name>
ros2 lifecycle set /<node_name> activate
ros2 run tf2_ros tf2_echo map base_link
```

Useful filters:

```bash
ros2 service list | rg 'route|operation_mode|engage|emergency|localization'
ros2 topic list | rg 'localization|perception|planning|control|system|tf'
ros2 node list | rg 'localization|perception|planning|control'
```

## Validation Checklist

Before declaring replay successful, confirm all of the following:

1. ROS and the workspace are sourced.
2. `<MAP_PATH>` points at the correct Eng Loading Dock map directory.
3. The correct replay launch file is running.
4. `ros2 bag play <BAG_PATH> --clock -r 1.0` is active.
5. Localization publishes pose and kinematic state.
6. Perception nodes are alive when enabled.
7. Planning publishes `/planning/trajectory` and `/planning/mission_planning/route`.
8. Control publishes `/control/command/control_cmd` and the related command topics.
9. Route services are available and a route has been set.
10. No launch include, package, or parameter errors are present.

## Notes

- Launch one replay entry point at a time. If you start the full stack entry point, do not also start
  the narrower localization, perception, planning, or control launch files in parallel.
- Repository problems usually look like XML errors, missing includes, wrong argument names, or
  missing package references.
- Runtime or environment problems usually look like missing map data, missing bag playback,
  simulation-time mismatches, or absent routes.
- Jetson-specific problems usually look like resource limits, storage pressure, thermal throttling,
  or local middleware/runtime constraints.
- General Autoware problems usually come from upstream package/version drift or module preset
  mismatches. Compare against the current branch before changing this package.
