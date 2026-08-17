cmake_minimum_required(VERSION 3.5)
project(eng_loading_dock_sensor_kit_launch)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(PROGRAMS
  scripts/gnss_to_velocity_report.py
  scripts/gnss_pose_altitude_corrector.py
  scripts/initialize_after_ready.py
  scripts/biased_pose_lidar_time_sync.py
  scripts/velodyne_points_to_xyzirc.py
  scripts/dedupe_vector_map_markers.py
  scripts/apply_measured_sensor_calibration.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package(INSTALL_TO_SHARE
  launch
  data
  config
)
