cmake_minimum_required(VERSION 3.8)
project(diff_drive_robot)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── Dependencies ──────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(Armadillo REQUIRED)

# lgpio (C library — not a ROS package)
find_library(LGPIO_LIB lgpio REQUIRED)

# ── Include Directories ───────────────────────────────────────────────────────
include_directories(
  include
  ${ARMADILLO_INCLUDE_DIRS}
)

# ── Turtlelib Core Math Library ───────────────────────────────────────────────
add_library(turtlelib_core
  src/rigid2d.cpp
  src/diff_drive_lib.cpp
  src/ekf.cpp
  src/ml.cpp
)

target_include_directories(turtlelib_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(turtlelib_core
  ${ARMADILLO_LIBRARIES}
)

# ── Hardware Driver Executable ────────────────────────────────────────────────
add_executable(diff_drive_robot
  src/main.cpp
  src/motor_driver.cpp
  src/encoder.cpp
  src/pid.cpp
  src/odometry.cpp
)

target_include_directories(diff_drive_robot PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(diff_drive_robot
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

target_link_libraries(diff_drive_robot
  ${LGPIO_LIB}
)

# ── SLAM Feature Extraction Node ──────────────────────────────────────────────
add_executable(landmarks_node
  src/landmarks_node.cpp
)

target_include_directories(landmarks_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(landmarks_node
  rclcpp
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
)

target_link_libraries(landmarks_node
  turtlelib_core
)

# ── EKF SLAM Estimation Node ──────────────────────────────────────────────────
add_executable(ekf_slam_node
  src/ekf_slam_node.cpp
)

target_include_directories(ekf_slam_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(ekf_slam_node
  rclcpp
  sensor_msgs
  nav_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

target_link_libraries(ekf_slam_node
  turtlelib_core
  ${ARMADILLO_LIBRARIES}
)

# ── Install Rules ─────────────────────────────────────────────────────────────
install(TARGETS
  diff_drive_robot
  landmarks_node
  ekf_slam_node
  turtlelib_core
  DESTINATION lib/${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

install(DIRECTORY include/
  DESTINATION include
)

install(PROGRAMS
  scripts/encoder_diag.py
  scripts/odom_teleop_diag.py
  scripts/test_motors.py
  scripts/yaw_diagnostic.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_core_math
    test/test_core_math.cpp
  )
  target_link_libraries(test_core_math
    turtlelib_core
  )
endif()

ament_package()
