# Copyright(c) 2020 eaibot limited.
cmake_minimum_required(VERSION 3.8)
project(ydlidar_ros2_driver VERSION 1.0.2 LANGUAGES C CXX)

##################ros2#############################################
# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

####################find package#####################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)

############## YDLIDAR SDK START#####################################
# Prefer the adjacent SDK source tree used on this robot. Build it inside this
# package's build directory so SDK source changes cannot leave a stale archive
# linked into the ROS node. Fall back to the system package for standalone
# builds that do not have the adjacent source checkout.
set(YDLIDAR_SDK_LOCAL_ROOT "" CACHE PATH "Path to a YDLidar-SDK source tree")
set(_adjacent_sdk_root "${PROJECT_SOURCE_DIR}/../../../YDLidar-SDK")
if(NOT YDLIDAR_SDK_LOCAL_ROOT AND
  EXISTS "${_adjacent_sdk_root}/CMakeLists.txt")
  set(YDLIDAR_SDK_LOCAL_ROOT "${_adjacent_sdk_root}")
endif()

if(YDLIDAR_SDK_LOCAL_ROOT)
  if(NOT EXISTS "${YDLIDAR_SDK_LOCAL_ROOT}/CMakeLists.txt")
    message(FATAL_ERROR
      "YDLIDAR_SDK_LOCAL_ROOT is not an SDK source tree: "
      "${YDLIDAR_SDK_LOCAL_ROOT}")
  endif()

  include(ExternalProject)
  set(_sdk_build_dir "${CMAKE_CURRENT_BINARY_DIR}/ydlidar_sdk_build")
  set(_sdk_archive "${_sdk_build_dir}/libydlidar_sdk.a")

  externalproject_add(ydlidar_sdk_local_build
    SOURCE_DIR "${YDLIDAR_SDK_LOCAL_ROOT}"
    BINARY_DIR "${_sdk_build_dir}"
    CMAKE_ARGS
      "-DBUILD_SHARED_LIBS=OFF"
      "-DBUILD_EXAMPLES=OFF"
      "-DBUILD_CSHARP=OFF"
      "-DBUILD_TEST=OFF"
      "-DCMAKE_C_COMPILER=${CMAKE_C_COMPILER}"
      "-DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}"
    BUILD_COMMAND
      "${CMAKE_COMMAND}" -E env "MAKEFLAGS="
      "${CMAKE_COMMAND}" --build <BINARY_DIR> --target ydlidar_sdk --parallel 2
    INSTALL_COMMAND ""
    BUILD_ALWAYS TRUE
    BUILD_BYPRODUCTS "${_sdk_archive}"
  )

  add_library(ydlidar_sdk_local STATIC IMPORTED GLOBAL)
  set_target_properties(ydlidar_sdk_local PROPERTIES
    IMPORTED_LOCATION "${_sdk_archive}"
  )
  set(YDLIDAR_SDK_INCLUDE_DIRS "${YDLIDAR_SDK_LOCAL_ROOT}")
  set(YDLIDAR_SDK_LIBRARIES
    ydlidar_sdk_local pthread rt)
  message(STATUS "Building local YDLidar SDK: ${YDLIDAR_SDK_LOCAL_ROOT}")
else()
  find_package(ydlidar_sdk REQUIRED)
endif()
############## YDLIDAR SDK END#####################################

#Include directories
include_directories(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/src
  ${PROJECT_SOURCE_DIR}/include
)
include_directories(SYSTEM ${YDLIDAR_SDK_INCLUDE_DIRS})

#link library directories
link_directories(${YDLIDAR_SDK_LIBRARY_DIRS})

#---------------------------------------------------------------------------------------
# generate excutable and add libraries
#---------------------------------------------------------------------------------------
add_executable(${PROJECT_NAME}_node
    src/${PROJECT_NAME}_node.cpp)
if(TARGET ydlidar_sdk_local_build)
  add_dependencies(${PROJECT_NAME}_node ydlidar_sdk_local_build)
endif()
#---------------------------------------------------------------------------------------
# link libraries
#--------------------------------------------------------------------------------------
ament_target_dependencies(${PROJECT_NAME}_node
  "rclcpp"
  "sensor_msgs"
  "geometry_msgs"
  "std_srvs"
)

target_link_libraries(${PROJECT_NAME}_node
  ${YDLIDAR_SDK_LIBRARIES}
)

target_compile_definitions(${PROJECT_NAME}_node PRIVATE
  YDLIDAR_ROS2_DRIVER_VERSION="${PROJECT_VERSION}"
)

#---------------------------------------------------------------------------------------
# generate excutable and add libraries
#---------------------------------------------------------------------------------------
add_executable(${PROJECT_NAME}_client
    src/${PROJECT_NAME}_client.cpp)
#---------------------------------------------------------------------------------------
# link libraries
#--------------------------------------------------------------------------------------
ament_target_dependencies(${PROJECT_NAME}_client
  "rclcpp"
  "sensor_msgs"
)

#---------------------------------------------------------------------------------------
# Install
#---------------------------------------------------------------------------------------
install(TARGETS
  ${PROJECT_NAME}_node ${PROJECT_NAME}_client
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch params startup config
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES LICENSE
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_scan_utils test/test_scan_utils.cpp)
  target_include_directories(test_scan_utils PRIVATE include)

  ament_add_pytest_test(test_x2_configuration
    test/test_x2_configuration.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  )
endif()

ament_export_include_directories(include)
ament_package()
