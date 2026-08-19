cmake_minimum_required(VERSION 3.8)
project(slam_nav2_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/auto_map_saver.py
  scripts/exploration_readiness_gate.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(test_auto_map_saver
    test/test_auto_map_saver.py
    TIMEOUT 30
  )
  ament_add_pytest_test(test_mapping_configuration
    test/test_mapping_configuration.py
  )
endif()

ament_package()
