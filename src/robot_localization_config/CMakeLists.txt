cmake_minimum_required(VERSION 3.8)
project(robot_localization_config)
find_package(ament_cmake REQUIRED)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_ekf_configuration
    test/test_ekf_configuration.py
  )
endif()

ament_package()
