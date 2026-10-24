cmake_minimum_required(VERSION 3.8)
project(rm_navigation_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_software_debt_launch test/test_software_debt_launch.py
    TIMEOUT 90)
endif()

ament_package()
