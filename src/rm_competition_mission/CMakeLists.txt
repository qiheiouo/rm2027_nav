cmake_minimum_required(VERSION 3.8)
project(rm_competition_mission)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
# Humble uses BT.CPP 3; Jazzy uses BT.CPP 4.
if("$ENV{ROS_DISTRO}" STREQUAL "humble")
  set(MISSION_BT_PACKAGE behaviortree_cpp_v3)
else()
  set(MISSION_BT_PACKAGE behaviortree_cpp)
endif()
find_package(${MISSION_BT_PACKAGE} REQUIRED)
find_package(action_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rm_competition_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(competition_mission_node src/competition_mission_node.cpp)
ament_target_dependencies(competition_mission_node
  ${MISSION_BT_PACKAGE}
  action_msgs
  geometry_msgs
  nav2_msgs
  rclcpp
  rclcpp_action
  rm_competition_interfaces
  std_msgs
)

add_executable(mission_safety_mock_node src/mission_safety_mock_node.cpp)
ament_target_dependencies(mission_safety_mock_node
  rclcpp
  rm_competition_interfaces
  std_msgs
)

install(TARGETS
  competition_mission_node
  mission_safety_mock_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch trees DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    test_mission_system_runtime
    test/test_mission_system_runtime.py
    TIMEOUT 180
    APPEND_ENV "RM_COMPETITION_MISSION_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}"
  )
  ament_add_pytest_test(
    test_navigation_arbitration_runtime
    test/test_navigation_arbitration_runtime.py
    TIMEOUT 90
    APPEND_ENV "RM_COMPETITION_MISSION_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}"
  )
  ament_add_pytest_test(
    test_three_point_spin_profile
    test/test_three_point_spin_profile.py
    APPEND_ENV "RM_COMPETITION_MISSION_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}"
  )
  ament_add_pytest_test(
    test_three_point_spin_runtime
    test/test_three_point_spin_runtime.py
    TIMEOUT 30
    APPEND_ENV "RM_COMPETITION_MISSION_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}"
  )
endif()

ament_package()
