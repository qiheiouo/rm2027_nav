cmake_minimum_required(VERSION 3.16)
project(neu_l1_offline_probe)
set(CMAKE_CXX_STANDARD 17)
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common features filters io registration search)
find_package(Eigen3 REQUIRED)
add_executable(registration_probe
  registration_probe.cpp
  ${PROJECT_ROOT}/src/rm_gicp_relocalization/src/relocalization_math.cpp
  ${PROJECT_ROOT}/src/rm_relocalization_bridge/src/global_pose_gate.cpp
  ${QUALITY_ROOT}/src/registration_quality.cpp)
target_include_directories(registration_probe PRIVATE
  ${PROJECT_ROOT}/src/rm_gicp_relocalization/include
  ${PROJECT_ROOT}/src/rm_relocalization_bridge/include
  ${QUALITY_ROOT}/include
  ${PCL_INCLUDE_DIRS})
target_link_libraries(registration_probe ${PCL_LIBRARIES} Eigen3::Eigen)
ament_target_dependencies(registration_probe geometry_msgs)
