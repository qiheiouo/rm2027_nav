cmake_minimum_required(VERSION 3.8)
project(rm_competition_interfaces)

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ChassisHeadingState.msg"
  "msg/ChassisMode.msg"
  "msg/GimbalState.msg"
  "msg/LocalizationStatus.msg"
  "msg/LocalizationFrame.msg"
  "msg/ManagedIngressStatus.msg"
  "msg/V2MotionTestLease.msg"
  "msg/V2MotionGuardState.msg"
  "msg/MissionState.msg"
  "msg/OperatorNavigationTarget.msg"
  "msg/PostureRequest.msg"
  "msg/PostureState.msg"
  "msg/RefereeState.msg"
  "msg/RobotPosture.msg"
  "msg/SerialConnectionState.msg"
  "msg/SystemReadiness.msg"
  "msg/TargetTrack.msg"
  "msg/PursuitCandidate.msg"
  "srv/SetMissionMode.srv"
  "srv/SetTemporaryMission.srv"
  "srv/SelectLocalizationSource.srv"
  DEPENDENCIES builtin_interfaces geometry_msgs nav_msgs std_msgs
)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
ament_export_include_directories(include/${PROJECT_NAME})
ament_export_dependencies(rosidl_default_runtime Eigen3)
ament_package()
