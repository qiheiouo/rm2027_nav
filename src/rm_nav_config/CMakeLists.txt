cmake_minimum_required(VERSION 3.8)
project(rm_nav_config)

find_package(ament_cmake REQUIRED)
set(CMAKE_CXX_STANDARD 17)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rm_competition_interfaces REQUIRED)
add_library(rebase_obstacle_layer SHARED src/rebase_obstacle_layer.cpp)
ament_target_dependencies(rebase_obstacle_layer nav2_costmap_2d pluginlib rm_competition_interfaces)
pluginlib_export_plugin_description_file(nav2_costmap_2d rebase_plugins.xml)
install(TARGETS rebase_obstacle_layer LIBRARY DESTINATION lib)

install(DIRECTORY
  config
  behavior_trees
  maps
  rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/costmap_inspector
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_rebase_obstacle_layer test/test_rebase_obstacle_layer.cpp)
  ament_target_dependencies(test_rebase_obstacle_layer nav2_costmap_2d pluginlib rm_competition_interfaces)
endif()
ament_package()
