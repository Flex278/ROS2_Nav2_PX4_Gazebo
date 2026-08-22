cmake_minimum_required(VERSION 3.8)
project(uav_navigation)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config behavior_trees maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
