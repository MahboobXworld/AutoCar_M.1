cmake_minimum_required(VERSION 3.0.2)
project(autocar_maneuver_planner)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  actionlib
  actionlib_msgs
  autocar_interfaces
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs sensor_msgs visualization_msgs actionlib actionlib_msgs autocar_interfaces
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)