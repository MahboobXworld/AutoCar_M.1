cmake_minimum_required(VERSION 3.0.2)
project(autocar_localization)
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs nav_msgs sensor_msgs amcl map_server hector_mapping robot_localization autocar_interfaces)
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs sensor_msgs amcl map_server hector_mapping robot_localization autocar_interfaces
)