cmake_minimum_required(VERSION 3.0.2)
project(autocar_navigation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_geometry_msgs
  move_base
  dwa_local_planner
  teb_local_planner
  autocar_interfaces
  controller_manager_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs sensor_msgs tf2 tf2_geometry_msgs move_base dwa_local_planner teb_local_planner autocar_interfaces controller_manager_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(ackermann_controller src/ackermann_controller.cpp)
target_link_libraries(ackermann_controller ${catkin_LIBRARIES})

add_executable(goal_stop_manager src/goal_stop_manager.cpp)
target_link_libraries(goal_stop_manager ${catkin_LIBRARIES})