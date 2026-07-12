cmake_minimum_required(VERSION 3.0.2)
project(autocar_behavior)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_geometry_msgs
  actionlib
  actionlib_msgs
  move_base_msgs
  autocar_interfaces
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs sensor_msgs tf2 tf2_geometry_msgs actionlib actionlib_msgs move_base_msgs autocar_interfaces
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(behavior_decision_manager_node
  src/behavior_decision_manager_node.cpp
  src/behavior_decision_manager.cpp
  src/decision_tree_baseline.cpp
)
target_link_libraries(behavior_decision_manager_node ${catkin_LIBRARIES})
add_dependencies(behavior_decision_manager_node autocar_interfaces_generate_messages_cpp)