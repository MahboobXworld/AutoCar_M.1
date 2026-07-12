cmake_minimum_required(VERSION 3.0.2)
project(autocar_tools)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  roslib
  autocar_interfaces
)

# Find SQLite3
find_package(PkgConfig REQUIRED)
pkg_search_module(SQLITE3 REQUIRED sqlite3)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs sensor_msgs roslib autocar_interfaces
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${SQLITE3_INCLUDE_DIRS}
)

add_executable(experience_logger_node src/experience_logger_node.cpp)
target_link_libraries(experience_logger_node ${catkin_LIBRARIES} ${SQLITE3_LIBRARIES})
add_dependencies(experience_logger_node autocar_interfaces_generate_messages_cpp)

add_executable(mission_performance_logger_node src/mission_performance_logger.cpp)
target_link_libraries(mission_performance_logger_node ${catkin_LIBRARIES} ${SQLITE3_LIBRARIES})
add_dependencies(mission_performance_logger_node autocar_interfaces_generate_messages_cpp)

add_executable(decision_trace_logger_node src/decision_trace_logger.cpp)
target_link_libraries(decision_trace_logger_node ${catkin_LIBRARIES} ${SQLITE3_LIBRARIES})
add_dependencies(decision_trace_logger_node autocar_interfaces_generate_messages_cpp)