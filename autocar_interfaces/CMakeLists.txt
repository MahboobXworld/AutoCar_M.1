cmake_minimum_required(VERSION 3.0.2)
project(autocar_interfaces)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  actionlib_msgs
  message_generation
)

add_message_files(
  FILES
  Situation.msg
  BehaviorState.msg
  Maneuver.msg
  ManeuverSequence.msg
  SemanticObject.msg
  SemanticWorldModel.msg
)

add_service_files(
  FILES
  GetRLAction.srv
)

add_action_files(
  FILES
  NavigateBehavior.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS std_msgs geometry_msgs actionlib_msgs message_runtime
)