cmake_minimum_required(VERSION 3.0.2)
project(autocar_gazebo)
find_package(catkin REQUIRED COMPONENTS gazebo_ros xacro controller_manager robot_state_publisher)
catkin_package()