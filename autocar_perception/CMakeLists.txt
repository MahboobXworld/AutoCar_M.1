cmake_minimum_required(VERSION 3.0.2)
project(autocar_perception)
find_package(catkin REQUIRED COMPONENTS rospy std_msgs geometry_msgs sensor_msgs visualization_msgs autocar_interfaces)
catkin_package()