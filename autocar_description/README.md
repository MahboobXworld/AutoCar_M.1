# autocar_description

Contains the physical model definition (URDF/Xacro), visual/collision meshes, sensor frames, and default RViz visualization layouts for the 4-wheeled Ackermann steering robot (`autocar`).

## 📂 Structure
- **`urdf/`**
  - `car.xacro`: Main robot model parameterizing geometry, wheels, steering knuckles, links, and joints.
  - `car.gazebo`: Gazebo-specific controllers, sensor plugins (3D LiDAR, IMU, cameras, and joint state publishers).
- **`rviz/`**
  - `nav.rviz`: Default RViz configuration displaying robot tf, LiDAR point clouds, costmaps, and global/local planners.
- **`meshes/`**
  - STL files for robot wheels, chassis, and sensors.

## 📐 Sensor Configurations
- **3D Hokuyo LiDAR**: Attached to `lidar_link`, publishes `/scan`
- **Xsens IMU**: Attached to `imu_link`, publishes `/imu/data`
- **Dual Cameras**: Left and right visual coverage models
- **Ackermann Steering Knuckles**: Actuated steering joints with custom Gazebo transmissions.
