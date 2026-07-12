# autocar_localization

Handles state estimation, sensor-fused odometry, LiDAR scan deskewing, mapping, and localization health management.

## 📂 Structure
- **`config/`**
  - `localization/ekf.yaml`: Configuration for the `robot_localization` EKF node, fusing wheel encoders, IMU, and scan matching odometry.
- **`maps/`**
  - `my_map.yaml` & `my_map.pgm`: Saved 2D occupancy grid maps for global navigation.
- **`scripts/`**
  - `lidar_deskew_node.py`: Corrects LiDAR point cloud distortion caused by vehicle movement.
  - `localization_manager_node.py`: Monitors EKF status, SLAM convergence, covariance thresholds, and updates local database logs.

## 🔗 Fusion Details
1. **Odom-Frame EKF**: Fuses `/joint_states` (wheel encoders), `/imu/data`, and `/scanmatch_odom` (from Hector SLAM) to publish `/odometry/filtered`.
2. **Global AMCL**: Localizes the vehicle inside the static map file using particle filtering.
3. **Hector SLAM**: Active scan matcher configured in non-broadcast mode, producing high-confidence odometry.
