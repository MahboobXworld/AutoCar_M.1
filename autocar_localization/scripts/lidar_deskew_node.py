#!/usr/bin/env python3

import rospy
import numpy as np
import math
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

class LidarDeskewNode:
    def __init__(self):
        rospy.init_node('lidar_deskew_node')

        # Cache velocities
        self.v_x = 0.0
        self.w_z = 0.0

        # Subscribers
        self.sub_odom = rospy.Subscriber('/odometry/filtered', Odometry, self.odom_callback, queue_size=1)
        self.sub_scan = rospy.Subscriber('/scan', LaserScan, self.scan_callback, queue_size=5)

        # Publisher
        self.pub_scan = rospy.Publisher('/scan_deskewed', LaserScan, queue_size=5)

        rospy.loginfo("[LidarDeskewNode] Initialized and ready to deskew scans.")

    def odom_callback(self, msg):
        self.v_x = msg.twist.twist.linear.x
        self.w_z = msg.twist.twist.angular.z

    def scan_callback(self, msg):
        # Read velocities at scan start
        v = self.v_x
        w = self.w_z

        # If robot is practically stationary, skip compensation to save CPU
        if abs(v) < 0.01 and abs(w) < 0.01:
            self.pub_scan.publish(msg)
            return

        # Prepare new laser scan message
        new_scan = LaserScan()
        new_scan.header = msg.header
        new_scan.angle_min = msg.angle_min
        new_scan.angle_max = msg.angle_max
        new_scan.angle_increment = msg.angle_increment
        new_scan.time_increment = msg.time_increment
        new_scan.scan_time = msg.scan_time
        new_scan.range_min = msg.range_min
        new_scan.range_max = msg.range_max

        n_beams = len(msg.ranges)
        # Initialize new ranges array with inf
        new_ranges = np.full(n_beams, np.inf, dtype=np.float32)

        # Precompute constants
        angle_increment = msg.angle_increment
        angle_min = msg.angle_min
        time_increment = msg.time_increment
        if time_increment == 0.0:
            # If scan doesn't provide time_increment, calculate from scan_time
            time_increment = msg.scan_time / float(n_beams)

        # Vectorized calculations for speed
        indices = np.arange(n_beams)
        times = indices * time_increment
        ranges = np.array(msg.ranges, dtype=np.float32)

        # Valid mask
        valid = (ranges >= msg.range_min) & (ranges <= msg.range_max) & (~np.isnan(ranges)) & (~np.isinf(ranges))

        if not np.any(valid):
            self.pub_scan.publish(msg)
            return

        valid_indices = indices[valid]
        valid_times = times[valid]
        valid_ranges = ranges[valid]

        # Calculate nominal angles
        angles = angle_min + valid_indices * angle_increment

        # Calculate robot displacement at each point measurement time
        delta_theta = w * valid_times
        
        # Avoid division by zero
        if abs(w) > 1e-5:
            delta_x = (v / w) * np.sin(delta_theta)
            delta_y = (v / w) * (1.0 - np.cos(delta_theta))
        else:
            delta_x = v * valid_times
            delta_y = np.zeros_like(valid_times)

        # Project point in measurement frame (polar to cartesian)
        px = valid_ranges * np.cos(angles)
        py = valid_ranges * np.sin(angles)

        # Transform points back to the robot starting frame (t=0)
        px_deskewed = delta_x + px * np.cos(delta_theta) - py * np.sin(delta_theta)
        py_deskewed = delta_y + px * np.sin(delta_theta) + py * np.cos(delta_theta)

        # Convert back to polar coordinates
        ranges_deskewed = np.sqrt(px_deskewed**2 + py_deskewed**2)
        angles_deskewed = np.arctan2(py_deskewed, px_deskewed)

        # Map to nearest nominal beam index
        bin_indices = np.round((angles_deskewed - angle_min) / angle_increment).astype(np.int32)

        # Filter indices inside range
        in_range = (bin_indices >= 0) & (bin_indices < n_beams)
        final_bins = bin_indices[in_range]
        final_ranges = ranges_deskewed[in_range]

        # Fill in the new ranges with the minimum distance mapped to each bin (conservative safety approach)
        # Using numpy.minimum.at for vectorized accumulation of minimum ranges
        np.minimum.at(new_ranges, final_bins, final_ranges)

        new_scan.ranges = new_ranges.tolist()
        self.pub_scan.publish(new_scan)

if __name__ == '__main__':
    try:
        node = LidarDeskewNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
