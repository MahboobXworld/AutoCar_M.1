#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Path, OccupancyGrid, Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from autocar_interfaces.msg import BehaviorState
from autocar_interfaces.msg import Situation
import tf.transformations

class SituationClassifierNode:
    """
    ROS Node for Situation Classification.
    Analyzes LiDAR scans, global paths, costmaps, velocity, and pose to classify
    the robot's current navigation context.
    """
    def __init__(self):
        rospy.init_node('situation_classifier_node')

        # Parameters
        self.publish_rate = rospy.get_param('~publish_rate', 10.0)

        # Inputs
        self.laser_scan = None
        self.global_path = None
        self.local_costmap = None
        self.global_costmap = None
        self.odom = None
        self.behavior_state = None

        # Subscribers
        self.sub_scan = rospy.subscribe('/scan', LaserScan, self.scan_callback)
        self.sub_path = rospy.subscribe('/move_base/GlobalPlanner/plan', Path, self.path_callback)
        self.sub_local_costmap = rospy.subscribe('/move_base/local_costmap/costmap', OccupancyGrid, self.local_costmap_callback)
        self.sub_global_costmap = rospy.subscribe('/move_base/global_costmap/costmap', OccupancyGrid, self.global_costmap_callback)
        self.sub_odom = rospy.subscribe('/diff_drive_controller/odom', Odometry, self.odom_callback)
        self.sub_behavior = rospy.subscribe('/behavior_state', BehaviorState, self.behavior_callback)

        # Publisher
        self.pub_situation = rospy.Publisher('/navigation_situation', Situation, queue_size=10)

        rospy.loginfo("[SituationClassifier] Node initialized and listening.")

    def scan_callback(self, msg):
        self.laser_scan = msg

    def path_callback(self, msg):
        self.global_path = msg

    def local_costmap_callback(self, msg):
        self.local_costmap = msg

    def global_costmap_callback(self, msg):
        self.global_costmap = msg

    def odom_callback(self, msg):
        self.odom = msg

    def behavior_callback(self, msg):
        self.behavior_state = msg

    def compute_path_curvature(self):
        """
        Calculates curvature along the global path.
        Returns the average change in heading per unit distance.
        """
        if not self.global_path or len(self.global_path.poses) < 3:
            return 0.0
        
        diffs = []
        poses = self.global_path.poses
        for i in range(1, len(poses) - 1):
            p1 = poses[i-1].pose.position
            p2 = poses[i].pose.position
            p3 = poses[i+1].pose.position
            
            v1 = np.array([p2.x - p1.x, p2.y - p1.y])
            v2 = np.array([p3.x - p2.x, p3.y - p2.y])
            
            norm1 = np.linalg.norm(v1)
            norm2 = np.linalg.norm(v2)
            
            if norm1 > 0.001 and norm2 > 0.001:
                cos_theta = np.dot(v1, v2) / (norm1 * norm2)
                cos_theta = np.clip(cos_theta, -1.0, 1.0)
                angle = np.arccos(cos_theta)
                diffs.append(angle)
                
        if len(diffs) == 0:
            return 0.0
        return float(np.mean(diffs))

    def compute_obstacle_distribution(self):
        """
        Analyzes sector clearances from LiDAR scan to get obstacle density.
        """
        if not self.laser_scan or len(self.laser_scan.ranges) == 0:
            return [0.0] * 4 # front, rear, left, right default clearances
            
        ranges = np.array(self.laser_scan.ranges)
        # Clean invalid values
        ranges = np.where(np.isnan(ranges) | np.isinf(ranges), 15.0, ranges)
        
        num_samples = len(ranges)
        # sector divisions (assuming 360 Lidar)
        front_sector = ranges[int(num_samples*0.45):int(num_samples*0.55)]
        rear_sector = np.concatenate((ranges[:int(num_samples*0.05)], ranges[int(num_samples*0.95):]))
        left_sector = ranges[int(num_samples*0.7):int(num_samples*0.8)]
        right_sector = ranges[int(num_samples*0.2):int(num_samples*0.3)]
        
        return [
            float(np.min(front_sector)) if len(front_sector) > 0 else 10.0,
            float(np.min(rear_sector)) if len(rear_sector) > 0 else 10.0,
            float(np.min(left_sector)) if len(left_sector) > 0 else 10.0,
            float(np.min(right_sector)) if len(right_sector) > 0 else 10.0
        ]

    def compute_costmap_features(self):
        """
        Extracts cost features from the local costmap.
        """
        if not self.local_costmap or len(self.local_costmap.data) == 0:
            return 0.0, 0.0 # max cost, mean cost
            
        data = np.array(self.local_costmap.data)
        return float(np.max(data)), float(np.mean(data))

    def classify(self):
        # Fallback values if behavior state isn't available yet
        dist_to_goal = 999.0
        heading_err = 0.0
        vx = 0.0
        front_c = 10.0
        rear_c = 10.0
        left_c = 10.0
        right_c = 10.0

        if self.behavior_state:
            dist_to_goal = self.behavior_state.distance_to_goal
            heading_err = self.behavior_state.heading_error
            vx = self.behavior_state.current_velocity
            front_c = self.behavior_state.front_clearance
            rear_c = self.behavior_state.rear_clearance
            left_c = self.behavior_state.left_clearance
            right_c = self.behavior_state.right_clearance
        elif self.odom:
            vx = self.odom.twist.twist.linear.x

        # Compute extra features
        lidar_sectors = self.compute_obstacle_distribution()
        if not self.behavior_state:
            front_c, rear_c, left_c, right_c = lidar_sectors

        path_curvature = self.compute_path_curvature()
        max_local_cost, mean_local_cost = self.compute_costmap_features()

        # Classification logic (Decision tree / heuristic)
        situation = "Unknown Situation"
        confidence = 0.5
        
        # Check reverse parking
        if dist_to_goal < 1.8 and abs(heading_err) > 2.0 and vx < 0.05:
            situation = "Reverse Parking"
            confidence = 0.92
        # Check Loading Dock, Pallet Pickup/Drop based on goal proximity and posture
        elif dist_to_goal < 0.6 and abs(heading_err) < 0.15:
            situation = "Tight Goal Alignment"
            confidence = 0.95
        elif dist_to_goal < 1.2:
            # Decide between Pallet Pickup, Pallet Drop, or Loading Dock depending on posture
            # If front/rear clearances are very tight
            if front_c < 0.5 or rear_c < 0.5:
                situation = "Pallet Pickup"
                confidence = 0.85
            else:
                situation = "Loading Dock"
                confidence = 0.88
        # Goal Behind Robot
        elif abs(heading_err) > 2.2:
            situation = "Goal Behind Robot"
            confidence = 0.94
        # Blocked Corridor / Dead End
        elif front_c < 0.8 and left_c < 1.0 and right_c < 1.0:
            situation = "Dead End"
            confidence = 0.90
        elif front_c < 0.8 and (left_c < 1.2 or right_c < 1.2) and max_local_cost > 90:
            situation = "Blocked Corridor"
            confidence = 0.87
        # Dynamic Obstacle Ahead (corridor is clear but front blocked and local cost is high)
        elif front_c < 1.2 and max_local_cost > 85 and vx > 0.01:
            situation = "Dynamic Obstacle Ahead"
            confidence = 0.89
        # Narrow Corridor
        elif left_c < 1.1 and right_c < 1.1 and front_c > 1.5:
            situation = "Narrow Corridor"
            confidence = 0.93
        # Straight Corridor
        elif left_c < 2.0 and right_c < 2.0 and abs(left_c - right_c) < 0.8 and path_curvature < 0.05:
            situation = "Straight Corridor"
            confidence = 0.85
        # Sharp Corners
        elif path_curvature > 0.15:
            if heading_err > 0.2:
                situation = "Sharp Left Corner"
                confidence = 0.88
            else:
                situation = "Sharp Right Corner"
                confidence = 0.88
        # Wide Open Area
        elif front_c > 4.5 and left_c > 4.5 and right_c > 4.5:
            situation = "Wide Open Area"
            confidence = 0.91

        # Populate features array
        features = [
            front_c, rear_c, left_c, right_c,
            dist_to_goal, heading_err, vx,
            path_curvature, max_local_cost, mean_local_cost
        ]
        feature_names = [
            "front_clearance", "rear_clearance", "left_clearance", "right_clearance",
            "distance_to_goal", "heading_error", "linear_velocity",
            "path_curvature", "max_local_cost", "mean_local_cost"
        ]

        # Publish
        msg = Situation()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "base_link"
        msg.situation = situation
        msg.confidence = confidence
        msg.features = features
        msg.feature_names = feature_names
        
        self.pub_situation.publish(msg)

    def run(self):
        rate = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown():
            self.classify()
            rate.sleep()

if __name__ == '__main__':
    try:
        node = SituationClassifierNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
