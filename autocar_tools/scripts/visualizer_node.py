#!/usr/bin/env python3
import rospy
import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import Point
from autocar_interfaces.msg import ManeuverSequence, SemanticWorldModel
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String

class VisualizerNode:
    """
    Visualization Tools for RViz.
    Visualizes motion primitives, predicted trajectories, semantic objects,
    and the current mission timeline using standard interactive Marker/MarkerArray.
    """
    def __init__(self):
        rospy.init_node('visualizer_node')

        # Subscribers
        self.sub_maneuver = rospy.Subscriber('/maneuver_sequence', ManeuverSequence, self.maneuver_callback)
        self.sub_world_model = rospy.Subscriber('/semantic_world_model', SemanticWorldModel, self.world_model_callback)
        self.sub_mission = rospy.Subscriber('/mission_status', String, self.mission_callback)
        self.sub_global_plan = rospy.Subscriber('/move_base/GlobalPlanner/plan', Path, self.global_plan_callback)

        # Publishers
        self.pub_predicted_traj = rospy.Publisher('/visualization/predicted_trajectory', Marker, queue_size=10)
        self.pub_mission_timeline = rospy.Publisher('/visualization/mission_timeline', Marker, queue_size=10)
        self.pub_parking_spot = rospy.Publisher('/parking_spot_marker', Marker, queue_size=1, latch=True)

        # Cache variables
        self.current_mission_status = "Waiting for mission..."

        rospy.loginfo("[Visualizer] RViz Visualizer Node initialized.")

    def global_plan_callback(self, msg):
        if not msg.poses:
            return

        # Get the final pose of the plan (the goal)
        goal_pose = msg.poses[-1]

        marker = Marker()
        marker.header.frame_id = msg.header.frame_id if msg.header.frame_id else "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "parking_spot"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        # Same pose as the last point in the path (the goal)
        marker.pose.position = goal_pose.pose.position
        marker.pose.orientation = goal_pose.pose.orientation

        # Parking slot dimensions (meters)
        marker.scale.x = 1.25      # Length
        marker.scale.y = 0.85      # Width
        marker.scale.z = 0.05      # Thin rectangle

        # Green color
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.6

        # Keep marker forever
        marker.lifetime = rospy.Duration(0)

        self.pub_parking_spot.publish(marker)
        rospy.loginfo("Parking spot marker published (path detected)")

    def mission_callback(self, msg):
        self.current_mission_status = msg.data

    def world_model_callback(self, msg):
        # The semantic world model node already publishes visual markers to /visualization/semantic_objects
        pass

    def maneuver_callback(self, msg):
        """
        Visualizes the predicted robot trajectory of the selected maneuvers.
        """
        if not msg.maneuvers:
            return

        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = "map"
        marker.ns = "predicted_trajectory"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.08  # Line width
        marker.color.r = 0.0
        marker.color.g = 0.8
        marker.color.b = 1.0
        marker.color.a = 0.9

        # Reconstruct path from maneuvers
        pose_x, pose_y = 0.0, 0.0
        # Check first maneuver for starting point
        for m in msg.maneuvers:
            p = Point()
            p.x = m.expected_final_pose.position.x
            p.y = m.expected_final_pose.position.y
            p.z = 0.05
            marker.points.append(p)

        self.pub_predicted_traj.publish(marker)
        self.publish_mission_timeline_marker()

    def publish_mission_timeline_marker(self):
        """
        Publishes a screen text marker in RViz to show the active mission task status.
        """
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = "map"
        marker.ns = "mission_timeline"
        marker.id = 99
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 2.5
        marker.scale.z = 0.4
        marker.color.r = 1.0
        marker.color.g = 0.8
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.text = f"Active Mission Timeline:\n{self.current_mission_status}"
        
        self.pub_mission_timeline.publish(marker)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = VisualizerNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
