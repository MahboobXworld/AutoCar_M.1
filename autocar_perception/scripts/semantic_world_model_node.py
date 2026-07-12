#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Pose, Vector3
from autocar_interfaces.msg import SemanticObject, SemanticWorldModel
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import LaserScan
import numpy as np

class SemanticWorldModelNode:
    """
    Semantic World Model Node.
    Aggregates spatial readings and classifies them into structured objects (Shelf, Pallet, Charger, Humans, etc.)
    and publishes the current zone type.
    """
    def __init__(self):
        rospy.init_node('semantic_world_model_node')

        # Publishers
        self.pub_world_model = rospy.Publisher('/semantic_world_model', SemanticWorldModel, queue_size=10)
        self.pub_markers = rospy.Publisher('/visualization/semantic_objects', MarkerArray, queue_size=10)

        # Subscribers
        self.sub_scan = rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        # Pre-defined semantic map elements (representing typical warehouse layout)
        self.static_objects = [
            {"id": "shelf_1", "type": "Shelf", "pose": [2.0, 4.0, 0.0], "dims": [2.0, 0.8, 1.8], "conf": 0.98},
            {"id": "shelf_2", "type": "Shelf", "pose": [-2.0, 4.0, 0.0], "dims": [2.0, 0.8, 1.8], "conf": 0.98},
            {"id": "charging_station_1", "type": "Charging Station", "pose": [-4.0, -1.0, 0.0], "dims": [0.6, 0.6, 1.2], "conf": 0.95},
            {"id": "loading_dock_4", "type": "Loading Dock", "pose": [5.0, 0.0, -1.57], "dims": [1.5, 1.5, 0.1], "conf": 0.99},
            {"id": "pallet_pickup_1", "type": "Pallet", "pose": [1.0, 2.0, 3.14], "dims": [1.2, 0.8, 0.4], "conf": 0.90}
        ]

        # Dynamic obstacles found via sensor feedback
        self.dynamic_objects = []
        self.current_zone = "Safety Zone"

        rospy.loginfo("[WorldModel] Semantic World Model node is running.")

    def scan_callback(self, msg):
        """
        Processes lidar raw scanner streams to identify humans or moving forklifts
        based on cluster shapes.
        """
        if not msg.ranges:
            return
            
        ranges = np.array(msg.ranges)
        ranges = np.where(np.isnan(ranges) | np.isinf(ranges), 30.0, ranges)
        
        # Simple clustering based on differences
        diffs = np.abs(np.diff(ranges))
        split_indices = np.where(diffs > 0.4)[0] + 1
        clusters = np.split(ranges, split_indices)

        self.dynamic_objects = []
        human_detected = False
        forklift_detected = False

        # Classify dynamic clusters
        for idx, cluster in enumerate(clusters):
            if len(cluster) < 3 or len(cluster) > 30:
                continue
            
            mean_dist = np.mean(cluster)
            if mean_dist < 4.0:
                cluster_width = mean_dist * len(cluster) * msg.angle_increment
                
                # Pedestrian classification: narrow cluster
                if 0.15 < cluster_width < 0.45:
                    human_detected = True
                    self.dynamic_objects.append({
                        "id": f"human_{idx}",
                        "type": "Human",
                        "pose": [float(mean_dist * 0.8), float(mean_dist * 0.2), 0.0],
                        "dims": [0.4, 0.4, 1.7],
                        "conf": 0.85
                    })
                # Forklift classification: wider cluster
                elif 0.8 < cluster_width < 1.8:
                    forklift_detected = True
                    self.dynamic_objects.append({
                        "id": f"forklift_{idx}",
                        "type": "Forklift",
                        "pose": [float(mean_dist * 0.7), float(mean_dist * 0.1), 0.0],
                        "dims": [1.5, 0.9, 1.6],
                        "conf": 0.80
                    })

        # Update active zone
        if human_detected:
            self.current_zone = "Restricted Area (Muted Speed)"
        elif forklift_detected:
            self.current_zone = "Waiting Zone"
        else:
            self.current_zone = "Safety Zone"

    def publish_model(self):
        msg = SemanticWorldModel()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        msg.current_zone = self.current_zone

        all_objects = self.static_objects + self.dynamic_objects
        for obj in all_objects:
            so = SemanticObject()
            so.id = obj["id"]
            so.type = obj["type"]
            so.pose.position.x = obj["pose"][0]
            so.pose.position.y = obj["pose"][1]
            so.pose.position.z = obj["pose"][2]
            
            # Simple rotation conversion
            qz = float(np.sin(obj["pose"][2] / 2.0))
            qw = float(np.cos(obj["pose"][2] / 2.0))
            so.pose.orientation.z = qz
            so.pose.orientation.w = qw
            
            so.dimensions.x = obj["dims"][0]
            so.dimensions.y = obj["dims"][1]
            so.dimensions.z = obj["dims"][2]
            so.confidence = obj["conf"]
            msg.objects.append(so)

        self.pub_world_model.publish(msg)
        self.publish_rviz_markers(all_objects)

    def publish_rviz_markers(self, objects):
        marker_array = MarkerArray()
        for idx, obj in enumerate(objects):
            marker = Marker()
            marker.header.stamp = rospy.Time.now()
            marker.header.frame_id = "map"
            marker.ns = "semantic_objects"
            marker.id = idx
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            
            marker.pose.position.x = obj["pose"][0]
            marker.pose.position.y = obj["pose"][1]
            marker.pose.position.z = obj["dims"][2] / 2.0
            marker.pose.orientation.z = float(np.sin(obj["pose"][2] / 2.0))
            marker.pose.orientation.w = float(np.cos(obj["pose"][2] / 2.0))
            
            marker.scale.x = obj["dims"][0]
            marker.scale.y = obj["dims"][1]
            marker.scale.z = obj["dims"][2]

            # Custom colors by classification type
            if obj["type"] == "Shelf":
                marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.5, 0.5, 0.5, 0.7
            elif obj["type"] == "Human":
                marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, 0.0, 0.0, 0.8
            elif obj["type"] == "Forklift":
                marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, 0.5, 0.0, 0.8
            elif obj["type"] == "Pallet":
                marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.6, 0.4, 0.2, 0.8
            else: # Charging station / docks
                marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.0, 1.0, 0.0, 0.7

            marker_array.markers.append(marker)

            # Label marker
            text_marker = Marker()
            text_marker.header.stamp = rospy.Time.now()
            text_marker.header.frame_id = "map"
            text_marker.ns = "semantic_labels"
            text_marker.id = idx + 1000
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = obj["pose"][0]
            text_marker.pose.position.y = obj["pose"][1]
            text_marker.pose.position.z = obj["dims"][2] + 0.3
            text_marker.scale.z = 0.35
            text_marker.color.r, text_marker.color.g, text_marker.color.b, text_marker.color.a = 1.0, 1.0, 1.0, 1.0
            text_marker.text = f"{obj['type']} [{obj['id']}]"
            marker_array.markers.append(text_marker)

        self.pub_markers.publish(marker_array)

    def run(self):
        rate = rospy.Rate(5.0) # 5Hz updates
        while not rospy.is_shutdown():
            self.publish_model()
            rate.sleep()

if __name__ == '__main__':
    try:
        node = SemanticWorldModelNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
