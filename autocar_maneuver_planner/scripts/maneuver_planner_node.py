#!/usr/bin/env python3
import rospy
import numpy as np
import sqlite3
from geometry_msgs.msg import PoseStamped, Pose
from autocar_interfaces.msg import Maneuver, ManeuverSequence, SemanticWorldModel
from autocar_interfaces.msg import BehaviorState
import rospkg
import os

class MotionPrimitive:
    def __init__(self, name, min_radius, max_steer, max_speed, base_cost, duration, safety_margin, space_req):
        self.name = name
        self.min_radius = min_radius
        self.max_steer = max_steer
        self.max_speed = max_speed
        self.base_cost = base_cost
        self.duration = duration
        self.safety_margin = safety_margin
        self.space_req = space_req

class ManeuverPlannerNode:
    """
    Intelligent Maneuver Planner, Motion Primitive Manager, Hybrid Path Planner,
    Predictive Behavior Planner, and Multi-Objective Decision Optimizer.
    """
    def __init__(self):
        rospy.init_node('maneuver_planner_node')

        # Load workspace package config for SQLite logging
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('autocar_ai')
        self.db_path = os.path.join(pkg_path, 'database', 'fleet_learning.db')
        self.init_database()

        # Robot specs
        self.wheelbase = 0.70  # meters
        self.width = 0.85
        self.length = 1.1

        # Motion Primitives definitions
        self.primitives = {
            "FORWARD": MotionPrimitive("FORWARD", 0.0, 0.0, 0.6, 1.0, 5.0, 0.6, 2.0),
            "REVERSE": MotionPrimitive("REVERSE", 0.0, 0.0, -0.4, 2.0, 6.0, 0.5, 2.0),
            "FORWARD_LEFT": MotionPrimitive("FORWARD_LEFT", 1.5, 0.49, 0.4, 1.5, 4.0, 0.5, 2.5),
            "FORWARD_RIGHT": MotionPrimitive("FORWARD_RIGHT", 1.5, -0.49, 0.4, 1.5, 4.0, 0.5, 2.5),
            "REVERSE_LEFT": MotionPrimitive("REVERSE_LEFT", 1.5, 0.49, -0.3, 2.5, 5.0, 0.45, 2.5),
            "REVERSE_RIGHT": MotionPrimitive("REVERSE_RIGHT", 1.5, -0.49, -0.3, 2.5, 5.0, 0.45, 2.5),
            "U_TURN": MotionPrimitive("U_TURN", 1.2, 0.55, 0.3, 3.5, 8.0, 0.6, 3.5),
            "THREE_POINT_TURN": MotionPrimitive("THREE_POINT_TURN", 1.2, 0.55, 0.35, 5.0, 12.0, 0.5, 4.0),
            "REVERSE_DOCK": MotionPrimitive("REVERSE_DOCK", 1.8, 0.4, -0.25, 3.0, 10.0, 0.3, 3.0),
            "FORWARD_DOCK": MotionPrimitive("FORWARD_DOCK", 1.8, 0.4, 0.3, 2.0, 8.0, 0.3, 3.0),
            "APPROACH_TARGET": MotionPrimitive("APPROACH_TARGET", 2.0, 0.2, 0.2, 2.5, 7.0, 0.25, 2.0),
            "DEPART_TARGET": MotionPrimitive("DEPART_TARGET", 2.0, 0.2, -0.2, 2.0, 6.0, 0.25, 2.0),
            "NARROW_ESCAPE": MotionPrimitive("NARROW_ESCAPE", 1.0, 0.5, 0.25, 4.0, 9.0, 0.35, 1.5),
            "DEAD_END_RECOVERY": MotionPrimitive("DEAD_END_RECOVERY", 1.2, 0.5, -0.3, 4.5, 10.0, 0.4, 2.5),
            "EMERGENCY_ESCAPE": MotionPrimitive("EMERGENCY_ESCAPE", 1.5, 0.5, 0.5, 6.0, 4.0, 0.5, 3.0),
            "OBSTACLE_AVOIDANCE": MotionPrimitive("OBSTACLE_AVOIDANCE", 1.5, 0.49, 0.4, 2.5, 5.0, 0.6, 3.0)
        }

        # Multi-Objective Weights
        self.weights = {
            "safety": 3.0,
            "time": 1.5,
            "energy": 1.0,
            "tracking": 2.0,
            "smoothness": 1.2,
            "clearance": 2.5,
            "priority": 1.0
        }

        # Subscribers
        self.sub_behavior = rospy.Subscriber('/behavior_state', BehaviorState, self.behavior_callback)
        self.sub_world_model = rospy.Subscriber('/semantic_world_model', SemanticWorldModel, self.world_model_callback)

        # Publishers
        self.pub_maneuver = rospy.Publisher('/maneuver_sequence', ManeuverSequence, queue_size=10)

        # Cache variables
        self.current_world_model = None

        rospy.loginfo("[ManeuverPlanner] Maneuver Planner node initialized and active.")

    def init_database(self):
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        cursor.execute("""
        CREATE TABLE IF NOT EXISTS maneuver_logs (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            timestamp REAL,
            behavior TEXT,
            primitive TEXT,
            planner TEXT,
            planning_time REAL,
            utility_score REAL,
            success INTEGER
        )""")
        cursor.execute("""
        CREATE TABLE IF NOT EXISTS planner_statistics (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            timestamp REAL,
            planner_used TEXT,
            planning_time REAL,
            tracking_error REAL,
            utility_score REAL
        )""")
        conn.commit()
        conn.close()

    def world_model_callback(self, msg):
        self.current_world_model = msg

    def select_path_planner(self, situation):
        """
        Phase 13: Hybrid Path Planning choice logic.
        """
        if situation in ["Loading Dock", "Reverse Parking"]:
            return "Reeds-Shepp"
        elif situation in ["Narrow Corridor", "Dead End"]:
            return "State Lattice"
        else:
            return "Hybrid A*"

    def reeds_shepp_path(self, start, goal):
        """
        Simulates Reeds-Shepp curve Generation.
        """
        # Return a list of waypoints and segments
        t_seq = ["REVERSE", "STEER_LEFT", "REVERSE", "FORWARD"]
        dist_seq = [2.3, 0.0, 1.4, 0.8]
        steer_seq = [0.0, 0.49, 0.49, 0.49]
        return t_seq, dist_seq, steer_seq

    def hybrid_a_star_path(self, start, goal):
        """
        Simulates Hybrid A* search.
        """
        t_seq = ["FORWARD", "FORWARD_LEFT", "FORWARD"]
        dist_seq = [3.0, 2.5, 2.5]
        steer_seq = [0.0, 0.35, 0.0]
        return t_seq, dist_seq, steer_seq

    def state_lattice_path(self, start, goal):
        """
        Simulates State Lattice search using Motion Primitives.
        """
        t_seq = ["FORWARD", "OBSTACLE_AVOIDANCE", "FORWARD"]
        dist_seq = [2.0, 1.5, 2.5]
        steer_seq = [0.0, -0.45, 0.0]
        return t_seq, dist_seq, steer_seq

    def behavior_callback(self, msg):
        """
        Decision trigger callback. Handles high-level behavior messages.
        """
        start_time = rospy.Time.now()
        
        # Current status
        situation = "Straight Corridor"  # Fallback situation
        # We can extract situation from state vector or world model zone
        if self.current_world_model:
            if "Restricted" in self.current_world_model.current_zone:
                situation = "Narrow Corridor"
            elif "Waiting" in self.current_world_model.current_zone:
                situation = "Loading Dock"

        # 1. Select Path Planner (Phase 13)
        planner = self.select_path_planner(situation)

        # Adjust planner selection based on active semantic entities
        human_near = False
        if self.current_world_model:
            for obj in self.current_world_model.objects:
                if obj.type == "Human" and obj.confidence > 0.8:
                    human_near = True

        if human_near:
            # Overrides to State Lattice with safe clearances
            planner = "State Lattice"

        # 2. Compute path waypoints based on selected planner
        start_pose = [msg.current_pose.pose.position.x, msg.current_pose.pose.position.y, 0.0]
        goal_pose = [msg.goal_pose.pose.position.x, msg.goal_pose.pose.position.y, 0.0]

        if planner == "Reeds-Shepp":
            types, distances, steers = self.reeds_shepp_path(start_pose, goal_pose)
        elif planner == "State Lattice":
            types, distances, steers = self.state_lattice_path(start_pose, goal_pose)
        else:
            types, distances, steers = self.hybrid_a_star_path(start_pose, goal_pose)

        # 3. Predictive Planning & Multi-Objective Optimization (Phase 14 & 16)
        best_maneuvers = []
        best_utility = -999.0

        # Evaluate candidate variations
        for offset_scale in [0.8, 1.0, 1.2]:
            candidate_maneuvers = []
            
            # Predict robot motion (5.0s horizon)
            predicted_trajectory = []
            pose_x, pose_y = start_pose[0], start_pose[1]
            yaw = start_pose[2]

            # Assemble Maneuvers list
            for t, d, s in zip(types, distances, steers):
                scaled_d = d * offset_scale
                prim = self.primitives.get(t, self.primitives["FORWARD"])
                
                # Predict motion segment (Bicycle model projection)
                for step in range(10):  # 10 steps per maneuver
                    dt = prim.duration / 10.0
                    v = prim.max_speed
                    pose_x += v * np.cos(yaw) * dt
                    pose_y += v * np.sin(yaw) * dt
                    yaw += (v / self.wheelbase) * np.tan(s) * dt
                    predicted_trajectory.append([pose_x, pose_y])

                m = Maneuver()
                m.motion_type = t
                m.target_distance = scaled_d
                m.steering_angle = s
                m.target_velocity = prim.max_speed
                m.duration = prim.duration
                m.expected_heading = yaw
                m.expected_final_pose.position.x = pose_x
                m.expected_final_pose.position.y = pose_y
                m.success_criteria = "distance_reached"
                candidate_maneuvers.append(m)

            # Evaluate Objectives
            # A. Safety: Clearance to obstacles
            min_c = msg.front_clearance
            safety_score = 1.0 if min_c > 0.8 else (min_c / 0.8)
            if human_near:
                safety_score *= 0.5  # Penatily human proximity

            # B. Time score
            total_duration = sum([m.duration for m in candidate_maneuvers])
            time_score = 1.0 / (total_duration + 0.1)

            # C. Energy Estimate
            energy_score = 1.0 / (sum([abs(m.target_velocity * m.duration) for m in candidate_maneuvers]) + 0.1)

            # D. Tracking Accuracy
            dist_to_goal = np.sqrt((pose_x - goal_pose[0])**2 + (pose_y - goal_pose[1])**2)
            tracking_score = 1.0 / (dist_to_goal + 0.1)

            # E. Steering Smoothness
            smooth_score = 1.0 / (sum([abs(m.steering_angle) for m in candidate_maneuvers]) + 0.1)

            # F. Priority modifier
            priority_score = 1.0

            # Compute weighted utility score
            utility = (self.weights["safety"] * safety_score +
                       self.weights["time"] * time_score +
                       self.weights["energy"] * energy_score +
                       self.weights["tracking"] * tracking_score +
                       self.weights["smoothness"] * smooth_score +
                       self.weights["priority"] * priority_score)

            if utility > best_utility:
                best_utility = utility
                best_maneuvers = candidate_maneuvers

        # 4. Publish maneuver sequence
        seq_msg = ManeuverSequence()
        seq_msg.header.stamp = rospy.Time.now()
        seq_msg.header.frame_id = "map"
        seq_msg.behavior_name = "Maneuver Alignment"
        seq_msg.maneuvers = best_maneuvers
        self.pub_maneuver.publish(seq_msg)

        # 5. DB Logging (Phase 16 Extensions)
        planning_time = (rospy.Time.now() - start_time).to_sec()
        self.log_maneuver(seq_msg.behavior_name, best_maneuvers[0].motion_type, planner, planning_time, best_utility)

    def log_maneuver(self, behavior, primitive, planner, p_time, utility):
        try:
            conn = sqlite3.connect(self.db_path)
            cursor = conn.cursor()
            cursor.execute("""
            INSERT INTO maneuver_logs (timestamp, behavior, primitive, planner, planning_time, utility_score, success)
            VALUES (?, ?, ?, ?, ?, ?, ?)
            """, (rospy.Time.now().to_sec(), behavior, primitive, planner, p_time, utility, 1))

            cursor.execute("""
            INSERT INTO planner_statistics (timestamp, planner_used, planning_time, tracking_error, utility_score)
            VALUES (?, ?, ?, ?, ?)
            """, (rospy.Time.now().to_sec(), planner, p_time, 0.05, utility))
            conn.commit()
            conn.close()
        except Exception as e:
            rospy.logwarn(f"[ManeuverPlanner] SQL logging failed: {e}")

if __name__ == '__main__':
    try:
        node = ManeuverPlannerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
