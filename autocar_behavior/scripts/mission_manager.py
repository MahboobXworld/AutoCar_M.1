#!/usr/bin/env python3
import rospy
import actionlib
import sqlite3
import rospkg
import os
import math
import numpy as np
import time
import uuid
import json
import tf.transformations
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Pose, Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path
from actionlib_msgs.msg import GoalStatusArray, GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionFeedback, MoveBaseActionResult
from std_srvs.srv import Empty
from visualization_msgs.msg import Marker, MarkerArray

# Main execution states
STATE_IDLE = "IDLE"
STATE_SEND_GOAL = "SEND_GOAL"
STATE_WAIT_FOR_ACCEPT = "WAIT_FOR_ACCEPT"
STATE_NAVIGATING = "NAVIGATING"
STATE_APPROACHING_GOAL = "APPROACHING_GOAL"
STATE_PRECISION_PARKING = "PRECISION_PARKING"
STATE_VERIFY_FINAL_POSE = "VERIFY_FINAL_POSE"
STATE_TASK_COMPLETE = "TASK_COMPLETE"
STATE_NEXT_TASK = "NEXT_TASK"

# Failure / Recovery states
STATE_GOAL_REJECTED = "GOAL_REJECTED"
STATE_PLANNER_FAILED = "PLANNER_FAILED"
STATE_LOCALIZATION_FAILED = "LOCALIZATION_FAILED"
STATE_OBSTACLE_BLOCKED = "OBSTACLE_BLOCKED"
STATE_RECOVERY = "RECOVERY"
STATE_REPLAN = "REPLAN"
STATE_MISSION_ABORT = "MISSION_ABORT"

class MissionManagerNode:
    """
    Production-Grade Hierarchical Mission Execution System for Autonomous Vehicles.
    Integrates with move_base, localization, and safety supervisor.
    """
    def __init__(self):
        rospy.init_node('mission_manager')

        # Load database path
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('autocar_ai')
        self.db_path = os.path.join(pkg_path, 'database', 'fleet_learning.db')
        self.init_database()

        # Generate a unique Mission ID
        self.mission_id = "mission_" + str(uuid.uuid4())[:8]
        rospy.loginfo(f"[MissionManager] Initializing new mission: {self.mission_id}")

        # Configurable thresholds
        self.max_retries = 3

        # Task List Definition with real warehouse coordinates
        self.tasks = [
            {"id": 1, "name": "Navigate to Waypoint 1", "type": "NAV", "x": 4.5, "y": 1.1, "yaw": 0.0},
            {"id": 2, "name": "Approach Target", "type": "NAV", "x": 6.0, "y": 1.0, "yaw": -0.5},
            {"id": 3, "name": "Perform Action at Target", "type": "ACTION", "action_duration": 5.0},
            {"id": 4, "name": "Navigate to Waypoint 2", "type": "NAV", "x": -2.0, "y": 5.5, "yaw": 1.57},
            {"id": 5, "name": "Avoid Obstacles", "type": "NAV", "x": -5.0, "y": 0.0, "yaw": 3.14},
            {"id": 6, "name": "Precise Docking", "type": "NAV", "x": -6.17, "y": 2.18, "yaw": 2.66},
            {"id": 7, "name": "Perform Action at Destination", "type": "ACTION", "action_duration": 5.0},
            {"id": 8, "name": "Return to Base", "type": "NAV", "x": 0.0, "y": 0.0, "yaw": 0.0}
        ]

        # State Variables
        self.state = STATE_IDLE
        self.current_task_idx = 0
        self.active_task = None
        self.last_safety_recovery_state = "NORMAL"
        self.startup_time = rospy.Time.now().to_sec()
        
        self.auto_start = rospy.get_param('~auto_start', False)
        self.mission_started = self.auto_start

        # Robot Pose & State Variables
        self.robot_x = None
        self.robot_y = None
        self.robot_yaw = None
        self.robot_v = 0.0
        self.robot_w = 0.0
        
        self.latest_odom = None
        self.latest_localization_health = "Excellent"
        self.latest_goal_hold_state = "INIT"
        self.latest_cmd_vel = None
        
        self.amcl_cov_trace = 0.0

        # Task Metrics Tracking
        self.task_start_time = 0.0
        self.task_end_time = 0.0
        self.goal_sent_time = 0.0
        self.planning_start_time = None
        self.planning_duration = 0.0
        self.navigation_start_time = None
        self.navigation_duration = 0.0
        
        self.task_retries = 0
        self.task_recovery_count = 0
        self.task_replan_count = 0
        self.task_distance_travelled = 0.0
        self.task_success = False
        self.task_failure_reason = ""
        
        self.last_odom_x = None
        self.last_odom_y = None

        # Setup publishers
        self.pub_mission_status = rospy.Publisher('/mission_status', String, queue_size=5, latch=True)
        self.pub_current_task = rospy.Publisher('/current_task', String, queue_size=5, latch=True)
        self.pub_task_progress = rospy.Publisher('/task_progress', String, queue_size=5, latch=True)
        self.pub_mission_events = rospy.Publisher('/mission_events', String, queue_size=10)
        self.pub_markers = rospy.Publisher('/visualization/mission_markers', MarkerArray, queue_size=5, latch=True)
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        # Setup ActionClient
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("[MissionManager] Waiting for move_base action server...")
        self.move_base_client.wait_for_server()
        rospy.loginfo("[MissionManager] Connected to move_base action server.")

        # Setup subscribers
        self.sub_mb_status = rospy.Subscriber('/move_base/status', GoalStatusArray, self.move_base_status_callback)
        self.sub_mb_feedback = rospy.Subscriber('/move_base/feedback', MoveBaseActionFeedback, self.move_base_feedback_callback)
        self.sub_mb_result = rospy.Subscriber('/move_base/result', MoveBaseActionResult, self.move_base_result_callback)
        self.sub_odom = rospy.Subscriber('/odometry/filtered', Odometry, self.odom_callback)
        self.sub_amcl = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_callback)
        self.sub_cmd_vel = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        self.sub_mission_status = rospy.Subscriber('/mission_status', String, self.mission_status_callback)
        self.sub_loc_health = rospy.Subscriber('/localization_health', String, self.localization_health_callback)
        self.sub_hold_state = rospy.Subscriber('/goal_hold_state', String, self.goal_hold_state_callback)
        self.sub_start_mission = rospy.Subscriber('/start_mission', String, self.start_mission_callback)
        
        # Subscribe to safety diagnostics to count recoveries/replans
        self.sub_safety_diag = rospy.Subscriber('/safety_diagnostics', String, self.safety_diagnostics_callback)
        # Subscribe to global planner plan to count replans
        self.sub_global_plan = rospy.Subscriber('/move_base/GlobalPlanner/plan', Path, self.global_plan_callback)

        rospy.loginfo("[MissionManager] Redesigned Hierarchical Mission Executive fully initialized.")

    def init_database(self):
        try:
            conn = sqlite3.connect(self.db_path)
            cursor = conn.cursor()
            cursor.execute("""
            CREATE TABLE IF NOT EXISTS mission_executions (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                mission_id TEXT,
                task_id INTEGER,
                goal_pose TEXT,
                start_time REAL,
                finish_time REAL,
                navigation_duration REAL,
                success INTEGER,
                failure_reason TEXT,
                planner_used TEXT,
                recovery_count INTEGER,
                replan_count INTEGER,
                distance_travelled REAL,
                goal_position_error REAL,
                goal_yaw_error REAL,
                average_speed REAL
            )""")
            conn.commit()
            conn.close()
        except Exception as e:
            rospy.logerr(f"[MissionManager] Failed to init sqlite database: {e}")

    def log_task_to_sqlite(self):
        try:
            conn = sqlite3.connect(self.db_path)
            cursor = conn.cursor()
            
            # Compute errors
            pos_err, yaw_err = self.get_pose_errors()
            if pos_err is None: pos_err = 0.0
            if yaw_err is None: yaw_err = 0.0
            
            # Compute average speed
            avg_speed = 0.0
            if self.navigation_duration > 0.0:
                avg_speed = self.task_distance_travelled / self.navigation_duration
            
            # Get current planner parameter
            planner_used = rospy.get_param("/move_base/base_local_planner", "unknown")
            if "Teb" in planner_used or "teb" in planner_used:
                planner_used_str = "teb"
            elif "DWA" in planner_used or "dwa" in planner_used:
                planner_used_str = "dwa"
            else:
                planner_used_str = planner_used

            cursor.execute("""
            INSERT INTO mission_executions (
                mission_id, task_id, goal_pose, start_time, finish_time, 
                navigation_duration, success, failure_reason, planner_used, 
                recovery_count, replan_count, distance_travelled, 
                goal_position_error, goal_yaw_error, average_speed
            ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
            """, (
                self.mission_id,
                self.active_task["id"],
                f"{self.active_task.get('x', 0.0):.2f}, {self.active_task.get('y', 0.0):.2f}, {self.active_task.get('yaw', 0.0):.2f}" if self.active_task["type"] == "NAV" else "ACTION",
                self.task_start_time,
                self.task_end_time,
                self.navigation_duration,
                1 if self.task_success else 0,
                self.task_failure_reason,
                planner_used_str,
                self.task_recovery_count,
                self.task_replan_count,
                self.task_distance_travelled,
                pos_err,
                yaw_err,
                avg_speed
            ))
            conn.commit()
            conn.close()
            rospy.loginfo(f"[MissionManager] Logged metrics to DB for task: {self.active_task['name']}")
        except Exception as e:
            rospy.logwarn(f"[MissionManager] SQLite database logging failed: {e}")

    # ROS Callbacks
    def odom_callback(self, msg):
        self.latest_odom = msg
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.robot_yaw = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]

        # Accumulate distance while navigating
        if self.state in [STATE_NAVIGATING, STATE_APPROACHING_GOAL, STATE_PRECISION_PARKING]:
            if self.last_odom_x is not None and self.last_odom_y is not None:
                dx = self.robot_x - self.last_odom_x
                dy = self.robot_y - self.last_odom_y
                dist = math.hypot(dx, dy)
                self.task_distance_travelled += dist
            self.last_odom_x = self.robot_x
            self.last_odom_y = self.robot_y

    def amcl_callback(self, msg):
        cov = msg.pose.covariance
        self.amcl_cov_trace = cov[0] + cov[7] + cov[35]

    def cmd_vel_callback(self, msg):
        self.latest_cmd_vel = msg
        self.robot_v = msg.linear.x
        self.robot_w = msg.angular.z

    def mission_status_callback(self, msg):
        pass

    def start_mission_callback(self, msg):
        rospy.loginfo(f"[MissionManager] Start mission command received: {msg.data}")
        self.mission_started = True

    def localization_health_callback(self, msg):
        self.latest_localization_health = msg.data

    def goal_hold_state_callback(self, msg):
        self.latest_goal_hold_state = msg.data

    def move_base_status_callback(self, msg):
        pass

    def move_base_feedback_callback(self, msg):
        pass

    def move_base_result_callback(self, msg):
        pass

    def global_plan_callback(self, msg):
        if self.state in [STATE_NAVIGATING, STATE_APPROACHING_GOAL, STATE_PRECISION_PARKING]:
            self.task_replan_count += 1
            rospy.loginfo(f"[MissionManager] Global Replanned. Count: {self.task_replan_count}")

    def safety_diagnostics_callback(self, msg):
        try:
            data = json.loads(msg.data)
            rec_state = data.get("recovery_state", "NORMAL")
            if rec_state != "NORMAL" and self.last_safety_recovery_state == "NORMAL":
                self.task_recovery_count += 1
                rospy.loginfo(f"[MissionManager] Safety recovery triggered. Count: {self.task_recovery_count}")
            self.last_safety_recovery_state = rec_state
        except:
            pass

    # Helper operations
    def clear_costmaps(self):
        try:
            rospy.wait_for_service('/move_base/clear_costmaps', timeout=1.0)
            srv = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
            srv()
            rospy.loginfo("[MissionManager] Cleared move_base costmaps.")
        except Exception as e:
            rospy.logwarn(f"[MissionManager] Clear costmaps service call failed: {e}")

    def stop_robot(self):
        cmd = Twist()
        self.pub_cmd_vel.publish(cmd)

    def get_distance_to_goal(self):
        if self.robot_x is None or self.robot_y is None or self.active_task is None:
            return None
        if self.active_task["type"] != "NAV":
            return 0.0
        dx = self.active_task["x"] - self.robot_x
        dy = self.active_task["y"] - self.robot_y
        return math.hypot(dx, dy)

    def get_pose_errors(self):
        if self.robot_x is None or self.robot_y is None or self.robot_yaw is None or self.active_task is None:
            return None, None
        if self.active_task["type"] != "NAV":
            return 0.0, 0.0
        
        pos_err = self.get_distance_to_goal()
        target_yaw = self.active_task["yaw"]
        yaw_err = abs(self.normalize_angle(target_yaw - self.robot_yaw))
        return pos_err, yaw_err

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def is_robot_stopped(self):
        if self.latest_odom is None:
            return False
        v = self.latest_odom.twist.twist.linear.x
        w = self.latest_odom.twist.twist.angular.z
        return abs(v) < 0.05 and abs(w) < 0.05

    def is_goal_stop_manager_parked(self):
        return self.latest_goal_hold_state == "HOLDING"

    def is_localization_confident(self):
        if self.latest_localization_health in ["Excellent", "Good"]:
            return True
        if self.amcl_cov_trace < 1.5:
            return True
        return False

    def publish_status(self, status):
        rospy.loginfo(f"[MissionManager] Status: {status}")
        self.pub_mission_status.publish(String(data=status))

    def publish_event(self, event):
        self.pub_mission_events.publish(String(data=f"[{self.mission_id}] {event}"))

    # State Machine Update Loop
    def update_state_machine(self):
        if self.state == STATE_IDLE:
            if not self.mission_started:
                self.publish_status("Waiting for start command on /start_mission...")
                return

            # Wait for startup stabilization (e.g., 8 seconds) and localization to become confident
            time_since_start = rospy.Time.now().to_sec() - self.startup_time
            if time_since_start < 8.0:
                self.publish_status(f"Waiting for system stabilization... ({8.0 - time_since_start:.1f}s remaining)")
                return
            if not self.is_localization_confident():
                self.publish_status("Waiting for localization confidence to stabilize...")
                return

            if self.current_task_idx < len(self.tasks):
                self.active_task = self.tasks[self.current_task_idx]
                self.task_start_time = rospy.Time.now().to_sec()
                self.task_distance_travelled = 0.0
                self.last_odom_x = self.robot_x
                self.last_odom_y = self.robot_y
                self.task_retries = 0
                self.task_recovery_count = 0
                self.task_replan_count = 0
                self.planning_start_time = None
                self.planning_duration = 0.0
                self.navigation_start_time = None
                self.navigation_duration = 0.0
                self.task_success = False
                self.task_failure_reason = ""
                
                self.pub_current_task.publish(String(data=self.active_task["name"]))
                self.pub_task_progress.publish(String(data=f"{self.current_task_idx + 1}/{len(self.tasks)}"))
                
                self.publish_status(f"Executing Task {self.current_task_idx + 1}/{len(self.tasks)}: {self.active_task['name']}")
                self.publish_event(f"TASK_START: {self.active_task['name']}")
                
                if self.active_task["type"] == "NAV":
                    self.state = STATE_SEND_GOAL
                elif self.active_task["type"] == "ACTION":
                    self.action_start_time = rospy.Time.now().to_sec()
                    self.state = STATE_VERIFY_FINAL_POSE
            else:
                self.publish_status("Mission Complete: All tasks successfully finished.")
                self.publish_event("MISSION_COMPLETE")
                rospy.signal_shutdown("Mission execution completed successfully.")
                
        elif self.state == STATE_SEND_GOAL:
            mb_goal = MoveBaseGoal()
            mb_goal.target_pose.header.frame_id = "map"
            mb_goal.target_pose.header.stamp = rospy.Time.now()
            mb_goal.target_pose.pose.position.x = self.active_task["x"]
            mb_goal.target_pose.pose.position.y = self.active_task["y"]
            
            q_z = math.sin(self.active_task["yaw"] / 2.0)
            q_w = math.cos(self.active_task["yaw"] / 2.0)
            mb_goal.target_pose.pose.orientation.z = q_z
            mb_goal.target_pose.pose.orientation.w = q_w
            
            self.move_base_client.send_goal(mb_goal)
            self.goal_sent_time = rospy.Time.now().to_sec()
            self.planning_start_time = self.goal_sent_time
            self.state = STATE_WAIT_FOR_ACCEPT
            self.publish_status(f"Goal sent to move_base. Waiting for acceptance...")

        elif self.state == STATE_WAIT_FOR_ACCEPT:
            client_state = self.move_base_client.get_state()
            
            if client_state == GoalStatus.ACTIVE:
                self.planning_duration = rospy.Time.now().to_sec() - self.planning_start_time
                self.navigation_start_time = rospy.Time.now().to_sec()
                self.state = STATE_NAVIGATING
                self.publish_status(f"Goal accepted. Navigating to {self.active_task['name']}.")
            elif client_state in [GoalStatus.REJECTED, GoalStatus.ABORTED, GoalStatus.LOST]:
                self.task_failure_reason = f"Goal rejected/aborted by move_base (state: {client_state})"
                self.state = STATE_GOAL_REJECTED
            else:
                # Accept timeout check (10s limit)
                if rospy.Time.now().to_sec() - self.goal_sent_time > 10.0:
                    self.task_failure_reason = "Timeout waiting for goal acceptance"
                    self.state = STATE_GOAL_REJECTED

        elif self.state == STATE_NAVIGATING:
            dist = self.get_distance_to_goal()
            if dist is not None:
                if dist < 1.5:
                    self.state = STATE_APPROACHING_GOAL
                    self.publish_status(f"Approaching goal ({dist:.2f}m remaining).")
                    
            client_state = self.move_base_client.get_state()
            if client_state in [GoalStatus.ABORTED, GoalStatus.LOST, GoalStatus.REJECTED]:
                self.task_failure_reason = "Planner aborted goal navigation"
                self.state = STATE_PLANNER_FAILED
            elif client_state == GoalStatus.SUCCEEDED:
                self.state = STATE_VERIFY_FINAL_POSE

        elif self.state == STATE_APPROACHING_GOAL:
            dist = self.get_distance_to_goal()
            if dist is not None:
                if dist < 0.35:
                    self.state = STATE_PRECISION_PARKING
                    self.publish_status(f"Precision parking phase ({dist:.2f}m remaining).")
                    
            client_state = self.move_base_client.get_state()
            if client_state in [GoalStatus.ABORTED, GoalStatus.LOST, GoalStatus.REJECTED]:
                self.task_failure_reason = "Planner aborted during approach"
                self.state = STATE_PLANNER_FAILED
            elif client_state == GoalStatus.SUCCEEDED:
                self.state = STATE_VERIFY_FINAL_POSE

        elif self.state == STATE_PRECISION_PARKING:
            client_state = self.move_base_client.get_state()
            parked = self.is_goal_stop_manager_parked()
            
            if client_state == GoalStatus.SUCCEEDED or parked:
                self.state = STATE_VERIFY_FINAL_POSE
                self.publish_status("At goal target. Beginning validation checks...")
            elif client_state in [GoalStatus.ABORTED, GoalStatus.LOST, GoalStatus.REJECTED]:
                self.task_failure_reason = "Planner failed during precision parking"
                self.state = STATE_PLANNER_FAILED

        elif self.state == STATE_VERIFY_FINAL_POSE:
            if self.active_task["type"] == "NAV":
                pos_err, yaw_err = self.get_pose_errors()
                vel_ok = self.is_robot_stopped()
                parked_ok = self.is_goal_stop_manager_parked()
                loc_ok = self.is_localization_confident()
                
                valid = True
                reasons = []
                if pos_err is None or pos_err > 0.40:
                    valid = False
                    reasons.append(f"Position error too high ({pos_err:.2f}m)" if pos_err is not None else "No TF pose")
                if yaw_err is None or yaw_err > 0.30:
                    valid = False
                    reasons.append(f"Heading error too high ({math.degrees(yaw_err):.1f} deg)" if yaw_err is not None else "No TF heading")
                if not vel_ok:
                    valid = False
                    reasons.append("Robot still moving")
                if not parked_ok:
                    valid = False
                    reasons.append("GoalStopManager holding inactive")
                if not loc_ok:
                    valid = False
                    reasons.append("Poor localization confidence")
                    
                if valid:
                    self.task_success = True
                    self.state = STATE_TASK_COMPLETE
                else:
                    # Retry if verify stage times out (15 seconds limit)
                    verify_duration = rospy.Time.now().to_sec() - self.task_start_time
                    if verify_duration > 20.0:
                        self.task_failure_reason = f"Verification failed: {', '.join(reasons)}"
                        self.state = STATE_PLANNER_FAILED
                    else:
                        self.publish_status(f"Waiting for validation checks: {', '.join(reasons)}")
            
            elif self.active_task["type"] == "ACTION":
                parked_ok = self.is_goal_stop_manager_parked()
                loc_ok = self.is_localization_confident()
                elapsed = rospy.Time.now().to_sec() - self.action_start_time
                
                if not parked_ok:
                    self.publish_status("Waiting for parking lock...")
                elif not loc_ok:
                    self.publish_status("Waiting for stable localization...")
                else:
                    self.publish_status(f"Executing action: {elapsed:.1f}s / {self.active_task['action_duration']}s")
                    if elapsed >= self.active_task["action_duration"]:
                        self.task_success = True
                        self.state = STATE_TASK_COMPLETE

        elif self.state == STATE_TASK_COMPLETE:
            self.task_end_time = rospy.Time.now().to_sec()
            if self.navigation_start_time:
                self.navigation_duration = self.task_end_time - self.navigation_start_time
                
            self.log_task_to_sqlite()
            self.publish_event(f"TASK_SUCCESS: {self.active_task['name']}")
            self.state = STATE_NEXT_TASK

        elif self.state == STATE_NEXT_TASK:
            self.current_task_idx += 1
            self.state = STATE_IDLE

        elif self.state == STATE_GOAL_REJECTED or self.state == STATE_PLANNER_FAILED:
            self.task_retries += 1
            self.publish_event(f"TASK_FAILED: {self.active_task['name']} | Reason: {self.task_failure_reason}")
            
            if self.task_retries >= self.max_retries:
                self.state = STATE_MISSION_ABORT
            else:
                self.state = STATE_RECOVERY
                self.recovery_start_time = rospy.Time.now().to_sec()

        elif self.state == STATE_RECOVERY:
            self.task_recovery_count += 1
            self.publish_status(f"Goal recovery active (clearing costmaps). Retry {self.task_retries}/{self.max_retries}")
            self.clear_costmaps()
            
            # recovery sleep
            if rospy.Time.now().to_sec() - self.recovery_start_time > 2.0:
                self.state = STATE_SEND_GOAL

        elif self.state == STATE_MISSION_ABORT:
            self.publish_status(f"Mission Aborted! Failed at task {self.active_task['name']} | Reason: {self.task_failure_reason}")
            self.publish_event(f"MISSION_ABORTED: {self.active_task['name']}")
            self.stop_robot()
            
            self.task_end_time = rospy.Time.now().to_sec()
            self.task_success = False
            self.log_task_to_sqlite()
            rospy.signal_shutdown(f"Mission aborted due to failures: {self.task_failure_reason}")

    # RViz markers
    def publish_visualizations(self):
        marker_arr = MarkerArray()
        
        # 1. Timeline Text Marker
        timeline_marker = Marker()
        timeline_marker.header.stamp = rospy.Time.now()
        timeline_marker.header.frame_id = "map"
        timeline_marker.ns = "mission_timeline"
        timeline_marker.id = 100
        timeline_marker.type = Marker.TEXT_VIEW_FACING
        timeline_marker.action = Marker.ADD
        timeline_marker.pose.position.x = 0.0
        timeline_marker.pose.position.y = -2.0
        timeline_marker.pose.position.z = 3.5
        timeline_marker.scale.z = 0.35 # text size
        timeline_marker.color.r = 1.0
        timeline_marker.color.g = 1.0
        timeline_marker.color.b = 1.0
        timeline_marker.color.a = 1.0
        
        text = f"=== MISSION MANAGER DASHBOARD ===\n"
        text += f"Mission ID: {self.mission_id}\n"
        text += f"System State: {self.state}\n"
        text += f"Progress: {self.current_task_idx}/{len(self.tasks)} Tasks\n"
        text += f"---------------------------------\n"
        for i, t in enumerate(self.tasks):
            prefix = "[ ]"
            if i < self.current_task_idx:
                prefix = "[x]"
            elif i == self.current_task_idx:
                prefix = "[>]"
            
            text += f"{prefix} Task {t['id']}: {t['name']}"
            if i == self.current_task_idx and self.state in [STATE_NAVIGATING, STATE_APPROACHING_GOAL]:
                dist = self.get_distance_to_goal()
                if dist is not None:
                    text += f" (dist: {dist:.2f}m)"
            text += "\n"
        text += f"================================="
        timeline_marker.text = text
        marker_arr.markers.append(timeline_marker)
        
        # 2. Waypoint Spheres
        for i, t in enumerate(self.tasks):
            if t["type"] != "NAV":
                continue
            
            wp_marker = Marker()
            wp_marker.header.stamp = rospy.Time.now()
            wp_marker.header.frame_id = "map"
            wp_marker.ns = "waypoints"
            wp_marker.id = t["id"]
            wp_marker.type = Marker.SPHERE
            wp_marker.action = Marker.ADD
            wp_marker.pose.position.x = t["x"]
            wp_marker.pose.position.y = t["y"]
            wp_marker.pose.position.z = 0.15
            wp_marker.scale.x = 0.5
            wp_marker.scale.y = 0.5
            wp_marker.scale.z = 0.5
            
            # Color coding waypoints
            if i < self.current_task_idx:
                # Green
                wp_marker.color.r = 0.0
                wp_marker.color.g = 1.0
                wp_marker.color.b = 0.0
                wp_marker.color.a = 0.5
            elif i == self.current_task_idx:
                # Yellow
                wp_marker.color.r = 1.0
                wp_marker.color.g = 1.0
                wp_marker.color.b = 0.0
                wp_marker.color.a = 0.8
            else:
                # Blue
                wp_marker.color.r = 0.0
                wp_marker.color.g = 0.5
                wp_marker.color.b = 1.0
                wp_marker.color.a = 0.4
                
            marker_arr.markers.append(wp_marker)
            
            # Label
            lbl_marker = Marker()
            lbl_marker.header.stamp = rospy.Time.now()
            lbl_marker.header.frame_id = "map"
            lbl_marker.ns = "waypoint_labels"
            lbl_marker.id = t["id"] + 1000
            lbl_marker.type = Marker.TEXT_VIEW_FACING
            lbl_marker.action = Marker.ADD
            lbl_marker.pose.position.x = t["x"]
            lbl_marker.pose.position.y = t["y"]
            lbl_marker.pose.position.z = 0.5
            lbl_marker.scale.z = 0.2
            lbl_marker.color.r = 1.0
            lbl_marker.color.g = 1.0
            lbl_marker.color.b = 1.0
            lbl_marker.color.a = 1.0
            lbl_marker.text = f"WP {t['id']}"
            marker_arr.markers.append(lbl_marker)
            
        self.pub_markers.publish(marker_arr)

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.update_state_machine()
            self.publish_visualizations()
            rate.sleep()

if __name__ == '__main__':
    try:
        node = MissionManagerNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
