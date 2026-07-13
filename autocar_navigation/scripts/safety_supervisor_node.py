#!/usr/bin/env python3

import rospy
import sqlite3
import numpy as np
import tf
import math
import threading
import queue
import json
import time
from std_msgs.msg import String, ColorRGBA
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from sensor_msgs.msg import LaserScan, Imu
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, Point
from actionlib_msgs.msg import GoalStatusArray
from visualization_msgs.msg import Marker, MarkerArray
from std_srvs.srv import Empty

class RollingRateTracker:
    def __init__(self, default_period=0.1, window_size=10):
        self.default_period = default_period
        self.window_size = window_size
        self.timestamps = []
        self.lock = threading.Lock()

    def update(self, t):
        with self.lock:
            self.timestamps.append(t)
            if len(self.timestamps) > self.window_size:
                self.timestamps.pop(0)

    def get_timeout_threshold(self):
        with self.lock:
            if len(self.timestamps) < 3:
                return self.default_period * 3.0
            diffs = [self.timestamps[i] - self.timestamps[i-1] for i in range(1, len(self.timestamps))]
            avg_period = sum(diffs) / len(diffs)
            avg_period = max(0.01, avg_period)
            return avg_period * 3.0

class SafetySupervisorNode:
    def __init__(self):
        rospy.init_node('safety_supervisor')
        
        self.data_lock = threading.Lock()

        # Load parameters
        self.db_path = rospy.get_param('~db_path', '/home/mahboob-alam/four_wheel_drive/src/autocar_ai/database/fleet_learning.db')
        self.wheelbase = rospy.get_param('~wheelbase', 0.70)
        self.max_steer_angle = rospy.get_param('~max_steer_angle', 0.785398)
        self.bubble_base_radius = rospy.get_param('~bubble_base_radius', 0.65)
        self.bubble_speed_coef = rospy.get_param('~bubble_speed_coef', 0.45)

        # Rolling rate trackers for watchdogs
        self.rate_scan = RollingRateTracker(default_period=0.1) # 10Hz
        self.rate_odom = RollingRateTracker(default_period=0.033) # 30Hz
        self.rate_imu = RollingRateTracker(default_period=0.02) # 50Hz
        self.rate_amcl = RollingRateTracker(default_period=0.5) # 2Hz
        self.rate_planner = RollingRateTracker(default_period=0.2) # 5Hz

        # Cache variables
        self.cmd_raw = None
        self.ekf_odom = None
        self.imu_data = None
        self.local_costmap = None
        self.local_plan = None
        self.global_plan = None
        self.localization_health = "Excellent"
        self.laser_scan = None
        self.planner_status = None
        self.goal_hold_state = "INIT"
        
        # Message arrival time caches (secs)
        self.last_scan_time = None
        self.last_imu_time = None
        self.last_odom_time = None
        self.last_amcl_time = None
        self.last_planner_time = None
        self.last_cmd_raw_time = None
        self.last_tf_map_odom_time = time.time()
        self.last_tf_odom_base_time = time.time()

        # Health state machine
        self.health_state = "Healthy"
        
        # Clearances
        self.clearance_front = 10.0
        self.clearance_rear = 10.0
        self.clearance_left = 10.0
        self.clearance_right = 10.0
        self.nearest_obstacle_dist = 10.0
        self.nearest_obstacle_angle = 0.0
        self.corridor_width = 3.0

        # Trajectory Validator Variables
        self.trajectory_valid = True
        self.collision_prediction = False
        self.time_to_collision = -1.0
        self.future_clearance = 3.0
        self.planner_quality_score = 100.0
        self.reason_for_replan = ""
        self.speed_scaling_factor = 1.0
        self.safety_state = "Safe"
        self.consecutive_rejections = 0
        self.last_replan_time = 0.0

        # Recovery state machine
        self.recovery_state = "NORMAL" # NORMAL, RECOVERY_STOP, RECOVERY_SEARCH, RECOVERY_MANEUVER, RECOVERY_CLEAR
        self.recovery_start_time = 0.0
        self.recovery_v = 0.0
        self.recovery_w = 0.0
        self.recovery_yaw_direction = 0.0

        # Swept path simulation
        self.predicted_swept_path = [] # list of (x, y, yaw)
        self.predicted_footprints = [] # list of 8 footprint points at future steps
        self.collision_point = None

        # Offloaded DB and visualizer queues
        self.log_queue = queue.Queue()
        self.marker_queue = queue.Queue()
        
        self.db_init()
        
        # Worker Threads
        self.worker_thread = threading.Thread(target=self.async_worker)
        self.worker_thread.daemon = True
        self.worker_thread.start()

        # TF Listener
        self.tf_listener = tf.TransformListener()

        # Service Client for clear costmaps
        self.client_clear_costmaps = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)

        # Publishers
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.pub_status = rospy.Publisher('/safety/status', String, queue_size=5, latch=True)
        self.pub_diagnostics = rospy.Publisher('/safety_diagnostics', String, queue_size=5, latch=True)
        self.pub_markers = rospy.Publisher('/safety/markers', MarkerArray, queue_size=5, latch=True)

        # Subscribers
        self.cmd_raw_topic = rospy.get_param('~cmd_raw_topic', '/cmd_vel_raw')
        self.sub_cmd_raw = rospy.Subscriber(self.cmd_raw_topic, Twist, self.cmd_raw_callback, queue_size=1)
        self.sub_laser = rospy.Subscriber('/scan_deskewed', LaserScan, self.laser_callback, queue_size=1)
        self.sub_imu = rospy.Subscriber('/imu/data', Imu, self.imu_callback, queue_size=1)
        self.sub_odom = rospy.Subscriber('/odometry/filtered', Odometry, self.odom_callback, queue_size=1)
        self.sub_amcl = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_callback, queue_size=1)
        self.sub_planner_status = rospy.Subscriber('/move_base/status', GoalStatusArray, self.planner_status_callback, queue_size=1)
        self.sub_costmap = rospy.Subscriber('/move_base/local_costmap/costmap', OccupancyGrid, self.costmap_callback, queue_size=1)
        
        self.sub_local_path = rospy.Subscriber('/move_base/TebLocalPlannerROS/local_plan', Path, self.local_path_callback, queue_size=1)
        self.sub_local_path_dwa = rospy.Subscriber('/move_base/DWAPlannerROS/local_plan', Path, self.local_path_callback, queue_size=1)
        self.sub_global_path = rospy.Subscriber('/move_base/GlobalPlanner/plan', Path, self.global_plan_callback, queue_size=1)
        
        self.sub_loc_health = rospy.Subscriber('/localization_health', String, self.loc_health_callback, queue_size=1)
        self.sub_hold_state = rospy.Subscriber('/goal_hold_state', String, self.hold_state_callback, queue_size=1)

        # Control Loop: 25 Hz
        self.timer = rospy.Timer(rospy.Duration(0.04), self.control_loop)

        rospy.loginfo("[SafetySupervisor] Redesigned Trajectory Validator active at 25 Hz.")

    def db_init(self):
        try:
            conn = sqlite3.connect(self.db_path)
            cursor = conn.cursor()
            cursor.execute("""
            CREATE TABLE IF NOT EXISTS safety_supervisor_events (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                timestamp REAL,
                x REAL, y REAL, yaw REAL,
                speed REAL, steering_angle REAL,
                localization_confidence REAL,
                clearance_front REAL, clearance_rear REAL, clearance_left REAL, clearance_right REAL,
                risk_score REAL, risk_level TEXT,
                active_layer INTEGER,
                intervention_type TEXT,
                speed_reduction REAL,
                sensor_status TEXT
            )
            """)
            cursor.execute("""
            CREATE TABLE IF NOT EXISTS health_transitions (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                timestamp REAL,
                old_state TEXT,
                new_state TEXT,
                component TEXT,
                age REAL,
                threshold REAL,
                reason TEXT
            )
            """)
            conn.commit()
            conn.close()
        except Exception as e:
            rospy.logerr(f"[SafetySupervisor] DB Init failed: {e}")

    def async_worker(self):
        while not rospy.is_shutdown():
            try:
                while not self.log_queue.empty():
                    event_type, data = self.log_queue.get_nowait()
                    conn = sqlite3.connect(self.db_path)
                    cursor = conn.cursor()
                    if event_type == "event":
                        cursor.execute("""
                        INSERT INTO safety_supervisor_events (
                            timestamp, x, y, yaw, speed, steering_angle, localization_confidence,
                            clearance_front, clearance_rear, clearance_left, clearance_right,
                            risk_score, risk_level, active_layer, intervention_type, speed_reduction, sensor_status
                        ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                        """, data)
                    elif event_type == "transition":
                        cursor.execute("""
                        INSERT INTO health_transitions (timestamp, old_state, new_state, component, age, threshold, reason)
                        VALUES (?, ?, ?, ?, ?, ?, ?)
                        """, data)
                    conn.commit()
                    conn.close()
            except Exception:
                pass

            try:
                while not self.marker_queue.empty():
                    markers = self.marker_queue.get_nowait()
                    self.pub_markers.publish(markers)
            except Exception:
                pass

            time.sleep(0.04)

    def cmd_raw_callback(self, msg):
        with self.data_lock:
            self.cmd_raw = msg
            self.last_cmd_raw_time = time.time()

    def laser_callback(self, msg):
        with self.data_lock:
            self.laser_scan = msg
            t_now = time.time()
            self.last_scan_time = t_now
            self.rate_scan.update(t_now)

    def imu_callback(self, msg):
        with self.data_lock:
            self.imu_data = msg
            t_now = time.time()
            self.last_imu_time = t_now
            self.rate_imu.update(t_now)

    def odom_callback(self, msg):
        with self.data_lock:
            self.ekf_odom = msg
            t_now = time.time()
            self.last_odom_time = t_now
            self.rate_odom.update(t_now)

    def amcl_callback(self, msg):
        with self.data_lock:
            t_now = time.time()
            self.last_amcl_time = t_now
            self.rate_amcl.update(t_now)

    def planner_status_callback(self, msg):
        with self.data_lock:
            self.planner_status = msg
            t_now = time.time()
            self.last_planner_time = t_now
            self.rate_planner.update(t_now)

    def costmap_callback(self, msg):
        with self.data_lock:
            self.local_costmap = msg

    def local_path_callback(self, msg):
        with self.data_lock:
            self.local_plan = msg

    def global_plan_callback(self, msg):
        with self.data_lock:
            self.global_plan = msg

    def loc_health_callback(self, msg):
        with self.data_lock:
            self.localization_health = msg.data

    def hold_state_callback(self, msg):
        with self.data_lock:
            self.goal_hold_state = msg.data

    def calculate_clearances(self):
        with self.data_lock:
            if not self.laser_scan:
                return
            ranges = np.array(self.laser_scan.ranges)
            angle_min = self.laser_scan.angle_min
            angle_increment = self.laser_scan.angle_increment
            range_max = self.laser_scan.range_max

        angles = angle_min + np.arange(len(ranges)) * angle_increment
        ranges = np.where(np.isnan(ranges) | np.isinf(ranges), range_max, ranges)
        
        valid_idx = np.where(ranges > 0.05)
        if len(valid_idx[0]) == 0:
            return
        
        ranges = ranges[valid_idx]
        angles = angles[valid_idx]

        # Partition ranges into simple sectors for clearance monitoring
        fc_idx = np.where((angles >= -math.radians(15)) & (angles <= math.radians(15)))
        l_idx = np.where((angles > math.radians(15)) & (angles <= math.radians(90)))
        r_idx = np.where((angles >= -math.radians(90)) & (angles < -math.radians(15)))
        rear_idx = np.where((angles > math.radians(90)) | (angles < -math.radians(90)))

        self.clearance_front = float(np.min(ranges[fc_idx])) if len(fc_idx[0]) > 0 else range_max
        self.clearance_left = float(np.min(ranges[l_idx])) if len(l_idx[0]) > 0 else range_max
        self.clearance_right = float(np.min(ranges[r_idx])) if len(r_idx[0]) > 0 else range_max
        self.clearance_rear = float(np.min(ranges[rear_idx])) if len(rear_idx[0]) > 0 else range_max

        min_idx = np.argmin(ranges)
        self.nearest_obstacle_dist = float(ranges[min_idx])
        self.nearest_obstacle_angle = float(angles[min_idx])
        self.corridor_width = self.clearance_left + self.clearance_right + 0.85

    def get_current_footprint_cost(self):
        with self.data_lock:
            if not self.local_costmap:
                return 0
            resolution = self.local_costmap.info.resolution
            width = self.local_costmap.info.width
            height = self.local_costmap.info.height
            origin_x = self.local_costmap.info.origin.position.x
            origin_y = self.local_costmap.info.origin.position.y
            costmap_data = self.local_costmap.data

        try:
            (trans, rot) = self.tf_listener.lookupTransform(self.local_costmap.header.frame_id, 'base_link', rospy.Time(0))
            x, y = trans[0], trans[1]
            yaw = tf.transformations.euler_from_quaternion(rot)[2]
        except Exception:
            return 0

        max_cost = 0
        footprint_points = [
            (0.6, 0.35), (0.6, -0.35),
            (-0.6, 0.35), (-0.6, -0.35),
            (0.0, 0.35), (0.0, -0.35),
            (0.6, 0.0), (-0.6, 0.0)
        ]
        for fx, fy in footprint_points:
            cx = x + fx * math.cos(yaw) - fy * math.sin(yaw)
            cy = y + fx * math.sin(yaw) + fy * math.cos(yaw)
            gx = int((cx - origin_x) / resolution)
            gy = int((cy - origin_y) / resolution)
            if 0 <= gx < width and 0 <= gy < height:
                cost = costmap_data[gy * width + gx]
                if cost > max_cost:
                    max_cost = cost
        return max_cost

    def find_safest_escape_direction(self):
        with self.data_lock:
            if not self.local_costmap:
                return -0.15, 0.0
            resolution = self.local_costmap.info.resolution
            width = self.local_costmap.info.width
            height = self.local_costmap.info.height
            origin_x = self.local_costmap.info.origin.position.x
            origin_y = self.local_costmap.info.origin.position.y
            costmap_data = self.local_costmap.data

        try:
            (trans, rot) = self.tf_listener.lookupTransform(self.local_costmap.header.frame_id, 'base_link', rospy.Time(0))
            x_curr, y_curr = trans[0], trans[1]
            yaw_curr = tf.transformations.euler_from_quaternion(rot)[2]
        except Exception:
            return -0.15, 0.0

        candidates = [
            (-0.15, 0.0),   # Back straight
            (-0.15, 0.4),   # Back left
            (-0.15, -0.4),  # Back right
            (0.15, 0.4),    # Forward left
            (0.15, -0.4),   # Forward right
            (0.15, 0.0),    # Forward straight
        ]

        best_cand = (-0.15, 0.0)
        min_cost = 999
        footprint_points = [
            (0.6, 0.35), (0.6, -0.35),
            (-0.6, 0.35), (-0.6, -0.35),
            (0.0, 0.35), (0.0, -0.35),
            (0.6, 0.0), (-0.6, 0.0)
        ]

        for v, w in candidates:
            dt = 1.0
            yaw_sim = yaw_curr + w * dt
            x_sim = x_curr + v * math.cos(yaw_sim) * dt
            y_sim = y_curr + v * math.sin(yaw_sim) * dt

            max_cost = 0
            for fx, fy in footprint_points:
                cx = x_sim + fx * math.cos(yaw_sim) - fy * math.sin(yaw_sim)
                cy = y_sim + fx * math.sin(yaw_sim) + fy * math.cos(yaw_sim)
                gx = int((cx - origin_x) / resolution)
                gy = int((cy - origin_y) / resolution)
                if 0 <= gx < width and 0 <= gy < height:
                    cost = costmap_data[gy * width + gx]
                    if cost > max_cost:
                        max_cost = cost
                else:
                    max_cost = max(max_cost, 100)
            
            if max_cost < min_cost:
                min_cost = max_cost
                best_cand = (v, w)

        return best_cand

    def check_watchdogs_and_state(self):
        t_now = time.time()
        th_scan = self.rate_scan.get_timeout_threshold()
        th_odom = self.rate_odom.get_timeout_threshold()
        th_imu = self.rate_imu.get_timeout_threshold()
        th_amcl = self.rate_amcl.get_timeout_threshold()

        age_scan = (t_now - self.last_scan_time) if self.last_scan_time else 999.0
        age_odom = (t_now - self.last_odom_time) if self.last_odom_time else 999.0
        age_imu = (t_now - self.last_imu_time) if self.last_imu_time else 999.0
        age_amcl = (t_now - self.last_amcl_time) if self.last_amcl_time else 999.0

        try:
            self.tf_listener.lookupTransform('map', 'odom', rospy.Time(0))
            self.last_tf_map_odom_time = t_now
        except Exception:
            pass
            
        try:
            self.tf_listener.lookupTransform('odom', 'base_link', rospy.Time(0))
            self.last_tf_odom_base_time = t_now
        except Exception:
            pass

        age_tf_map_odom = t_now - self.last_tf_map_odom_time
        age_tf_odom_base = t_now - self.last_tf_odom_base_time

        failures = []
        warnings = []
        degraded = []

        if age_scan > th_scan * 4.0:
            failures.append(f"LiDAR dropout (age: {age_scan:.2f}s)")
        elif age_scan > th_scan * 2.0:
            degraded.append(f"LiDAR degraded (age: {age_scan:.2f}s)")
        elif age_scan > th_scan:
            warnings.append(f"LiDAR jitter (age: {age_scan:.2f}s)")

        if age_odom > th_odom * 4.0:
            failures.append(f"EKF Odom dropout (age: {age_odom:.2f}s)")
        elif age_odom > th_odom * 2.0:
            degraded.append(f"EKF Odom degraded (age: {age_odom:.2f}s)")
        elif age_odom > th_odom:
            warnings.append(f"EKF Odom jitter (age: {age_odom:.2f}s)")

        if age_imu > th_imu * 4.0:
            failures.append(f"IMU dropout (age: {age_imu:.2f}s)")
        elif age_imu > th_imu * 2.0:
            degraded.append(f"IMU degraded (age: {age_imu:.2f}s)")
        elif age_imu > th_imu:
            warnings.append(f"IMU jitter (age: {age_imu:.2f}s)")

        if age_amcl > th_amcl * 4.0:
            degraded.append(f"AMCL pose sparse (age: {age_amcl:.2f}s)")

        if age_tf_odom_base > 1.2:
            failures.append(f"TF odom->base_link lost (age: {age_tf_odom_base:.2f}s)")

        if age_tf_map_odom > 2.5:
            degraded.append(f"TF map->odom lost (age: {age_tf_map_odom:.2f}s)")

        new_state = "Healthy"
        reason = "All signals normal"
        comp = "None"

        if len(failures) > 0:
            new_state = "Critical"
            reason = "; ".join(failures)
            comp = "Sensors/TF"
        elif len(degraded) > 0:
            new_state = "Degraded"
            reason = "; ".join(degraded)
            comp = "Sensors/AMCL"
        elif len(warnings) > 0:
            new_state = "Warning"
            reason = "; ".join(warnings)
            comp = "Sensors"

        if new_state != self.health_state:
            rospy.logwarn(f"[SafetySupervisor] Health Transition: {self.health_state} -> {new_state}. Reason: {reason}")
            log_data = (t_now, self.health_state, new_state, comp, 0.0, 0.0, reason[:200])
            self.log_queue.put(("transition", log_data))
            self.health_state = new_state

        # Assemble diagnostics JSON
        diagnostics = {
            "trajectory_valid": self.trajectory_valid,
            "collision_prediction": self.collision_prediction,
            "time_to_collision": self.time_to_collision,
            "future_clearance": self.future_clearance,
            "planner_quality_score": self.planner_quality_score,
            "recovery_state": self.recovery_state,
            "reason_for_replan": self.reason_for_replan,
            "speed_scaling_factor": self.speed_scaling_factor,
            "safety_state": self.safety_state,
            "scan_age": age_scan,
            "odom_age": age_odom,
            "imu_age": age_imu,
            "amcl_age": age_amcl,
            "health_state": self.health_state
        }
        self.pub_diagnostics.publish(String(data=json.dumps(diagnostics)))

    def call_clear_costmaps(self):
        try:
            rospy.loginfo("[SafetySupervisor] Requesting move_base clear_costmaps for replanning...")
            rospy.wait_for_service('/move_base/clear_costmaps', timeout=1.0)
            self.client_clear_costmaps()
        except Exception as e:
            rospy.logwarn(f"[SafetySupervisor] Failed to call clear_costmaps service: {e}")

    def simulate_kinematic_trajectory(self, nominal_v, nominal_w, duration=2.5, dt=0.25):
        """Simulates Ackermann model from base_link (0,0,0) in robot local frame."""
        x, y, yaw = 0.0, 0.0, 0.0
        trajectory = []
        steps = int(duration / dt)
        steer = math.atan2(nominal_w * self.wheelbase, abs(nominal_v)) if abs(nominal_v) > 0.01 else 0.0
        
        for _ in range(steps):
            x += nominal_v * math.cos(yaw) * dt
            y += nominal_v * math.sin(yaw) * dt
            yaw += (nominal_v * math.tan(steer) / self.wheelbase) * dt
            trajectory.append((x, y, yaw))
        return trajectory

    def control_loop(self, event):
        t_now = time.time()
        out_msg = Twist()

        with self.data_lock:
            # Check command watchdog timeout (0.5s)
            if self.last_cmd_raw_time and (t_now - self.last_cmd_raw_time > 0.5):
                self.cmd_raw = None
            cmd_raw = self.cmd_raw
            local_plan = self.local_plan
            global_plan = self.global_plan
            local_costmap = self.local_costmap
            hold_state = self.goal_hold_state

        self.check_watchdogs_and_state()
        self.calculate_clearances()

        # Check Goal Hold Mode
        if hold_state == "HOLDING":
            self.safety_state = "Hold State"
            self.trajectory_valid = True
            self.speed_scaling_factor = 0.0
            out_msg.linear.x = 0.0
            out_msg.angular.z = 0.0
            self.pub_cmd_vel.publish(out_msg)
            self.publish_status_and_markers()
            return

        # Check Goal Distance for Precision Mode
        dist_to_goal = 999.0
        if global_plan and len(global_plan.poses) > 0:
            try:
                goal_pose = global_plan.poses[-1]
                (trans, rot) = self.tf_listener.lookupTransform('map', 'base_link', rospy.Time(0))
                rx, ry = trans[0], trans[1]
                gx = goal_pose.pose.position.x
                gy = goal_pose.pose.position.y
                dist_to_goal = math.hypot(gx - rx, gy - ry)
            except Exception:
                pass

        if dist_to_goal < 1.5:
            self.safety_state = "Precision Parking"
        else:
            self.safety_state = "Normal Navigation"

        # Check Critical state (sensor dropout)
        if self.health_state == "Critical":
            self.trajectory_valid = False
            self.speed_scaling_factor = 0.0
            self.pub_cmd_vel.publish(out_msg)
            self.publish_status_and_markers()
            return

        # Get nominal planner command details
        nominal_v = 0.0
        nominal_w = 0.0
        if cmd_raw:
            nominal_v = cmd_raw.linear.x
            nominal_w = cmd_raw.angular.z

        # --- RECOVERY STATE MACHINE ---
        curr_footprint_cost = self.get_current_footprint_cost()
        if self.recovery_state == "NORMAL":
            if curr_footprint_cost >= 245 or self.consecutive_rejections >= 25:
                self.recovery_state = "RECOVERY_STOP"
                self.recovery_start_time = t_now
                rospy.logwarn(f"[SafetySupervisor] Stuck or blocked. Entering RECOVERY. Footprint cost: {curr_footprint_cost}, Rejections: {self.consecutive_rejections}")
        
        if self.recovery_state != "NORMAL":
            if self.recovery_state == "RECOVERY_STOP":
                out_msg.linear.x = 0.0
                out_msg.angular.z = 0.0
                if t_now - self.recovery_start_time > 0.8:
                    self.recovery_state = "RECOVERY_SEARCH"
            
            elif self.recovery_state == "RECOVERY_SEARCH":
                self.recovery_v, self.recovery_w = self.find_safest_escape_direction()
                self.recovery_yaw_direction = math.atan2(self.recovery_w * self.wheelbase, abs(self.recovery_v)) if abs(self.recovery_v) > 0.01 else 0.0
                self.recovery_state = "RECOVERY_MANEUVER"
                self.recovery_start_time = t_now
            
            elif self.recovery_state == "RECOVERY_MANEUVER":
                out_msg.linear.x = self.recovery_v
                out_msg.angular.z = self.recovery_w
                if t_now - self.recovery_start_time > 1.8:
                    self.recovery_state = "RECOVERY_CLEAR"
            
            elif self.recovery_state == "RECOVERY_CLEAR":
                out_msg.linear.x = 0.0
                out_msg.angular.z = 0.0
                self.consecutive_rejections = 0
                self.recovery_state = "NORMAL"
                # Trigger an async clear_costmaps to give move_base a fresh grid
                threading.Thread(target=self.call_clear_costmaps).start()

            self.pub_cmd_vel.publish(out_msg)
            self.publish_status_and_markers()
            return

        # --- TRAJECTORY VALIDATION ---
        self.predicted_swept_path = []
        self.predicted_footprints = []
        self.collision_prediction = False
        self.collision_point = None
        self.time_to_collision = -1.0
        self.future_clearance = 3.0
        
        max_cost_along_path = 0
        curvature_changes = []
        prev_steer = None

        if local_costmap:
            resolution = local_costmap.info.resolution
            width = local_costmap.info.width
            height = local_costmap.info.height
            origin_x = local_costmap.info.origin.position.x
            origin_y = local_costmap.info.origin.position.y
            costmap_data = local_costmap.data
            costmap_frame = local_costmap.header.frame_id

            # Determine whether to use local_plan or kinematic projection
            path_poses = []
            if local_plan and len(local_plan.poses) > 0 and (t_now - local_plan.header.stamp.to_sec() < 0.5):
                # We have a valid recent local plan from TEB! Transform and evaluate it
                try:
                    self.tf_listener.waitForTransform(costmap_frame, local_plan.header.frame_id, rospy.Time(0), rospy.Duration(0.1))
                    (trans, rot) = self.tf_listener.lookupTransform(costmap_frame, local_plan.header.frame_id, rospy.Time(0))
                    mat = tf.transformations.quaternion_matrix(rot)
                    
                    for pose in local_plan.poses[:15]: # check next 15 path points (approx 2-3s ahead)
                        px = pose.pose.position.x
                        py = pose.pose.position.y
                        pyaw = tf.transformations.euler_from_quaternion([
                            pose.pose.orientation.x,
                            pose.pose.orientation.y,
                            pose.pose.orientation.z,
                            pose.pose.orientation.w
                        ])[2]
                        
                        # Apply transform
                        p_orig = [px, py, 0.0]
                        p_trans = np.dot(mat[:3, :3], p_orig) + trans
                        q_pose = tf.transformations.quaternion_from_euler(0, 0, pyaw)
                        q_new = tf.transformations.quaternion_multiply(rot, q_pose)
                        yaw_new = tf.transformations.euler_from_quaternion(q_new)[2]
                        
                        path_poses.append((p_trans[0], p_trans[1], yaw_new))
                except Exception:
                    path_poses = []

            # If local plan lookup failed or is missing, fall back to kinematic projection
            if len(path_poses) == 0:
                traj_local = self.simulate_kinematic_trajectory(nominal_v, nominal_w, duration=2.5, dt=0.25)
                try:
                    (trans, rot) = self.tf_listener.lookupTransform(costmap_frame, 'base_link', rospy.Time(0))
                    mat = tf.transformations.quaternion_matrix(rot)
                    for lx, ly, lyaw in traj_local:
                        p_orig = [lx, ly, 0.0]
                        p_trans = np.dot(mat[:3, :3], p_orig) + trans
                        q_pose = tf.transformations.quaternion_from_euler(0, 0, lyaw)
                        q_new = tf.transformations.quaternion_multiply(rot, q_pose)
                        yaw_new = tf.transformations.euler_from_quaternion(q_new)[2]
                        path_poses.append((p_trans[0], p_trans[1], yaw_new))
                except Exception:
                    path_poses = []

            # Evaluate footprints along path_poses
            footprint_points = [
                (0.6, 0.35), (0.6, -0.35),
                (-0.6, 0.35), (-0.6, -0.35),
                (0.0, 0.35), (0.0, -0.35),
                (0.6, 0.0), (-0.6, 0.0)
            ]

            self.predicted_swept_path = path_poses
            
            for idx, (x, y, yaw) in enumerate(path_poses):
                step_footprint = []
                step_max_cost = 0
                
                # Check steering change for curvature quality evaluation
                if prev_steer is not None:
                    curvature_changes.append(abs(yaw - prev_steer))
                prev_steer = yaw

                for fx, fy in footprint_points:
                    cx = x + fx * math.cos(yaw) - fy * math.sin(yaw)
                    cy = y + fx * math.sin(yaw) + fy * math.cos(yaw)
                    step_footprint.append((cx, cy))
                    
                    gx = int((cx - origin_x) / resolution)
                    gy = int((cy - origin_y) / resolution)
                    if 0 <= gx < width and 0 <= gy < height:
                        cost = costmap_data[gy * width + gx]
                        if cost > step_max_cost:
                            step_max_cost = cost
                
                self.predicted_footprints.append(step_footprint)
                if step_max_cost > max_cost_along_path:
                    max_cost_along_path = step_max_cost

                # Clearance estimate
                step_clearance = max(0.0, 0.8 * (254 - step_max_cost) / 254.0)
                if step_clearance < self.future_clearance:
                    self.future_clearance = step_clearance

                # Collision prediction check
                if step_max_cost >= 254 and not self.collision_prediction:
                    self.collision_prediction = True
                    self.collision_point = (x, y)
                    self.time_to_collision = idx * 0.25

        # Check reverse maneuver safety
        reverse_unsafe = False
        if nominal_v < -0.01 and self.clearance_rear < 0.4:
            reverse_unsafe = True

        # Trajectory Quality Monitoring & Decision
        oscillation_penalty = 0.0
        if len(curvature_changes) > 1:
            avg_diff = sum(curvature_changes) / len(curvature_changes)
            if avg_diff > 0.4:
                oscillation_penalty = 25.0

        cost_penalty = (max_cost_along_path / 254.0) * 60.0
        clearance_penalty = max(0.0, 1.0 - self.nearest_obstacle_dist / 1.5) * 15.0
        
        self.planner_quality_score = 100.0 - cost_penalty - clearance_penalty - oscillation_penalty
        self.planner_quality_score = max(0.0, min(100.0, self.planner_quality_score))

        # Reject path if lethal collision, rear collision, or clearance falls below unsafe thresholds
        reject_threshold_cost = 240
        if self.safety_state == "Precision Parking":
            # Be more lenient close to the goal point
            reject_threshold_cost = 253

        trajectory_rejected = False
        self.reason_for_replan = ""

        if self.collision_prediction:
            trajectory_rejected = True
            self.reason_for_replan = "Lethal collision predicted"
        elif max_cost_along_path >= reject_threshold_cost:
            trajectory_rejected = True
            self.reason_for_replan = f"Footprint enters high cost zone ({max_cost_along_path})"
        elif reverse_unsafe:
            trajectory_rejected = True
            self.reason_for_replan = "Reverse path rear clearance unsafe"
        elif self.future_clearance < 0.15 and self.safety_state != "Precision Parking":
            trajectory_rejected = True
            self.reason_for_replan = f"Clearance too low ({self.future_clearance:.2f}m)"

        if trajectory_rejected:
            self.trajectory_valid = False
            self.consecutive_rejections += 1
            # Request replan
            if t_now - self.last_replan_time > 1.5:
                self.last_replan_time = t_now
                threading.Thread(target=self.call_clear_costmaps).start()
            
            # Reject: publish 0 velocity, keeping steering command unchanged
            self.speed_scaling_factor = 0.0
            out_msg.linear.x = 0.0
            out_msg.angular.z = nominal_w
            self.pub_cmd_vel.publish(out_msg)
            self.publish_status_and_markers()
            return
        else:
            self.trajectory_valid = True
            self.consecutive_rejections = 0

        # --- SPEED MANAGER ---
        # 1. Curve Slowdown
        nominal_steer = math.atan2(nominal_w * self.wheelbase, abs(nominal_v)) if abs(nominal_v) > 0.01 else 0.0
        S_curve = 1.0 - 0.40 * (abs(nominal_steer) / self.max_steer_angle)

        # 2. Obstacle Proximity Slowdown
        if self.nearest_obstacle_dist < 0.35:
            S_obs = 0.0
        elif self.nearest_obstacle_dist < 1.2:
            S_obs = (self.nearest_obstacle_dist - 0.35) / 0.85
        else:
            S_obs = 1.0

        # 3. Corridor width Slowdown
        if self.corridor_width < 1.4:
            S_corridor = 0.35
        elif self.corridor_width < 1.9:
            S_corridor = 0.65
        else:
            S_corridor = 1.0

        # 4. Future Footprint Cost Slowdown
        if max_cost_along_path > 180:
            S_cost = 1.0 - 0.70 * (min(254, max_cost_along_path) - 180) / 74.0
        else:
            S_cost = 1.0

        # 5. Health & Localization confidence slowdowns
        health_speed_scale = 1.0
        if self.health_state == "Degraded":
            health_speed_scale = 0.40
        elif self.health_state == "Warning":
            health_speed_scale = 0.75

        loc_scale = 1.0
        if self.localization_health == "Poor":
            loc_scale = 0.35
        elif self.localization_health == "Warning":
            loc_scale = 0.60

        if self.safety_state == "Precision Parking":
            # Disable corridor, health, and curve-based slowdown parameters near the goal
            self.speed_scaling_factor = min(S_obs, S_cost)
        else:
            self.speed_scaling_factor = min(health_speed_scale, loc_scale, S_cost, S_curve, S_obs, S_corridor)
        
        self.speed_scaling_factor = max(0.0, min(1.0, self.speed_scaling_factor))

        # Output scaled velocity and proportional steering rates
        out_msg.linear.x = nominal_v * self.speed_scaling_factor
        if abs(nominal_v) > 0.01:
            out_msg.angular.z = nominal_w * self.speed_scaling_factor
        else:
            out_msg.angular.z = nominal_w

        self.pub_cmd_vel.publish(out_msg)
        self.publish_status_and_markers()

    def publish_status_and_markers(self):
        # Human readable status message
        valid_str = "VALID" if self.trajectory_valid else "REJECTED"
        status_data = f"State:{self.safety_state} | Traj:{valid_str} | Quality:{self.planner_quality_score:.1f} | SpeedScale:{self.speed_scaling_factor:.2f}"
        self.pub_status.publish(String(data=status_data))

        marker_arr = MarkerArray()
        t_now = rospy.Time.now()

        # 1. Text Status Marker
        text_marker = Marker()
        text_marker.header.frame_id = "base_link"
        text_marker.header.stamp = t_now
        text_marker.ns = "safety_diagnostics"
        text_marker.id = 300
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.pose.position.z = 1.6
        text_marker.scale.z = 0.22
        text_marker.text = (
            f"Trajectory Validator\n"
            f"State: {self.safety_state}\n"
            f"Traj Status: {valid_str} (Score: {self.planner_quality_score:.1f})\n"
            f"Speed Scaling: {self.speed_scaling_factor*100.0:.1f}%\n"
            f"Recovery: {self.recovery_state}"
        )
        if self.trajectory_valid:
            text_marker.color.r, text_marker.color.g, text_marker.color.b = 0.0, 1.0, 0.0
        else:
            text_marker.color.r, text_marker.color.g, text_marker.color.b = 1.0, 0.0, 0.0
        text_marker.color.a = 1.0
        text_marker.lifetime = rospy.Duration(0.2)
        marker_arr.markers.append(text_marker)

        # 2. Predicted Swept Path Trajectory Line
        with self.data_lock:
            local_costmap = self.local_costmap

        if local_costmap and len(self.predicted_swept_path) > 0:
            traj_marker = Marker()
            traj_marker.header.frame_id = local_costmap.header.frame_id
            traj_marker.header.stamp = t_now
            traj_marker.ns = "predicted_path"
            traj_marker.id = 301
            traj_marker.type = Marker.LINE_STRIP
            traj_marker.action = Marker.ADD
            traj_marker.scale.x = 0.05
            if self.trajectory_valid:
                traj_marker.color.r, traj_marker.color.g, traj_marker.color.b = 0.0, 0.8, 0.0
            else:
                traj_marker.color.r, traj_marker.color.g, traj_marker.color.b = 1.0, 0.0, 0.0
            traj_marker.color.a = 0.7
            traj_marker.lifetime = rospy.Duration(0.2)

            for x, y, _ in self.predicted_swept_path:
                traj_marker.points.append(Point(x=x, y=y, z=0.03))
            marker_arr.markers.append(traj_marker)

        # 3. Translucent Predicted Footprints
        if local_costmap and len(self.predicted_footprints) > 0:
            for idx, footprint in enumerate(self.predicted_footprints):
                foot_marker = Marker()
                foot_marker.header.frame_id = local_costmap.header.frame_id
                foot_marker.header.stamp = t_now
                foot_marker.ns = "footprint_projection"
                foot_marker.id = 400 + idx
                foot_marker.type = Marker.LINE_STRIP
                foot_marker.action = Marker.ADD
                foot_marker.scale.x = 0.02
                if self.trajectory_valid:
                    foot_marker.color.r, foot_marker.color.g, foot_marker.color.b = 0.2, 0.7, 1.0
                else:
                    foot_marker.color.r, foot_marker.color.g, foot_marker.color.b = 1.0, 0.2, 0.2
                foot_marker.color.a = 0.25
                foot_marker.lifetime = rospy.Duration(0.2)

                for fx, fy in footprint:
                    foot_marker.points.append(Point(x=fx, y=fy, z=0.02))
                foot_marker.points.append(Point(x=footprint[0][0], y=footprint[0][1], z=0.02))
                marker_arr.markers.append(foot_marker)

        # 4. Collision Point Sphere
        if self.collision_prediction and self.collision_point and local_costmap:
            sphere = Marker()
            sphere.header.frame_id = local_costmap.header.frame_id
            sphere.header.stamp = t_now
            sphere.ns = "collision_point"
            sphere.id = 302
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = self.collision_point[0]
            sphere.pose.position.y = self.collision_point[1]
            sphere.pose.position.z = 0.15
            sphere.scale.x, sphere.scale.y, sphere.scale.z = 0.25, 0.25, 0.25
            sphere.color.r, sphere.color.g, sphere.color.b = 1.0, 0.0, 0.0
            sphere.color.a = 0.95
            sphere.lifetime = rospy.Duration(0.2)
            marker_arr.markers.append(sphere)

        # 5. Recovery Direction Vector Arrow
        if self.recovery_state == "RECOVERY_MANEUVER":
            esc_marker = Marker()
            esc_marker.header.frame_id = "base_link"
            esc_marker.header.stamp = t_now
            esc_marker.ns = "recovery_direction"
            esc_marker.id = 303
            esc_marker.type = Marker.ARROW
            esc_marker.action = Marker.ADD
            esc_marker.scale.x = 0.08
            esc_marker.scale.y = 0.16
            esc_marker.scale.z = 0.2
            esc_marker.color.r, esc_marker.color.g, esc_marker.color.b = 0.0, 1.0, 1.0
            esc_marker.color.a = 1.0
            esc_marker.lifetime = rospy.Duration(0.2)
            
            p_start = Point(x=0.0, y=0.0, z=0.25)
            p_end = Point(x=1.0 * math.cos(self.recovery_yaw_direction), y=1.0 * math.sin(self.recovery_yaw_direction), z=0.25)
            esc_marker.points.append(p_start)
            esc_marker.points.append(p_end)
            marker_arr.markers.append(esc_marker)

        self.marker_queue.put(marker_arr)

    def logging_loop(self):
        with self.data_lock:
            ekf_odom = self.ekf_odom
            
        if not ekf_odom:
            return
            
        x = ekf_odom.pose.pose.position.x
        y = ekf_odom.pose.pose.position.y
        q = [
            ekf_odom.pose.pose.orientation.x,
            ekf_odom.pose.pose.orientation.y,
            ekf_odom.pose.pose.orientation.z,
            ekf_odom.pose.pose.orientation.w
        ]
        yaw = tf.transformations.euler_from_quaternion(q)[2]
        speed = ekf_odom.twist.twist.linear.x
        w = ekf_odom.twist.twist.angular.z
        steering_angle = math.atan2(self.wheelbase * w, abs(speed)) if abs(speed) > 0.01 else 0.0
        
        # Calculate a simplified risk score for backwards compatibility with database
        clearance_factor = max(0.0, 1.0 - self.nearest_obstacle_dist / 3.0)
        risk_score = (clearance_factor * 0.4 + (1.0 - self.speed_scaling_factor) * 0.6) * 100.0

        data = (
            time.time(),
            x, y, yaw,
            speed, steering_angle,
            1.0 if self.localization_health in ["Excellent", "Good"] else 0.5,
            self.clearance_front, self.clearance_rear, self.clearance_left, self.clearance_right,
            risk_score, "High" if risk_score > 60.0 else "Safe",
            3 if not self.trajectory_valid else 0,
            self.reason_for_replan if not self.trajectory_valid else "NONE",
            (1.0 - self.speed_scaling_factor) * 100.0,
            self.health_state
        )
        self.log_queue.put(("event", data))

if __name__ == '__main__':
    try:
        node = SafetySupervisorNode()
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            node.logging_loop()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
