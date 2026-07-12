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
from std_msgs.msg import String
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from sensor_msgs.msg import LaserScan, Imu
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from actionlib_msgs.msg import GoalStatusArray
from visualization_msgs.msg import Marker, MarkerArray

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
                # Use default fallback timeout (3x default period)
                return self.default_period * 3.0
            
            diffs = [self.timestamps[i] - self.timestamps[i-1] for i in range(1, len(self.timestamps))]
            avg_period = sum(diffs) / len(diffs)
            
            # Avoid divide-by-zero or extremely small periods causing zero timeout
            avg_period = max(0.01, avg_period)
            
            # Return 3x average period
            return avg_period * 3.0

class SafetySupervisorNode:
    def __init__(self):
        rospy.init_node('safety_supervisor')

        # Load parameters
        self.db_path = rospy.get_param('~db_path', '/home/mahboob-alam/four_wheel_drive/src/autocar_ai/database/fleet_learning.db')
        self.wheelbase = rospy.get_param('~wheelbase', 0.70)
        self.max_steer_angle = rospy.get_param('~max_steer_angle', 0.785398)
        self.bubble_base_radius = rospy.get_param('~bubble_base_radius', 0.65)
        self.bubble_speed_coef = rospy.get_param('~bubble_speed_coef', 0.45)

        # rolling rate trackers
        self.rate_scan = RollingRateTracker(default_period=0.1) # 10Hz
        self.rate_odom = RollingRateTracker(default_period=0.033) # 30Hz
        self.rate_imu = RollingRateTracker(default_period=0.02) # 50Hz
        self.rate_amcl = RollingRateTracker(default_period=0.5) # 2Hz
        self.rate_planner = RollingRateTracker(default_period=0.2) # 5Hz

        # Cache variables (with thread safety locks where appropriate)
        self.cmd_raw = None
        self.ekf_odom = None
        self.imu_data = None
        self.local_costmap = None
        self.local_plan = None
        self.localization_health = "Excellent"
        self.laser_scan = None
        self.planner_status = None
        
        # Message arrival time caches (secs)
        self.last_scan_time = None
        self.last_imu_time = None
        self.last_odom_time = None
        self.last_amcl_time = None
        self.last_planner_time = None
        self.last_tf_map_odom_time = time.time()
        self.last_tf_odom_base_time = time.time()

        # Health state machine
        # States: "Healthy", "Warning", "Degraded", "Critical", "Emergency Stop"
        self.health_state = "Healthy"
        self.last_health_state = "Healthy"
        
        # Safety clearance states
        self.clearance_front = 10.0
        self.clearance_rear = 10.0
        self.clearance_left = 10.0
        self.clearance_right = 10.0
        self.risk_score = 0.0
        self.risk_level = "Safe"
        self.active_layer = 0
        self.intervention_type = "NONE"
        self.speed_reduction = 0.0

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

        # Publishers
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.pub_status = rospy.Publisher('/safety/status', String, queue_size=5, latch=True)
        self.pub_diagnostics = rospy.Publisher('/safety_diagnostics', String, queue_size=5, latch=True)
        self.pub_markers = rospy.Publisher('/safety/markers', MarkerArray, queue_size=5, latch=True)

        # Subscribers
        self.sub_cmd_raw = rospy.Subscriber('/cmd_vel_raw', Twist, self.cmd_raw_callback, queue_size=1)
        self.sub_laser = rospy.Subscriber('/scan_deskewed', LaserScan, self.laser_callback, queue_size=1)
        self.sub_imu = rospy.Subscriber('/imu/data', Imu, self.imu_callback, queue_size=1)
        self.sub_odom = rospy.Subscriber('/odometry/filtered', Odometry, self.odom_callback, queue_size=1)
        self.sub_amcl = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_callback, queue_size=1)
        self.sub_planner_status = rospy.Subscriber('/move_base/status', GoalStatusArray, self.planner_status_callback, queue_size=1)
        self.sub_costmap = rospy.Subscriber('/move_base/local_costmap/costmap', OccupancyGrid, self.costmap_callback, queue_size=1)
        
        self.sub_local_path = rospy.Subscriber('/move_base/TebLocalPlannerROS/local_plan', Path, self.local_path_callback, queue_size=1)
        self.sub_local_path_dwa = rospy.Subscriber('/move_base/DWAPlannerROS/local_plan', Path, self.local_path_callback, queue_size=1)
        
        self.sub_loc_health = rospy.Subscriber('/localization_health', String, self.loc_health_callback, queue_size=1)

        # Control Loop: 10 Hz
        self.timer = rospy.Timer(rospy.Duration(0.1), self.control_loop)

        rospy.loginfo("[SafetySupervisor] Redesigned adaptive watchdog supervisor initialized.")

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
        # Dedicated worker thread for DB writes and RViz Marker updates
        while not rospy.is_shutdown():
            # 1. Process Database logging events
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
            except Exception as e:
                pass

            # 2. Process RViz markers
            try:
                while not self.marker_queue.empty():
                    markers = self.marker_queue.get_nowait()
                    self.pub_markers.publish(markers)
            except Exception:
                pass

            time.sleep(0.05)

    def cmd_raw_callback(self, msg):
        self.cmd_raw = msg

    def laser_callback(self, msg):
        self.laser_scan = msg
        t_now = time.time()
        self.last_scan_time = t_now
        self.rate_scan.update(t_now)

    def imu_callback(self, msg):
        self.imu_data = msg
        t_now = time.time()
        self.last_imu_time = t_now
        self.rate_imu.update(t_now)

    def odom_callback(self, msg):
        self.ekf_odom = msg
        t_now = time.time()
        self.last_odom_time = t_now
        self.rate_odom.update(t_now)

    def amcl_callback(self, msg):
        t_now = time.time()
        self.last_amcl_time = t_now
        self.rate_amcl.update(t_now)

    def planner_status_callback(self, msg):
        self.planner_status = msg
        t_now = time.time()
        self.last_planner_time = t_now
        self.rate_planner.update(t_now)

    def costmap_callback(self, msg):
        self.local_costmap = msg
        self.last_costmap_time = time.time()

    def local_path_callback(self, msg):
        self.local_plan = msg

    def loc_health_callback(self, msg):
        self.localization_health = msg.data

    def calculate_clearances(self):
        if not self.laser_scan:
            return
            
        ranges = np.array(self.laser_scan.ranges)
        angles = np.linspace(self.laser_scan.angle_min, self.laser_scan.angle_max, len(ranges))
        ranges = np.where(np.isnan(ranges) | np.isinf(ranges), self.laser_scan.range_max, ranges)
        
        front_idx = np.where((angles >= -math.radians(30)) & (angles <= math.radians(30)))
        left_idx = np.where((angles > math.radians(30)) & (angles < math.radians(120)))
        right_idx = np.where((angles < -math.radians(30)) & (angles > -math.radians(120)))
        rear_idx = np.where((angles >= math.radians(120)) | (angles <= -math.radians(120)))
        
        self.clearance_front = float(np.min(ranges[front_idx])) if len(front_idx[0]) > 0 else 10.0
        self.clearance_left = float(np.min(ranges[left_idx])) if len(left_idx[0]) > 0 else 10.0
        self.clearance_right = float(np.min(ranges[right_idx])) if len(right_idx[0]) > 0 else 10.0
        self.clearance_rear = float(np.min(ranges[rear_idx])) if len(rear_idx[0]) > 0 else 10.0

    def check_watchdogs_and_state(self):
        t_now = time.time()
        
        # 1. Fetch Dynamic Thresholds
        th_scan = self.rate_scan.get_timeout_threshold()
        th_odom = self.rate_odom.get_timeout_threshold()
        th_imu = self.rate_imu.get_timeout_threshold()
        th_amcl = self.rate_amcl.get_timeout_threshold()
        th_planner = self.rate_planner.get_timeout_threshold()

        # 2. Compute Age values
        age_scan = (t_now - self.last_scan_time) if self.last_scan_time else 999.0
        age_odom = (t_now - self.last_odom_time) if self.last_odom_time else 999.0
        age_imu = (t_now - self.last_imu_time) if self.last_imu_time else 999.0
        age_amcl = (t_now - self.last_amcl_time) if self.last_amcl_time else 999.0
        age_planner = (t_now - self.last_planner_time) if self.last_planner_time else 999.0

        # Check TF status
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

        # 3. Watchdog evaluations (Dynamic period scaling)
        failures = []
        warnings = []
        degraded = []

        # LiDAR
        if age_scan > th_scan * 4.0:
            failures.append(f"LiDAR dropout (age: {age_scan:.2f}s, th: {th_scan * 4.0:.2f}s)")
        elif age_scan > th_scan * 2.0:
            degraded.append(f"LiDAR degraded (age: {age_scan:.2f}s)")
        elif age_scan > th_scan:
            warnings.append(f"LiDAR jitter (age: {age_scan:.2f}s)")

        # Odom (EKF)
        if age_odom > th_odom * 4.0:
            failures.append(f"EKF Odom dropout (age: {age_odom:.2f}s, th: {th_odom * 4.0:.2f}s)")
        elif age_odom > th_odom * 2.0:
            degraded.append(f"EKF Odom degraded (age: {age_odom:.2f}s)")
        elif age_odom > th_odom:
            warnings.append(f"EKF Odom jitter (age: {age_odom:.2f}s)")

        # IMU
        if age_imu > th_imu * 4.0:
            failures.append(f"IMU dropout (age: {age_imu:.2f}s, th: {th_imu * 4.0:.2f}s)")
        elif age_imu > th_imu * 2.0:
            degraded.append(f"IMU degraded (age: {age_imu:.2f}s)")
        elif age_imu > th_imu:
            warnings.append(f"IMU jitter (age: {age_imu:.2f}s)")

        # AMCL (Localization updates can be slower, threshold handles it dynamically)
        if age_amcl > th_amcl * 4.0:
            degraded.append(f"AMCL pose sparse (age: {age_amcl:.2f}s, th: {th_amcl * 4.0:.2f}s)")
        elif age_amcl > th_amcl * 2.0:
            warnings.append(f"AMCL pose delayed (age: {age_amcl:.2f}s)")

        # TF Lookups
        if age_tf_odom_base > 1.2:
            failures.append(f"TF odom->base_link lost (age: {age_tf_odom_base:.2f}s)")
        elif age_tf_odom_base > 0.4:
            degraded.append(f"TF odom->base_link delay (age: {age_tf_odom_base:.2f}s)")

        if age_tf_map_odom > 2.0:
            degraded.append(f"TF map->odom lost (age: {age_tf_map_odom:.2f}s)")

        # 4. Resolve Health State Machine
        new_state = "Healthy"
        reason = "All signals normal"
        comp = "None"
        age_val = 0.0
        th_val = 0.0

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

        # Log transition changes asynchronously
        if new_state != self.health_state:
            rospy.logwarn(f"[SafetySupervisor] Health Transition: {self.health_state} -> {new_state}. Reason: {reason}")
            log_data = (
                t_now,
                self.health_state,
                new_state,
                comp,
                age_val,
                th_val,
                reason[:200]
            )
            self.log_queue.put(("transition", log_data))
            self.health_state = new_state

        # Pack diagnostics metadata
        diagnostics = {
            "scan_age": age_scan,
            "scan_timeout": th_scan,
            "odom_age": age_odom,
            "odom_timeout": th_odom,
            "imu_age": age_imu,
            "imu_timeout": th_imu,
            "amcl_age": age_amcl,
            "amcl_timeout": th_amcl,
            "tf_map_odom_age": age_tf_map_odom,
            "tf_odom_base_age": age_tf_odom_base,
            "health_state": self.health_state,
            "risk_score": self.risk_score,
            "risk_level": self.risk_level
        }
        self.pub_diagnostics.publish(String(data=json.dumps(diagnostics)))

    def control_loop(self, event):
        t_now = time.time()
        out_msg = Twist()
        
        # 1. Update watchdogs and active health states
        self.check_watchdogs_and_state()
        
        # Default mode values
        self.active_layer = 0
        self.intervention_type = "NONE"
        self.speed_reduction = 0.0

        # If we are in Critical / E-stop, immediately publish zero command
        if self.health_state == "Critical":
            self.active_layer = 1
            self.intervention_type = "EMERGENCY_STOP"
            self.pub_cmd_vel.publish(out_msg)
            self.publish_status_and_markers()
            return

        # 2. Environment Sweeps (Clearances)
        self.calculate_clearances()

        # 3. Speed scaling boundaries from Health State Machine
        health_speed_scale = 1.0
        if self.health_state == "Degraded":
            health_speed_scale = 0.30
            self.active_layer = 1
            self.intervention_type = "SPEED_REDUCTION"
        elif self.health_state == "Warning":
            health_speed_scale = 0.75
            self.active_layer = 1
            self.intervention_type = "SPEED_REDUCTION"

        # 4. AMCL Localization Quality Checks
        loc_scale = 1.0
        if self.localization_health == "Lost":
            self.risk_score = 90.0
            self.risk_level = "Critical"
            self.active_layer = 2
            self.intervention_type = "EMERGENCY_STOP"
            self.pub_cmd_vel.publish(out_msg)
            self.publish_status_and_markers()
            return
        elif self.localization_health == "Poor":
            loc_scale = 0.35
            self.active_layer = 2
            self.intervention_type = "SPEED_REDUCTION"
        elif self.localization_health == "Warning":
            loc_scale = 0.60
            self.active_layer = 2
            self.intervention_type = "SPEED_REDUCTION"

        # 5. Environment clearance bubbles
        current_speed = abs(self.ekf_odom.twist.twist.linear.x) if self.ekf_odom else 0.0
        bubble_radius = self.bubble_base_radius + self.bubble_speed_coef * current_speed
        
        min_clearance = min(self.clearance_front, self.clearance_left, self.clearance_right, self.clearance_rear)
        if min_clearance < bubble_radius:
            self.active_layer = 8
            self.intervention_type = "SPEED_REDUCTION"

        # 6. Trajectory validation
        traj_ok = True
        if self.local_plan and self.local_costmap:
            resolution = self.local_costmap.info.resolution
            width = self.local_costmap.info.width
            height = self.local_costmap.info.height
            origin_x = self.local_costmap.info.origin.position.x
            origin_y = self.local_costmap.info.origin.position.y
            
            checked_dist = 0.0
            last_p = None
            for pose in self.local_plan.poses:
                p = pose.pose.position
                if last_p:
                    checked_dist += math.hypot(p.x - last_p.x, p.y - last_p.y)
                last_p = p
                if checked_dist > 1.5:
                    break
                
                grid_x = int((p.x - origin_x) / resolution)
                grid_y = int((p.y - origin_y) / resolution)
                
                if 0 <= grid_x < width and 0 <= grid_y < height:
                    cell_cost = self.local_costmap.data[grid_y * width + grid_x]
                    if cell_cost > 240:
                        traj_ok = False
                        self.active_layer = 4
                        self.intervention_type = "SPEED_REDUCTION"
                        break

        # 7. Predictive Footprint projections
        collision_predicted = False
        if self.cmd_raw and self.local_costmap:
            v_cmd = self.cmd_raw.linear.x
            w_cmd = self.cmd_raw.angular.z
            delta = math.atan2(self.wheelbase * w_cmd, abs(v_cmd)) if abs(v_cmd) > 0.01 else 0.0
            
            dt = 0.2
            sim_time = 2.0
            steps = int(sim_time / dt)
            
            resolution = self.local_costmap.info.resolution
            width = self.local_costmap.info.width
            height = self.local_costmap.info.height
            origin_x = self.local_costmap.info.origin.position.x
            origin_y = self.local_costmap.info.origin.position.y
            
            try:
                (trans, rot) = self.tf_listener.lookupTransform(self.local_costmap.header.frame_id, 'base_link', rospy.Time(0))
                x_sim, y_sim = trans[0], trans[1]
                yaw_sim = tf.transformations.euler_from_quaternion(rot)[2]
            except Exception:
                x_sim, y_sim, yaw_sim = 0.0, 0.0, 0.0
                
            for _ in range(steps):
                x_sim += v_cmd * math.cos(yaw_sim) * dt
                y_sim += v_cmd * math.sin(yaw_sim) * dt
                yaw_sim += (v_cmd * math.tan(delta) / self.wheelbase) * dt
                
                # Check corners
                corners = [
                    (x_sim + 0.6*math.cos(yaw_sim) - 0.425*math.sin(yaw_sim), y_sim + 0.6*math.sin(yaw_sim) + 0.425*math.cos(yaw_sim)),
                    (x_sim + 0.6*math.cos(yaw_sim) + 0.425*math.sin(yaw_sim), y_sim + 0.6*math.sin(yaw_sim) - 0.425*math.cos(yaw_sim)),
                    (x_sim - 0.6*math.cos(yaw_sim) - 0.425*math.sin(yaw_sim), y_sim - 0.6*math.sin(yaw_sim) + 0.425*math.cos(yaw_sim)),
                    (x_sim - 0.6*math.cos(yaw_sim) + 0.425*math.sin(yaw_sim), y_sim - 0.6*math.sin(yaw_sim) - 0.425*math.cos(yaw_sim))
                ]
                for cx, cy in corners:
                    gx = int((cx - origin_x) / resolution)
                    gy = int((cy - origin_y) / resolution)
                    if 0 <= gx < width and 0 <= gy < height:
                        if self.local_costmap.data[gy * width + gx] > 250:
                            collision_predicted = True
                            self.active_layer = 7
                            self.intervention_type = "EMERGENCY_STOP"
                            break
                if collision_predicted:
                    break

        # 8. Compute continuous Risk Score
        clearance_factor = max(0.0, 1.0 - min_clearance / 3.0)
        loc_factor = 1.0 - (loc_scale if self.localization_health != "Lost" else 0.0)
        traj_factor = 0.5 if not traj_ok else 0.0
        collision_factor = 1.0 if collision_predicted else 0.0
        
        self.risk_score = (0.35 * clearance_factor + 0.25 * loc_factor + 0.15 * traj_factor + 0.25 * collision_factor) * 100.0
        self.risk_score = max(0.0, min(100.0, self.risk_score))

        if self.risk_score >= 80.0 or collision_predicted:
            self.risk_level = "Critical"
        elif self.risk_score >= 55.0:
            self.risk_level = "High Risk"
        elif self.risk_score >= 35.0:
            self.risk_level = "Medium Risk"
        elif self.risk_score >= 15.0:
            self.risk_level = "Low Risk"
        else:
            self.risk_level = "Safe"

        # 9. Master Speed Scaling Mixer
        speed_scale = min(health_speed_scale, loc_scale)
        
        if min_clearance < 0.35:
            speed_scale = 0.0
        elif min_clearance < 1.2:
            speed_scale = min(speed_scale, (min_clearance - 0.35) / 0.85)

        corridor_width = self.clearance_left + self.clearance_right
        if corridor_width < 1.3:
            speed_scale = min(speed_scale, 0.25)

        if self.cmd_raw and abs(self.cmd_raw.angular.z) > 0.4:
            speed_scale = min(speed_scale, 0.50)

        if self.risk_level == "High Risk":
            speed_scale = min(speed_scale, 0.30)
        elif self.risk_level == "Critical":
            speed_scale = 0.0

        # Steering limits
        steer_scale = 1.0
        if min_clearance < 0.60:
            steer_scale = 0.50

        # Apply override
        if self.cmd_raw:
            out_msg.linear.x = self.cmd_raw.linear.x * speed_scale
            out_msg.angular.z = self.cmd_raw.angular.z * steer_scale
            if abs(self.cmd_raw.linear.x) > 0.01:
                self.speed_reduction = (1.0 - (abs(out_msg.linear.x) / abs(self.cmd_raw.linear.x))) * 100.0
        else:
            self.speed_reduction = 0.0

        if speed_scale == 0.0 and self.cmd_raw and abs(self.cmd_raw.linear.x) > 0.05:
            self.active_layer = 10
            self.intervention_type = "EMERGENCY_STOP"

        self.pub_cmd_vel.publish(out_msg)
        self.publish_status_and_markers()

    def publish_status_and_markers(self):
        # 1. Publish status string
        self.pub_status.publish(String(data=f"Risk:{self.risk_level} | ActiveLayer:{self.active_layer} | Intervene:{self.intervention_type}"))

        # 2. Build RViz markers array and offload to queue
        marker_arr = MarkerArray()
        
        text_marker = Marker()
        text_marker.header.frame_id = "base_link"
        text_marker.header.stamp = rospy.Time.now()
        text_marker.ns = "safety_status"
        text_marker.id = 100
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.pose.position.z = 1.6
        text_marker.scale.z = 0.22
        
        text_marker.text = f"Safety Supervisor\nHealth: {self.health_state}\nRisk: {self.risk_level} ({self.risk_score:.1f})\nLayer Active: L{self.active_layer}\nIntervention: {self.intervention_type}"
        
        if self.health_state == "Healthy":
            text_marker.color.r, text_marker.color.g, text_marker.color.b = 0.0, 1.0, 0.0
        elif self.health_state == "Warning":
            text_marker.color.r, text_marker.color.g, text_marker.color.b = 1.0, 1.0, 0.0
        elif self.health_state == "Degraded":
            text_marker.color.r, text_marker.color.g, text_marker.color.b = 1.0, 0.5, 0.0
        else: # Critical
            text_marker.color.r, text_marker.color.g, text_marker.color.b = 1.0, 0.0, 0.0
            
        text_marker.color.a = 1.0
        text_marker.lifetime = rospy.Duration(0.2)
        marker_arr.markers.append(text_marker)

        # Bubble size
        current_speed = abs(self.ekf_odom.twist.twist.linear.x) if self.ekf_odom else 0.0
        bubble_radius = self.bubble_base_radius + self.bubble_speed_coef * current_speed
        
        bubble_marker = Marker()
        bubble_marker.header.frame_id = "base_link"
        bubble_marker.header.stamp = rospy.Time.now()
        bubble_marker.ns = "safety_bubble"
        bubble_marker.id = 101
        bubble_marker.type = Marker.CYLINDER
        bubble_marker.action = Marker.ADD
        bubble_marker.scale.x = bubble_radius * 2
        bubble_marker.scale.y = bubble_radius * 2
        bubble_marker.scale.z = 0.02
        bubble_marker.pose.position.z = 0.01
        bubble_marker.color.r = text_marker.color.r
        bubble_marker.color.g = text_marker.color.g
        bubble_marker.color.b = text_marker.color.b
        bubble_marker.color.a = 0.15
        bubble_marker.lifetime = rospy.Duration(0.2)
        marker_arr.markers.append(bubble_marker)

        # Offload marker array publication to worker thread
        self.marker_queue.put(marker_arr)

    def logging_loop(self):
        if not self.ekf_odom:
            return
            
        x = self.ekf_odom.pose.pose.position.x
        y = self.ekf_odom.pose.pose.position.y
        q = [
            self.ekf_odom.pose.pose.orientation.x,
            self.ekf_odom.pose.pose.orientation.y,
            self.ekf_odom.pose.pose.orientation.z,
            self.ekf_odom.pose.pose.orientation.w
        ]
        yaw = tf.transformations.euler_from_quaternion(q)[2]
        speed = self.ekf_odom.twist.twist.linear.x
        w = self.ekf_odom.twist.twist.angular.z
        steering_angle = math.atan2(self.wheelbase * w, abs(speed)) if abs(speed) > 0.01 else 0.0
        
        data = (
            time.time(),
            x, y, yaw,
            speed, steering_angle,
            1.0 if self.localization_health in ["Excellent", "Good"] else 0.5,
            self.clearance_front, self.clearance_rear, self.clearance_left, self.clearance_right,
            self.risk_score, self.risk_level,
            self.active_layer, self.intervention_type, self.speed_reduction,
            self.health_state
        )
        self.log_queue.put(("event", data))

if __name__ == '__main__':
    try:
        node = SafetySupervisorNode()
        # Trigger logging loop at 1 Hz from main thread
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            node.logging_loop()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
