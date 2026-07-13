#!/usr/bin/env python3

import rospy
import sqlite3
import numpy as np
import tf
import math
import time
import threading
import queue
from std_msgs.msg import String, Header
from sensor_msgs.msg import LaserScan, PointCloud2
import sensor_msgs.point_cloud2 as pc2
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from geometry_msgs.msg import Twist, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_srvs.srv import Empty

class Track:
    def __init__(self, track_id, x, y, radius, class_name):
        self.track_id = track_id
        # State vector: [x, y, vx, vy]
        self.x = np.array([x, y, 0.0, 0.0])
        # Covariance matrix
        self.P = np.eye(4) * 0.5
        self.radius = radius
        self.class_name = class_name
        self.missed_frames = 0
        self.history = [] # list of (x, y, t)
        self.turn_rate = 0.0
        self.max_speed_observed = 0.0

    def predict(self, dt, q_val=0.15):
        # State transition F
        F = np.array([
            [1, 0, dt, 0],
            [0, 1, 0, dt],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
        self.x = np.dot(F, self.x)
        # Process noise Q
        Q = q_val * np.array([
            [dt**3/3.0, 0, dt**2/2.0, 0],
            [0, dt**3/3.0, 0, dt**2/2.0],
            [dt**2/2.0, 0, dt, 0],
            [0, dt**2/2.0, 0, dt]
        ])
        self.P = np.dot(np.dot(F, self.P), F.T) + Q

    def update(self, mx, my, r_val=0.1):
        # Measurement matrix H
        H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ])
        # Innovation
        z = np.array([mx, my])
        y_residual = z - np.dot(H, self.x)
        
        # Measurement noise R
        R = np.eye(2) * r_val
        # Innovation covariance S
        S = np.dot(np.dot(H, self.P), H.T) + R
        # Kalman gain K
        K = np.dot(np.dot(self.P, H.T), np.linalg.inv(S))
        
        # Update state and covariance
        self.x = self.x + np.dot(K, y_residual)
        self.P = np.dot(np.eye(4) - np.dot(K, H), self.P)
        self.missed_frames = 0
        
        speed = math.hypot(self.x[2], self.x[3])
        if speed > self.max_speed_observed:
            self.max_speed_observed = speed
        
        # Heading and turn rate estimation
        t_now = time.time()
        self.history.append((self.x[0], self.x[1], t_now))
        if len(self.history) > 15:
            self.history.pop(0)
            
        if len(self.history) >= 5:
            headings = []
            for i in range(1, len(self.history)):
                dx = self.history[i][0] - self.history[i-1][0]
                dy = self.history[i][1] - self.history[i-1][1]
                dt = self.history[i][2] - self.history[i-1][2]
                if math.hypot(dx, dy) > 0.05 and dt > 0.01:
                    headings.append(math.atan2(dy, dx))
            
            if len(headings) >= 2:
                # Average turn rate (radians/sec)
                diffs = [math.atan2(math.sin(headings[i] - headings[i-1]), math.cos(headings[i] - headings[i-1])) for i in range(1, len(headings))]
                self.turn_rate = sum(diffs) / len(diffs) * 20.0 # scale by frequency

class DynamicObstaclePlannerNode:
    def __init__(self):
        rospy.init_node('dynamic_obstacle_planner')
        
        self.data_lock = threading.Lock()
        
        # Parameters
        self.db_path = rospy.get_param('~db_path', '/home/mahboob-alam/four_wheel_drive/src/autocar_ai/database/fleet_learning.db')
        self.wheelbase = rospy.get_param('~wheelbase', 0.70)
        self.max_steer_angle = rospy.get_param('~max_steer_angle', 0.785398)
        self.max_detection_dist = rospy.get_param('~max_detection_dist', 8.0)
        
        # Multi-Target Tracker Cache
        self.tracks = []
        self.next_track_id = 1
        self.unmatched_detections = {} # centroid -> consecutive detections count
        
        # Sensor data caches
        self.ekf_odom = None
        self.local_plan = None
        self.global_plan = None
        self.local_costmap = None
        self.cmd_raw = None
        
        self.last_replan_time = 0.0
        
        # State machine
        self.behavior_state = "CONTINUE"
        self.overtaking_target_id = None
        self.overtaking_start_time = 0.0
        
        # Logging Queue
        self.log_queue = queue.Queue()
        self.db_init()
        self.worker_thread = threading.Thread(target=self.async_logging_worker)
        self.worker_thread.daemon = True
        self.worker_thread.start()
        
        # TF Listener
        self.tf_listener = tf.TransformListener()
        
        # Service Proxy for global replan request
        self.client_clear_costmaps = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
        
        # Publishers
        self.pub_cmd_safe = rospy.Publisher('/cmd_vel_raw_behavior', Twist, queue_size=1)
        self.pub_dynamic_obs = rospy.Publisher('/safety/dynamic_obstacles', PointCloud2, queue_size=5)
        self.pub_markers = rospy.Publisher('/safety/dynamic_markers', MarkerArray, queue_size=5)
        
        # Subscribers
        self.sub_laser = rospy.Subscriber('/scan_deskewed', LaserScan, self.laser_callback, queue_size=1)
        self.sub_odom = rospy.Subscriber('/odometry/filtered', Odometry, self.odom_callback, queue_size=1)
        self.sub_local_plan = rospy.Subscriber('/move_base/TebLocalPlannerROS/local_plan', Path, self.local_plan_callback, queue_size=1)
        self.sub_global_plan = rospy.Subscriber('/move_base/GlobalPlanner/plan', Path, self.global_plan_callback, queue_size=1)
        self.sub_costmap = rospy.Subscriber('/move_base/local_costmap/costmap', OccupancyGrid, self.costmap_callback, queue_size=1)
        self.sub_cmd_raw = rospy.Subscriber('/cmd_vel_raw', Twist, self.cmd_raw_callback, queue_size=1)
        
        rospy.loginfo("[DynamicObstaclePlanner] Dynamic Obstacle Prediction and Behavior Planner initialized.")

    def db_init(self):
        try:
            conn = sqlite3.connect(self.db_path)
            cursor = conn.cursor()
            cursor.execute("""
            CREATE TABLE IF NOT EXISTS dynamic_obstacle_events (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                timestamp REAL,
                object_id INTEGER,
                object_class TEXT,
                object_x REAL, object_y REAL,
                object_vx REAL, object_vy REAL,
                ttc REAL,
                behavior_decision TEXT,
                near_miss INTEGER,
                event_type TEXT
            )
            """)
            conn.commit()
            conn.close()
        except Exception as e:
            rospy.logerr(f"[DynamicObstaclePlanner] DB Init failed: {e}")

    def async_logging_worker(self):
        while not rospy.is_shutdown():
            try:
                while not self.log_queue.empty():
                    data = self.log_queue.get_nowait()
                    conn = sqlite3.connect(self.db_path)
                    cursor = conn.cursor()
                    cursor.execute("""
                    INSERT INTO dynamic_obstacle_events (
                        timestamp, object_id, object_class, object_x, object_y, object_vx, object_vy,
                        ttc, behavior_decision, near_miss, event_type
                    ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                    """, data)
                    conn.commit()
                    conn.close()
            except Exception:
                pass
            time.sleep(0.1)

    def laser_callback(self, msg):
        self.process_laser_scan(msg)

    def odom_callback(self, msg):
        with self.data_lock:
            self.ekf_odom = msg

    def local_plan_callback(self, msg):
        with self.data_lock:
            self.local_plan = msg

    def global_plan_callback(self, msg):
        with self.data_lock:
            self.global_plan = msg

    def costmap_callback(self, msg):
        with self.data_lock:
            self.local_costmap = msg

    def cmd_raw_callback(self, msg):
        with self.data_lock:
            self.cmd_raw = msg

    def process_laser_scan(self, msg):
        t_now = time.time()
        
        # 1. Parse Scan Ranges to 2D Points in base_link
        ranges = np.array(msg.ranges)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
        valid_idx = np.where((ranges > 0.05) & (ranges < self.max_detection_dist))
        if len(valid_idx[0]) == 0:
            self.update_tracks([], t_now)
            return

        ranges = ranges[valid_idx]
        angles = angles[valid_idx]
        
        lx = ranges * np.cos(angles)
        ly = ranges * np.sin(angles)
        
        # 2. Transform scan points to odom frame to track in fixed coordinates
        try:
            self.tf_listener.waitForTransform('odom', msg.header.frame_id, msg.header.stamp, rospy.Duration(0.1))
            (trans, rot) = self.tf_listener.lookupTransform('odom', msg.header.frame_id, msg.header.stamp)
            mat = tf.transformations.quaternion_matrix(rot)
        except Exception:
            self.update_tracks([], t_now)
            return
            
        pts_odom = []
        for i in range(len(lx)):
            p_orig = [lx[i], ly[i], 0.0]
            p_trans = np.dot(mat[:3, :3], p_orig) + trans
            pts_odom.append([p_trans[0], p_trans[1]])
        
        pts_odom = np.array(pts_odom)

        # 3. Simple Euclidean Clustering in odom frame
        clusters = []
        visited = np.zeros(len(pts_odom), dtype=bool)
        for i in range(len(pts_odom)):
            if visited[i]:
                continue
            dists = np.linalg.norm(pts_odom - pts_odom[i], axis=1)
            in_cluster = dists < 0.45
            cluster_indices = np.where(in_cluster)[0]
            
            q = list(cluster_indices)
            cluster = []
            while len(q) > 0:
                idx = q.pop(0)
                if visited[idx]:
                    continue
                visited[idx] = True
                cluster.append(pts_odom[idx])
                
                # Check neighbors of neighbors
                d = np.linalg.norm(pts_odom - pts_odom[idx], axis=1)
                neighbors = np.where(d < 0.45)[0]
                for n in neighbors:
                    if not visited[n]:
                        q.append(n)
                        
            if 3 <= len(cluster) <= 80:
                clusters.append(np.array(cluster))

        # 4. Form Detections: centroid, radius, classification
        detections = []
        for cluster in clusters:
            centroid = np.mean(cluster, axis=0)
            max_dist = np.max(np.linalg.norm(cluster - centroid, axis=1))
            radius = max(0.15, max_dist)
            
            # Simple size-based classification
            if radius < 0.35:
                cls_name = "Pedestrian"
            elif radius < 0.70:
                cls_name = "Bicycle"
            elif radius < 1.25:
                cls_name = "Forklift"
            else:
                cls_name = "Vehicle"
                
            detections.append((centroid[0], centroid[1], radius, cls_name))

        # 5. Multi-Target Association and Kalman Update
        self.update_tracks(detections, t_now)

    def update_tracks(self, detections, t_now):
        dt = 0.04 # nominal 25Hz loop

        # A. Predict existing tracks
        for track in self.tracks:
            track.predict(dt)

        # B. Greedy nearest-neighbor association
        matched_detections = set()
        matched_tracks = set()
        
        associations = []
        for d_idx, (dx, dy, dr, dcls) in enumerate(detections):
            best_t_idx = -1
            best_dist = 1.2 # gating distance threshold (meters)
            for t_idx, track in enumerate(self.tracks):
                if t_idx in matched_tracks:
                    continue
                dist = math.hypot(dx - track.x[0], dy - track.x[1])
                if dist < best_dist:
                    best_dist = dist
                    best_t_idx = t_idx
            
            if best_t_idx != -1:
                associations.append((best_t_idx, d_idx))
                matched_tracks.add(best_t_idx)
                matched_detections.add(d_idx)

        # C. Update matched tracks
        for t_idx, d_idx in associations:
            track = self.tracks[t_idx]
            dx, dy, dr, dcls = detections[d_idx]
            track.update(dx, dy)
            track.radius = dr
            track.class_name = dcls

        # D. Handle unmatched tracks (occlusion/missed detection)
        unmatched_t_indices = [i for i in range(len(self.tracks)) if i not in matched_tracks]
        for idx in unmatched_t_indices:
            track = self.tracks[idx]
            track.missed_frames += 1

        # Delete expired tracks
        self.tracks = [track for track in self.tracks if track.missed_frames < 38]

        # E. Initialize new tracks for unmatched detections (requires persistence check)
        for d_idx, (dx, dy, dr, dcls) in enumerate(detections):
            if d_idx in matched_detections:
                continue
            
            # Persistence tracking: must see unmatched detection near coordinates for 3 frames
            initialized = False
            for old_coord, count in list(self.unmatched_detections.items()):
                dist = math.hypot(dx - old_coord[0], dy - old_coord[1])
                if dist < 0.6:
                    self.unmatched_detections.pop(old_coord)
                    new_count = count + 1
                    if new_count >= 3:
                        # Initialize track
                        new_track = Track(self.next_track_id, dx, dy, dr, dcls)
                        self.tracks.append(new_track)
                        self.next_track_id += 1
                        initialized = True
                    else:
                        self.unmatched_detections[(dx, dy)] = new_count
                        initialized = True
                    break
            
            if not initialized:
                self.unmatched_detections[(dx, dy)] = 1

        # Run Behavior Loop
        self.behavior_planner_step(t_now)

    def predict_obstacle_trajectory(self, track, steps=12, dt=0.25):
        """Predicts the obstacle's trajectory using CTRV or Constant Velocity model."""
        x0 = track.x[0]
        y0 = track.x[1]
        vx = track.x[2]
        vy = track.x[3]
        
        speed = math.hypot(vx, vy)
        heading = math.atan2(vy, vx) if speed > 0.05 else 0.0
        
        trajectory = []
        
        # Use CTRV if turn rate is significant, otherwise CV
        if speed > 0.1 and abs(track.turn_rate) > 0.08:
            for step in range(1, steps + 1):
                t = step * dt
                w = track.turn_rate
                # CTRV Equations
                new_heading = heading + w * t
                new_x = x0 + (speed / w) * (math.sin(heading + w * t) - math.sin(heading))
                new_y = y0 - (speed / w) * (math.cos(heading + w * t) - math.cos(heading))
                trajectory.append((new_x, new_y, new_heading))
        else:
            # Constant Velocity (CV) model
            for step in range(1, steps + 1):
                t = step * dt
                new_x = x0 + vx * t
                new_y = y0 + vy * t
                trajectory.append((new_x, new_y, heading))
                
        return trajectory

    def simulate_ego_rollout(self, nominal_v, nominal_w, steps=12, dt=0.25):
        """Predicts ego trajectory using Ackermann kinematics."""
        try:
            (trans, rot) = self.tf_listener.lookupTransform('odom', 'base_link', rospy.Time(0))
            x, y = trans[0], trans[1]
            yaw = tf.transformations.euler_from_quaternion(rot)[2]
        except Exception:
            return []

        trajectory = []
        steer = math.atan2(nominal_w * self.wheelbase, abs(nominal_v)) if abs(nominal_v) > 0.01 else 0.0
        
        for _ in range(steps):
            x += nominal_v * math.cos(yaw) * dt
            y += nominal_v * math.sin(yaw) * dt
            yaw += (nominal_v * math.tan(steer) / self.wheelbase) * dt
            trajectory.append((x, y, yaw))
            
        return trajectory

    def behavior_planner_step(self, t_now):
        with self.data_lock:
            cmd_raw = self.cmd_raw
            ekf_odom = self.ekf_odom
            local_plan = self.local_plan
            global_plan = self.global_plan

        out_msg = Twist()
        if cmd_raw:
            out_msg.linear.x = cmd_raw.linear.x
            out_msg.angular.z = cmd_raw.angular.z

        # Default commands
        nominal_v = cmd_raw.linear.x if cmd_raw else 0.0
        nominal_w = cmd_raw.angular.z if cmd_raw else 0.0

        # Sim steps parameter setup
        sim_dt = 0.25
        steps = 12 # 3.0s prediction horizon

        # 1. Predict Ego vehicle trajectory
        ego_rollout = self.simulate_ego_rollout(nominal_v, nominal_w, steps=steps, dt=sim_dt)

        # 2. Collision Prediction and Pedestrian Crossing check
        self.behavior_state = "CONTINUE"
        min_ttc = 999.0
        closest_obstacle_id = None
        closest_obstacle_class = "Unknown"
        closest_obstacle_pose = (0.0, 0.0)
        closest_obstacle_vel = (0.0, 0.0)
        pedestrian_crossing = False
        near_miss = 0

        # Costmap points to publish
        dynamic_obs_points = []
        guide_obstacle_points = []

        # Coordinate transformation for checking local paths
        try:
            (trans_ego, rot_ego) = self.tf_listener.lookupTransform('base_link', 'odom', rospy.Time(0))
            mat_ego = tf.transformations.quaternion_matrix(rot_ego)
        except Exception:
            trans_ego, rot_ego = None, None

        for track in self.tracks:
            # Skip if the object is static (like walls/shelves) or currently stationary
            speed = math.hypot(track.x[2], track.x[3])
            if speed < 0.25:
                continue
            # Also ignore wall/shelf clusters which are excessively large
            if track.radius > 1.3:
                continue

            # Predict trajectory of the dynamic obstacle
            obs_traj = self.predict_obstacle_trajectory(track, steps=steps, dt=sim_dt)
            
            for step_idx, (ox, oy, oyaw) in enumerate(obs_traj):
                # Calculate size/inflation proportional to speed and uncertainty
                inflation = 0.25 + speed * 0.15 + (step_idx * 0.02)
                # Generate a ring of points representing dynamic costmap footprint
                num_ring_points = 8
                for j in range(num_ring_points):
                    ang = j * (2 * math.pi / num_ring_points)
                    px = ox + inflation * math.cos(ang)
                    py = oy + inflation * math.sin(ang)
                    dynamic_obs_points.append([px, py, 0.02])

            # Compare footprints along projected trajectories to detect collisions
            for idx in range(min(len(ego_rollout), len(obs_traj))):
                ex, ey, eyaw = ego_rollout[idx]
                ox, oy, oyaw = obs_traj[idx]
                
                dist = math.hypot(ex - ox, ey - oy)
                # Bounding collision boundary check (ego footprint + obstacle radius + buffer)
                collision_boundary = 0.65 + track.radius
                
                if dist < collision_boundary:
                    ttc = idx * sim_dt
                    if ttc < min_ttc:
                        min_ttc = ttc
                        closest_obstacle_id = track.track_id
                        closest_obstacle_class = track.class_name
                        closest_obstacle_pose = (track.x[0], track.x[1])
                        closest_obstacle_vel = (track.x[2], track.x[3])
                    
                    if ttc < 1.0:
                        near_miss = 1
                    break

            # Pedestrian priority checking: if pedestrian is in front of the vehicle
            if track.class_name == "Pedestrian" and trans_ego is not None:
                # Transform track to base_link frame
                p_odom = [track.x[0], track.x[1], 0.0]
                p_base = np.dot(mat_ego[:3, :3], p_odom) + trans_ego
                # If pedestrian is within a 2.5m wide corridor in front of the vehicle up to 4.5m
                if 0.0 < p_base[0] < 4.5 and -1.2 < p_base[1] < 1.2:
                    pedestrian_crossing = True

        # 3. Behavior Decision Tree
        speed_scale = 1.0
        event_type = "NONE"

        if pedestrian_crossing:
            self.behavior_state = "YIELD"
            speed_scale = 0.0
            event_type = "YIELD"
        elif min_ttc < 0.6:
            self.behavior_state = "EMERGENCY_STOP"
            speed_scale = 0.0
            event_type = "STOP"
        elif min_ttc < 1.2:
            self.behavior_state = "STOP"
            speed_scale = 0.0
            event_type = "STOP"
        elif min_ttc < 2.5:
            self.behavior_state = "CRAWL"
            speed_scale = 0.25
            event_type = "SLOW_DOWN"
        elif min_ttc < 4.5:
            self.behavior_state = "SLOW_DOWN"
            speed_scale = 0.60
            event_type = "SLOW_DOWN"
        else:
            # Check Overtaking Logic
            # If there is a slow obstacle blocking our path
            blocking_obstacle = None
            if trans_ego is not None:
                for track in self.tracks:
                    speed = math.hypot(track.x[2], track.x[3])
                    # If obstacle is slow, close (<3.0m), and in front
                    p_odom = [track.x[0], track.x[1], 0.0]
                    p_base = np.dot(mat_ego[:3, :3], p_odom) + trans_ego
                    if speed < 0.35 and track.max_speed_observed >= 0.35 and 0.5 < p_base[0] < 3.2 and -0.7 < p_base[1] < 0.7:
                        blocking_obstacle = track
                        break
            
            if blocking_obstacle is not None:
                # Overtaking checks: verify adjacent left space in the local costmap
                left_space_free = True
                if self.local_costmap:
                    # Check costmap coordinates on the left side of the robot (base_link y between 0.8 and 1.8)
                    res = self.local_costmap.info.resolution
                    w = self.local_costmap.info.width
                    h = self.local_costmap.info.height
                    ox_cm = self.local_costmap.info.origin.position.x
                    oy_cm = self.local_costmap.info.origin.position.y
                    data_cm = self.local_costmap.data
                    
                    try:
                        (trans_cm, rot_cm) = self.tf_listener.lookupTransform(self.local_costmap.header.frame_id, 'base_link', rospy.Time(0))
                        # Check left side points
                        for dy in np.linspace(0.8, 1.8, 5):
                            for dx in np.linspace(-0.5, 2.0, 6):
                                # Transform back to costmap frame
                                cx = trans_cm[0] + dx * math.cos(tf.transformations.euler_from_quaternion(rot_cm)[2]) - dy * math.sin(tf.transformations.euler_from_quaternion(rot_cm)[2])
                                cy = trans_cm[1] + dx * math.sin(tf.transformations.euler_from_quaternion(rot_cm)[2]) + dy * math.cos(tf.transformations.euler_from_quaternion(rot_cm)[2])
                                gx = int((cx - ox_cm) / res)
                                gy = int((cy - oy_cm) / res)
                                if 0 <= gx < w and 0 <= gy < h:
                                    if data_cm[gy * w + gx] > 90:
                                        left_space_free = False
                                        break
                            if not left_space_free:
                                break
                    except Exception:
                        left_space_free = False

                if left_space_free:
                    self.behavior_state = "OVERTAKE"
                    self.overtaking_target_id = blocking_obstacle.track_id
                    event_type = "OVERTAKE"
                    
                    # Generate a temporary overtaking corridor:
                    # Place virtual costmap obstacles to the right of the obstacle (forcing TEB to go left)
                    # Coordinates of virtual obstacles: centered on right side of obstacle in odom
                    obs_x, obs_y = blocking_obstacle.x[0], blocking_obstacle.x[1]
                    vx, vy = blocking_obstacle.x[2], blocking_obstacle.x[3]
                    heading = math.atan2(vy, vx) if math.hypot(vx, vy) > 0.05 else 0.0
                    
                    # Add point-cloud points on the right of the obstacle
                    perp_angle = heading - math.pi / 2.0
                    for d_offset in np.linspace(0.4, 1.5, 4):
                        rx = obs_x + d_offset * math.cos(perp_angle)
                        ry = obs_y + d_offset * math.sin(perp_angle)
                        # Append to points list to force TEB to steer left
                        for j in range(4):
                            ang = j * (math.pi / 2)
                            guide_obstacle_points.append([rx + 0.1 * math.cos(ang), ry + 0.1 * math.sin(ang), 0.02])
                else:
                    self.behavior_state = "WAIT"
                    speed_scale = 0.0
                    event_type = "YIELD"

        # Apply speed scale
        if cmd_raw:
            out_msg.linear.x = cmd_raw.linear.x * speed_scale
            # Scale w proportionally to preserve curvature
            if abs(cmd_raw.linear.x) > 0.01:
                out_msg.angular.z = cmd_raw.angular.z * speed_scale
            else:
                out_msg.angular.z = cmd_raw.angular.z

        # Publish the command to safety supervisor intercept topic
        self.pub_cmd_safe.publish(out_msg)

        # 4. Publish temporary dynamic obstacles PointCloud2
        all_points = dynamic_obs_points + guide_obstacle_points
        if len(all_points) > 0:
            header = Header()
            header.stamp = rospy.Time.now()
            header.frame_id = "odom"
            cloud_msg = pc2.create_cloud_xyz32(header, all_points)
            self.pub_dynamic_obs.publish(cloud_msg)
        else:
            # Publish empty cloud to clear
            header = Header()
            header.stamp = rospy.Time.now()
            header.frame_id = "odom"
            cloud_msg = pc2.create_cloud_xyz32(header, [])
            self.pub_dynamic_obs.publish(cloud_msg)

        # 5. SQLite Logging (triggered on state transitions/critical behavior events)
        if event_type != "NONE" and closest_obstacle_id is not None:
            log_data = (
                time.time(),
                closest_obstacle_id,
                closest_obstacle_class,
                closest_obstacle_pose[0], closest_obstacle_pose[1],
                closest_obstacle_vel[0], closest_obstacle_vel[1],
                min_ttc,
                self.behavior_state,
                near_miss,
                event_type
            )
            self.log_queue.put(log_data)

        # 6. Publish Visualizations
        self.publish_visualizations(ego_rollout, min_ttc, closest_obstacle_id)

    def publish_visualizations(self, ego_rollout, min_ttc, closest_obstacle_id):
        marker_arr = MarkerArray()
        t_now = rospy.Time.now()

        # A. Text Diagnostics Marker
        text_marker = Marker()
        text_marker.header.frame_id = "base_link"
        text_marker.header.stamp = t_now
        text_marker.ns = "behavior_diagnostics"
        text_marker.id = 500
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.pose.position.z = 1.9
        text_marker.scale.z = 0.22
        
        ttc_str = f"{min_ttc:.2f}s" if min_ttc < 100.0 else "N/A"
        text_marker.text = (
            f"Behavior Planner\n"
            f"State: {self.behavior_state}\n"
            f"TTC: {ttc_str}\n"
            f"Obstacles Tracked: {len(self.tracks)}"
        )
        if self.behavior_state in ["YIELD", "STOP", "EMERGENCY_STOP", "WAIT"]:
            text_marker.color.r, text_marker.color.g, text_marker.color.b = 1.0, 0.0, 0.0
        elif self.behavior_state in ["SLOW_DOWN", "CRAWL", "OVERTAKE"]:
            text_marker.color.r, text_marker.color.g, text_marker.color.b = 1.0, 0.5, 0.0
        else:
            text_marker.color.r, text_marker.color.g, text_marker.color.b = 0.0, 1.0, 0.0
        text_marker.color.a = 1.0
        text_marker.lifetime = rospy.Duration(0.1)
        marker_arr.markers.append(text_marker)

        # B. Tracked Objects cylinders
        for track in self.tracks:
            # Sphere/cylinder representation
            obj_marker = Marker()
            obj_marker.header.frame_id = "odom"
            obj_marker.header.stamp = t_now
            obj_marker.ns = "tracks"
            obj_marker.id = 600 + track.track_id
            obj_marker.type = Marker.CYLINDER
            obj_marker.action = Marker.ADD
            obj_marker.pose.position.x = track.x[0]
            obj_marker.pose.position.y = track.x[1]
            obj_marker.pose.position.z = 0.4
            obj_marker.scale.x = track.radius * 2
            obj_marker.scale.y = track.radius * 2
            obj_marker.scale.z = 0.8
            
            # Colors based on class
            if track.class_name == "Pedestrian":
                obj_marker.color.r, obj_marker.color.g, obj_marker.color.b = 1.0, 0.8, 0.2
            elif track.class_name == "Bicycle":
                obj_marker.color.r, obj_marker.color.g, obj_marker.color.b = 0.2, 0.8, 1.0
            elif track.class_name == "Forklift":
                obj_marker.color.r, obj_marker.color.g, obj_marker.color.b = 1.0, 0.2, 1.0
            else:
                obj_marker.color.r, obj_marker.color.g, obj_marker.color.b = 0.5, 0.5, 0.5
            obj_marker.color.a = 0.6
            obj_marker.lifetime = rospy.Duration(0.1)
            marker_arr.markers.append(obj_marker)

            # Class text marker
            lbl = Marker()
            lbl.header.frame_id = "odom"
            lbl.header.stamp = t_now
            lbl.ns = "track_labels"
            lbl.id = 700 + track.track_id
            lbl.type = Marker.TEXT_VIEW_FACING
            lbl.action = Marker.ADD
            lbl.pose.position.x = track.x[0]
            lbl.pose.position.y = track.x[1]
            lbl.pose.position.z = 1.0
            lbl.scale.z = 0.16
            
            speed = math.hypot(track.x[2], track.x[3])
            lbl.text = f"ID:{track.track_id} {track.class_name}\n({speed:.1f} m/s)"
            lbl.color.r, lbl.color.g, lbl.color.b = 1.0, 1.0, 1.0
            lbl.color.a = 0.9
            lbl.lifetime = rospy.Duration(0.1)
            marker_arr.markers.append(lbl)

            # Velocity vector arrow
            if speed > 0.05:
                vel_marker = Marker()
                vel_marker.header.frame_id = "odom"
                vel_marker.header.stamp = t_now
                vel_marker.ns = "velocity_arrows"
                vel_marker.id = 800 + track.track_id
                vel_marker.type = Marker.ARROW
                vel_marker.action = Marker.ADD
                vel_marker.scale.x = 0.05
                vel_marker.scale.y = 0.1
                vel_marker.scale.z = 0.1
                vel_marker.color.r, vel_marker.color.g, vel_marker.color.b = 0.0, 1.0, 1.0
                vel_marker.color.a = 0.8
                vel_marker.lifetime = rospy.Duration(0.1)
                
                p_start = Point(x=track.x[0], y=track.x[1], z=0.1)
                p_end = Point(x=track.x[0] + track.x[2], y=track.x[1] + track.x[3], z=0.1)
                vel_marker.points.append(p_start)
                vel_marker.points.append(p_end)
                marker_arr.markers.append(vel_marker)

            # Predicted trajectory line strip
            pred_traj = self.predict_obstacle_trajectory(track)
            if len(pred_traj) > 0:
                line = Marker()
                line.header.frame_id = "odom"
                line.header.stamp = t_now
                line.ns = "predicted_trajectories"
                line.id = 900 + track.track_id
                line.type = Marker.LINE_STRIP
                line.action = Marker.ADD
                line.scale.x = 0.04
                line.color.r, line.color.g, line.color.b = 0.0, 0.8, 1.0
                line.color.a = 0.4
                line.lifetime = rospy.Duration(0.1)
                
                line.points.append(Point(x=track.x[0], y=track.x[1], z=0.03))
                for px, py, _ in pred_traj:
                    line.points.append(Point(x=px, y=py, z=0.03))
                marker_arr.markers.append(line)

        # C. Ego kinematic rollout trajectory
        if len(ego_rollout) > 0:
            rollout_marker = Marker()
            rollout_marker.header.frame_id = "odom"
            rollout_marker.header.stamp = t_now
            rollout_marker.ns = "ego_projected_rollout"
            rollout_marker.id = 501
            rollout_marker.type = Marker.LINE_STRIP
            rollout_marker.action = Marker.ADD
            rollout_marker.scale.x = 0.05
            rollout_marker.color.r, rollout_marker.color.g, rollout_marker.color.b = 1.0, 0.8, 0.0
            rollout_marker.color.a = 0.5
            rollout_marker.lifetime = rospy.Duration(0.1)
            
            for ex, ey, _ in ego_rollout:
                rollout_marker.points.append(Point(x=ex, y=ey, z=0.04))
            marker_arr.markers.append(rollout_marker)

        self.pub_markers.publish(marker_arr)

if __name__ == '__main__':
    try:
        node = DynamicObstaclePlannerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
