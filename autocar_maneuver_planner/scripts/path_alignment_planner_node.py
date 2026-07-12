#!/usr/bin/env python3
import rospy
import math
import numpy as np
import sqlite3
import tf
from geometry_msgs.msg import Twist, PoseStamped, Point, Quaternion, Pose
from nav_msgs.msg import Path, OccupancyGrid
from nav_msgs.srv import GetPlan, GetPlanRequest
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String, Header
import actionlib
from autocar_interfaces.msg import NavigateBehaviorAction, NavigateBehaviorGoal, NavigateBehaviorFeedback, NavigateBehaviorResult
import rospkg
import os

class TrajectoryPoint:
    def __init__(self, x, y, yaw, v, steer):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.steer = steer

class PathAlignmentPlannerNode:
    def __init__(self):
        rospy.init_node('path_alignment_planner_node')

        # Load database path
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('autocar_ai')
        self.db_path = os.path.join(pkg_path, 'database', 'fleet_learning.db')
        self.init_database()

        # Vehicle parameters
        self.wheelbase = 0.70  # meters
        self.width = 0.85
        self.length = 1.1
        self.max_steer = 0.785  # radians (45 degrees)
        self.min_turn_radius = 1.0  # meters (for comfortable maneuvers)
        
        # Constraints
        self.v_max_forward = 0.18  # m/s (reduced from 0.35 to prevent localization loss)
        self.v_max_reverse = -0.12  # m/s (reduced from -0.25)
        self.accel_max = 0.4  # m/s^2
        self.steer_rate_max = 1.0  # rad/s

        # Footprint definitions relative to base_link with safety padding
        self.safety_padding = 0.12  # meters of extra safety margin
        self.footprint_local = [
            np.array([-0.6 - self.safety_padding, -0.35 - self.safety_padding]),
            np.array([-0.6 - self.safety_padding, 0.35 + self.safety_padding]),
            np.array([0.6 + self.safety_padding, 0.35 + self.safety_padding]),
            np.array([0.6 + self.safety_padding, -0.35 - self.safety_padding])
        ]
        
        # Heading error threshold for alignment (15 degrees)
        self.alignment_threshold = math.radians(15.0)

        # TF Listener
        self.listener = tf.TransformListener()

        # Costmap caching
        self.costmap = None
        self.sub_costmap = rospy.Subscriber('/move_base/local_costmap/costmap', OccupancyGrid, self.costmap_callback)

        # Publishers
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.pub_markers = rospy.Publisher('/alignment_planner/markers', MarkerArray, queue_size=10)

        # Action Client to forward to the original BDM
        self.client_bdm = actionlib.SimpleActionClient('navigate_behavior_bdm', NavigateBehaviorAction)
        
        # Action Server to intercept goals
        self.server = actionlib.SimpleActionServer('navigate_behavior', NavigateBehaviorAction, self.execute_cb, auto_start=False)
        self.server.start()

        rospy.loginfo("[PathAlignmentPlanner] Node initialized and ready.")

    def init_database(self):
        try:
            conn = sqlite3.connect(self.db_path)
            cursor = conn.cursor()
            cursor.execute("""
            CREATE TABLE IF NOT EXISTS alignment_maneuver_logs (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                timestamp REAL,
                initial_x REAL,
                initial_y REAL,
                initial_yaw REAL,
                initial_heading REAL,
                path_heading REAL,
                selected_maneuver TEXT,
                candidate_costs TEXT,
                planning_time REAL,
                execution_time REAL,
                final_x REAL,
                final_y REAL,
                final_yaw REAL,
                final_orientation_error REAL,
                success INTEGER,
                failure_reason TEXT
            )""")
            conn.commit()
            conn.close()
        except Exception as e:
            rospy.logwarn(f"[PathAlignmentPlanner] SQLite database initialization failed: {e}")

    def costmap_callback(self, msg):
        self.costmap = msg

    def get_robot_pose(self):
        try:
            self.listener.waitForTransform('map', 'base_link', rospy.Time(0), rospy.Duration(1.0))
            trans, rot = self.listener.lookupTransform('map', 'base_link', rospy.Time(0))
            yaw = tf.transformations.euler_from_quaternion(rot)[2]
            return trans[0], trans[1], yaw
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn(f"[PathAlignmentPlanner] TF Lookup failed: {e}")
            return None

    def query_global_plan(self, start_pose, goal_pose):
        rospy.wait_for_service('/move_base/make_plan', timeout=5.0)
        try:
            make_plan = rospy.ServiceProxy('/move_base/make_plan', GetPlan)
            req = GetPlanRequest()
            req.start.header.frame_id = 'map'
            req.start.header.stamp = rospy.Time.now()
            req.start.pose = start_pose
            req.goal.header.frame_id = 'map'
            req.goal.header.stamp = rospy.Time.now()
            req.goal.pose = goal_pose
            
            # Use small tolerance
            req.tolerance = 0.5
            resp = make_plan(req)
            return resp.plan
        except rospy.ServiceException as e:
            rospy.logerr(f"[PathAlignmentPlanner] make_plan service call failed: {e}")
            return None

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def get_path_start_heading(self, plan):
        if not plan or len(plan.poses) < 2:
            return 0.0
        
        # Look ahead 0.5 - 1.0 meters in the plan to get heading vector
        start_x = plan.poses[0].pose.position.x
        start_y = plan.poses[0].pose.position.y
        
        target_idx = 1
        for i in range(1, len(plan.poses)):
            dx = plan.poses[i].pose.position.x - start_x
            dy = plan.poses[i].pose.position.y - start_y
            dist = math.sqrt(dx*dx + dy*dy)
            if dist >= 0.5:
                target_idx = i
                break
                
        dx = plan.poses[target_idx].pose.position.x - start_x
        dy = plan.poses[target_idx].pose.position.y - start_y
        return math.atan2(dy, dx)

    def is_in_collision(self, x, y, yaw):
        if not self.costmap:
            return False  # Assume no collision if costmap not received yet

        costmap_frame = self.costmap.header.frame_id

        # Extract costmap properties
        res = self.costmap.info.resolution
        origin_x = self.costmap.info.origin.position.x
        origin_y = self.costmap.info.origin.position.y
        width = self.costmap.info.width
        height = self.costmap.info.height
        data = self.costmap.data

        # Generate footprint vertices in map frame
        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)
        
        # Check center, 4 vertices, and midpoints (9 points total)
        check_points = [np.array([0.0, 0.0])]
        for pt in self.footprint_local:
            check_points.append(pt)
            check_points.append(pt * 0.5)  # midpoints

        for pt in check_points:
            px_map = x + pt[0] * cos_y - pt[1] * sin_y
            py_map = y + pt[0] * sin_y + pt[1] * cos_y

            # Transform to costmap frame if different
            if costmap_frame != 'map':
                pose_map = PoseStamped()
                pose_map.header.frame_id = 'map'
                pose_map.header.stamp = rospy.Time(0)
                pose_map.pose.position.x = px_map
                pose_map.pose.position.y = py_map
                pose_map.pose.orientation.w = 1.0
                try:
                    pose_costmap = self.listener.transformPose(costmap_frame, pose_map)
                    px = pose_costmap.pose.position.x
                    py = pose_costmap.pose.position.y
                except Exception as e:
                    rospy.logwarn_throttle(5.0, f"[PathAlignmentPlanner] TF transform to costmap frame failed: {e}")
                    px, py = px_map, py_map
            else:
                px, py = px_map, py_map

            # Convert to costmap coords
            cell_x = int((px - origin_x) / res)
            cell_y = int((py - origin_y) / res)

            if cell_x < 0 or cell_x >= width or cell_y < 0 or cell_y >= height:
                # If outside local costmap bounds, check if it's too far from robot center
                continue

            index = cell_y * width + cell_x
            cost = data[index]
            
            # In OccupancyGrid: 100 is LETHAL, -1 is UNKNOWN, 99 is INSCRIBED
            if cost >= 99 or cost == -1:
                return True
        return False

    def get_velocity_limit(self, steer, forward=True):
        # Scale speed down as steering angle increases to reduce slippage and localization error
        max_v = self.v_max_forward if forward else self.v_max_reverse
        scale = 1.0 - 0.45 * (abs(steer) / self.max_steer)
        return max_v * scale

    def generate_forward_alignment(self, x0, y0, yaw0, target_yaw, target_x, target_y, steer_dir):
        # Steer and move forward
        traj = []
        x, y, yaw = x0, y0, yaw0
        dt = 0.1
        
        # Phase 1: Turn forward
        for _ in range(60):  # 6 seconds max
            steer = steer_dir * 0.45
            v = self.get_velocity_limit(steer, forward=True)
            x += v * math.cos(yaw) * dt
            y += v * math.sin(yaw) * dt
            yaw += (v / self.wheelbase) * math.tan(steer) * dt
            traj.append(TrajectoryPoint(x, y, yaw, v, steer))
            if abs(self.normalize_angle(yaw - target_yaw)) < 0.15:
                break
                
        # Phase 2: Drive straight to merge point
        steer = 0.0
        v = self.get_velocity_limit(steer, forward=True)
        for _ in range(40):
            dx = target_x - x
            dy = target_y - y
            dist = math.sqrt(dx*dx + dy*dy)
            if dist < 0.15:
                break
            x += v * math.cos(yaw) * dt
            y += v * math.sin(yaw) * dt
            traj.append(TrajectoryPoint(x, y, yaw, v, steer))
            
        return traj

    def generate_reverse_alignment(self, x0, y0, yaw0, target_yaw, target_x, target_y, steer_dir):
        # Steer and move reverse
        traj = []
        x, y, yaw = x0, y0, yaw0
        dt = 0.1
        
        # Phase 1: Turn reverse
        for _ in range(70):
            steer = steer_dir * 0.45
            v = self.get_velocity_limit(steer, forward=False)
            x += v * math.cos(yaw) * dt
            y += v * math.sin(yaw) * dt
            yaw += (v / self.wheelbase) * math.tan(steer) * dt
            traj.append(TrajectoryPoint(x, y, yaw, v, steer))
            if abs(self.normalize_angle(yaw - target_yaw)) < 0.15:
                break
                
        # Phase 2: Drive straight reverse to merge point
        steer = 0.0
        v = self.get_velocity_limit(steer, forward=False)
        for _ in range(50):
            dx = target_x - x
            dy = target_y - y
            dist = math.sqrt(dx*dx + dy*dy)
            if dist < 0.15:
                break
            x += v * math.cos(yaw) * dt
            y += v * math.sin(yaw) * dt
            traj.append(TrajectoryPoint(x, y, yaw, v, steer))
            
        return traj

    def generate_u_turn(self, x0, y0, yaw0, target_yaw, steer_dir):
        # Perform 180-deg U-turn
        traj = []
        x, y, yaw = x0, y0, yaw0
        dt = 0.1
        
        # Turn until heading matches or we turn 180 degrees
        for _ in range(120):
            steer = steer_dir * 0.55
            v = self.get_velocity_limit(steer, forward=True)
            x += v * math.cos(yaw) * dt
            y += v * math.sin(yaw) * dt
            yaw += (v / self.wheelbase) * math.tan(steer) * dt
            traj.append(TrajectoryPoint(x, y, yaw, v, steer))
            if abs(self.normalize_angle(yaw - target_yaw)) < 0.15:
                break
        return traj

    def generate_three_point_turn(self, x0, y0, yaw0, target_yaw, steer_dir):
        # Multi-segment turn: reverse with steer, forward with opposite steer
        traj = []
        x, y, yaw = x0, y0, yaw0
        dt = 0.1
        
        # Segment 1: Reverse Left/Right
        steer1 = steer_dir * 0.55
        v1 = self.get_velocity_limit(steer1, forward=False)
        for _ in range(50):  # 5 seconds reverse
            x += v1 * math.cos(yaw) * dt
            y += v1 * math.sin(yaw) * dt
            yaw += (v1 / self.wheelbase) * math.tan(steer1) * dt
            traj.append(TrajectoryPoint(x, y, yaw, v1, steer1))
            
        # Segment 2: Forward Right/Left (opposite steering)
        steer2 = -steer_dir * 0.55
        v2 = self.get_velocity_limit(steer2, forward=True)
        for _ in range(70):  # 7 seconds forward
            x += v2 * math.cos(yaw) * dt
            y += v2 * math.sin(yaw) * dt
            yaw += (v2 / self.wheelbase) * math.tan(steer2) * dt
            traj.append(TrajectoryPoint(x, y, yaw, v2, steer2))
            if abs(self.normalize_angle(yaw - target_yaw)) < 0.15:
                break
        return traj

    def evaluate_candidates(self, x0, y0, yaw0, path_yaw, path_x, path_y):
        candidates = {}
        
        # Define candidates
        candidates["FORWARD_ALIGN_LEFT"] = self.generate_forward_alignment(x0, y0, yaw0, path_yaw, path_x, path_y, 1)
        candidates["FORWARD_ALIGN_RIGHT"] = self.generate_forward_alignment(x0, y0, yaw0, path_yaw, path_x, path_y, -1)
        candidates["REVERSE_ALIGN_LEFT"] = self.generate_reverse_alignment(x0, y0, yaw0, path_yaw, path_x, path_y, 1)
        candidates["REVERSE_ALIGN_RIGHT"] = self.generate_reverse_alignment(x0, y0, yaw0, path_yaw, path_x, path_y, -1)
        candidates["U_TURN_LEFT"] = self.generate_u_turn(x0, y0, yaw0, path_yaw, 1)
        candidates["U_TURN_RIGHT"] = self.generate_u_turn(x0, y0, yaw0, path_yaw, -1)
        candidates["THREE_POINT_LEFT"] = self.generate_three_point_turn(x0, y0, yaw0, path_yaw, 1)
        candidates["THREE_POINT_RIGHT"] = self.generate_three_point_turn(x0, y0, yaw0, path_yaw, -1)

        valid_candidates = {}
        candidate_costs = {}

        for name, traj in candidates.items():
            if not traj:
                continue
                
            # Collision Check
            collision = False
            for pt in traj:
                if self.is_in_collision(pt.x, pt.y, pt.yaw):
                    collision = True
                    break
                    
            if collision:
                continue

            # Compute Cost
            dist = 0.0
            smoothness = 0.0
            dir_changes = 0
            prev_v = traj[0].v
            prev_steer = traj[0].steer
            
            for i in range(len(traj)):
                pt = traj[i]
                if i > 0:
                    dx = pt.x - traj[i-1].x
                    dy = pt.y - traj[i-1].y
                    dist += math.sqrt(dx*dx + dy*dy)
                    smoothness += abs(pt.steer - prev_steer)
                    if (pt.v > 0 and prev_v < 0) or (pt.v < 0 and prev_v > 0):
                        dir_changes += 1
                prev_v = pt.v
                prev_steer = pt.steer

            duration = len(traj) * 0.1
            energy = 0.15 * dist + 1.8 * dir_changes
            
            # Final alignment error
            final_pose_err = math.sqrt((traj[-1].x - path_x)**2 + (traj[-1].y - path_y)**2)
            final_heading_err = abs(self.normalize_angle(traj[-1].yaw - path_yaw))

            # Weight parameters
            cost = (2.0 * dist + 
                    1.2 * duration + 
                    3.5 * dir_changes + 
                    1.5 * smoothness + 
                    1.0 * energy + 
                    4.0 * final_pose_err + 
                    8.0 * final_heading_err)

            candidate_costs[name] = cost
            valid_candidates[name] = traj

        return valid_candidates, candidate_costs

    def publish_visualization_markers(self, valid_candidates, selected_name, path_x, path_y, path_yaw):
        marker_arr = MarkerArray()
        
        # 1. Clear previous markers
        clear_marker = Marker()
        clear_marker.action = Marker.DELETEALL
        marker_arr.markers.append(clear_marker)
        self.pub_markers.publish(marker_arr)
        
        marker_arr = MarkerArray()
        idx = 0
        
        # 2. Publish candidates
        for name, traj in valid_candidates.items():
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "candidates"
            marker.id = idx
            idx += 1
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            
            # Draw line
            for pt in traj:
                p = Point()
                p.x = pt.x
                p.y = pt.y
                p.z = 0.02
                marker.points.append(p)
                
            marker.scale.x = 0.03
            
            # Colors: Selected is bright green, others are translucent blue/gray
            if name == selected_name:
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.color.a = 0.95
                marker.scale.x = 0.07  # Thicker line
            else:
                marker.color.r = 0.4
                marker.color.g = 0.4
                marker.color.b = 0.8
                marker.color.a = 0.35

            marker_arr.markers.append(marker)

        # 3. Publish Merge Point
        merge_marker = Marker()
        merge_marker.header.frame_id = "map"
        merge_marker.header.stamp = rospy.Time.now()
        merge_marker.ns = "merge_point"
        merge_marker.id = idx
        idx += 1
        merge_marker.type = Marker.SPHERE
        merge_marker.action = Marker.ADD
        merge_marker.pose.position.x = path_x
        merge_marker.pose.position.y = path_y
        merge_marker.pose.position.z = 0.1
        merge_marker.scale.x = 0.3
        merge_marker.scale.y = 0.3
        merge_marker.scale.z = 0.3
        merge_marker.color.r = 1.0
        merge_marker.color.g = 0.8
        merge_marker.color.b = 0.0
        merge_marker.color.a = 0.9
        marker_arr.markers.append(merge_marker)

        # 4. Publish Target Orientation Arrow
        arrow_marker = Marker()
        arrow_marker.header.frame_id = "map"
        arrow_marker.header.stamp = rospy.Time.now()
        arrow_marker.ns = "path_heading"
        arrow_marker.id = idx
        idx += 1
        arrow_marker.type = Marker.ARROW
        arrow_marker.action = Marker.ADD
        arrow_marker.pose.position.x = path_x
        arrow_marker.pose.position.y = path_y
        arrow_marker.pose.position.z = 0.15
        
        # Quaternion from yaw
        q = tf.transformations.quaternion_from_euler(0, 0, path_yaw)
        arrow_marker.pose.orientation.x = q[0]
        arrow_marker.pose.orientation.y = q[1]
        arrow_marker.pose.orientation.z = q[2]
        arrow_marker.pose.orientation.w = q[3]
        
        arrow_marker.scale.x = 0.6
        arrow_marker.scale.y = 0.08
        arrow_marker.scale.z = 0.08
        arrow_marker.color.r = 1.0
        arrow_marker.color.g = 0.0
        arrow_marker.color.b = 0.0
        arrow_marker.color.a = 1.0
        marker_arr.markers.append(arrow_marker)

        self.pub_markers.publish(marker_arr)

    def log_to_sqlite(self, initial_pose, initial_yaw, path_heading, selected_name, costs, p_time, exec_time, final_pose, success, reason=""):
        try:
            conn = sqlite3.connect(self.db_path)
            cursor = conn.cursor()
            
            # Calculate final orientation error
            final_err = abs(self.normalize_angle(final_pose[2] - path_heading))
            
            cursor.execute("""
            INSERT INTO alignment_maneuver_logs 
            (timestamp, initial_x, initial_y, initial_yaw, initial_heading, path_heading, 
             selected_maneuver, candidate_costs, planning_time, execution_time, 
             final_x, final_y, final_yaw, final_orientation_error, success, failure_reason)
            VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
            """, (
                rospy.Time.now().to_sec(),
                initial_pose[0], initial_pose[1], initial_yaw, initial_yaw, path_heading,
                selected_name, str(costs), p_time, exec_time,
                final_pose[0], final_pose[1], final_pose[2], final_err,
                1 if success else 0, reason
            ))
            
            conn.commit()
            conn.close()
            rospy.loginfo("[PathAlignmentPlanner] Logged alignment execution to SQLite database.")
        except Exception as e:
            rospy.logwarn(f"[PathAlignmentPlanner] SQLite logging failed: {e}")

    def execute_trajectory(self, traj):
        rate = rospy.Rate(10)
        start_time = rospy.Time.now()
        
        for idx, pt in enumerate(traj):
            if rospy.is_shutdown() or self.server.is_preempt_requested():
                self.pub_cmd_vel.publish(Twist())
                return False, "Preempted"

            # Check if dynamic collision occurs along the remaining trajectory
            # (Just check the next 5 points for safety)
            colliding = False
            for j in range(idx, min(idx + 5, len(traj))):
                future_pt = traj[j]
                if self.is_in_collision(future_pt.x, future_pt.y, future_pt.yaw):
                    colliding = True
                    break
                    
            if colliding:
                self.pub_cmd_vel.publish(Twist())
                return False, "Collision detected during execution"

            # Publish cmd_vel command
            cmd = Twist()
            cmd.linear.x = pt.v
            
            # Simple bicycle model yaw rate command translation:
            # w = (v / L) * tan(steer)
            cmd.angular.z = (pt.v / self.wheelbase) * math.tan(pt.steer)
            self.pub_cmd_vel.publish(cmd)
            
            rate.sleep()

        # Stop the robot
        self.pub_cmd_vel.publish(Twist())
        exec_time = (rospy.Time.now() - start_time).to_sec()
        return True, str(exec_time)

    def execute_cb(self, goal):
        rospy.loginfo("[PathAlignmentPlanner] Intercepted NavigateBehavior goal.")
        
        # 1. Fetch current pose
        start_time = rospy.Time.now()
        pose_init = self.get_robot_pose()
        if not pose_init:
            result = NavigateBehaviorResult()
            result.success = False
            result.message = "Failed to get initial robot pose via TF."
            self.server.set_aborted(result)
            return

        x0, y0, yaw0 = pose_init

        # 2. Get Global Plan
        target_pose = goal.target_goal
        start_pose_stamped = PoseStamped()
        start_pose_stamped.header.frame_id = 'map'
        start_pose_stamped.header.stamp = rospy.Time.now()
        start_pose_stamped.pose.position.x = x0
        start_pose_stamped.pose.position.y = y0
        start_pose_stamped.pose.orientation.w = 1.0 # arbitrary orientation for global planner start pose
        
        # Calculate plan
        plan = self.query_global_plan(start_pose_stamped.pose, target_pose.pose)
        if not plan or len(plan.poses) == 0:
            result = NavigateBehaviorResult()
            result.success = False
            result.message = "Failed to generate global plan for alignment."
            self.server.set_aborted(result)
            return

        # 3. Read Path Heading and Heading Error
        path_heading = self.get_path_start_heading(plan)
        heading_err = self.normalize_angle(path_heading - yaw0)
        
        rospy.loginfo(f"[PathAlignmentPlanner] Heading Error: {math.degrees(heading_err):.2f} degrees.")

        # 4. Check if Alignment is required
        if abs(heading_err) < self.alignment_threshold:
            rospy.loginfo("[PathAlignmentPlanner] Vehicle is already aligned. Forwarding goal directly to BDM.")
            self.forward_goal_to_bdm(goal)
            return

        # 5. Evaluate available maneuvers
        path_x = plan.poses[0].pose.position.x
        path_y = plan.poses[0].pose.position.y
        
        valid_candidates, candidate_costs = self.evaluate_candidates(x0, y0, yaw0, path_heading, path_x, path_y)
        
        p_time = (rospy.Time.now() - start_time).to_sec()

        if not valid_candidates:
            rospy.logwarn("[PathAlignmentPlanner] No collision-free alignment candidates found. Triggering Replan.")
            # Log failure to SQLite
            self.log_to_sqlite((x0, y0, yaw0), yaw0, path_heading, "NONE", {}, p_time, 0.0, (x0, y0, yaw0), False, "No valid maneuver found")
            result = NavigateBehaviorResult()
            result.success = False
            result.message = "No collision-free alignment maneuvers available."
            self.server.set_aborted(result)
            return

        # 6. Select lowest-cost maneuver
        selected_name = min(candidate_costs, key=candidate_costs.get)
        selected_traj = valid_candidates[selected_name]
        
        rospy.loginfo(f"[PathAlignmentPlanner] Selected Maneuver: {selected_name} with cost {candidate_costs[selected_name]:.2f}")

        # 7. Publish visualization markers
        self.publish_visualization_markers(valid_candidates, selected_name, path_x, path_y, path_heading)

        # 8. Execute the maneuver
        success, exec_res = self.execute_trajectory(selected_traj)

        # 9. Get final pose
        pose_final = self.get_robot_pose()
        if not pose_final:
            pose_final = (x0, y0, yaw0) # fallback
            
        xf, yf, yawf = pose_final

        if success:
            rospy.loginfo(f"[PathAlignmentPlanner] Alignment complete. Execution time: {exec_res}s. Handing over to BDM.")
            exec_time = float(exec_res)
            self.log_to_sqlite((x0, y0, yaw0), yaw0, path_heading, selected_name, candidate_costs, p_time, exec_time, (xf, yf, yawf), True)
            
            # Hand control back to the normal path-following controller (BDM)
            self.forward_goal_to_bdm(goal)
        else:
            rospy.logwarn(f"[PathAlignmentPlanner] Alignment execution failed: {exec_res}")
            self.log_to_sqlite((x0, y0, yaw0), yaw0, path_heading, selected_name, candidate_costs, p_time, 0.0, (xf, yf, yawf), False, exec_res)
            result = NavigateBehaviorResult()
            result.success = False
            result.message = f"Alignment maneuver aborted: {exec_res}"
            self.server.set_aborted(result)

    def forward_goal_to_bdm(self, goal):
        # Forward original goal to remapped BDM Action Server
        rospy.loginfo("[PathAlignmentPlanner] Connecting to navigate_behavior_bdm action server...")
        self.client_bdm.wait_for_server()
        
        bdm_goal = NavigateBehaviorGoal()
        bdm_goal.target_goal = goal.target_goal
        
        # Define feedback callback to proxy feedback
        def feedback_cb(feedback):
            self.server.publish_feedback(feedback)

        rospy.loginfo("[PathAlignmentPlanner] Forwarding goal to BDM.")
        self.client_bdm.send_goal(bdm_goal, feedback_cb=feedback_cb)
        
        # Monitor the BDM action state
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and not self.client_bdm.wait_for_result(rospy.Duration(0.1)):
            if self.server.is_preempt_requested():
                rospy.loginfo("[PathAlignmentPlanner] Preempt requested during BDM action. Cancelling BDM goal.")
                self.client_bdm.cancel_goal()
                self.server.set_preempted()
                return

        state = self.client_bdm.get_state()
        res = self.client_bdm.get_result()
        
        rospy.loginfo(f"[PathAlignmentPlanner] BDM action finished with state code: {state}.")
        
        # Return success or failure as appropriate
        if state == actionlib.GoalStatus.SUCCEEDED:
            self.server.set_succeeded(res)
        elif state == actionlib.GoalStatus.PREEMPTED:
            self.server.set_preempted()
        else:
            self.server.set_aborted(res)

if __name__ == '__main__':
    try:
        node = PathAlignmentPlannerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
