#!/usr/bin/env python3

import rospy
import sqlite3
import numpy as np
import tf
import math
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, Twist
from gazebo_msgs.msg import ModelStates
from visualization_msgs.msg import Marker
from std_srvs.srv import Empty
import dynamic_reconfigure.client
import threading
import queue

class LocalizationManagerNode:
    def __init__(self):
        rospy.init_node('localization_manager_node')

        # Load parameters
        self.db_path = rospy.get_param('~db_path', '/home/mahboob-alam/four_wheel_drive/src/autocar_ai/database/fleet_learning.db')
        self.wheelbase = rospy.get_param('~wheelbase', 0.70)
        self.max_steer_angle = rospy.get_param('~max_steer_angle', 0.785398)

        # Cache variables
        self.ekf_odom = None
        self.wheel_odom = None
        self.imu_data = None
        self.amcl_pose = None
        self.particle_cloud = None
        self.gt_pose = None

        # State flags
        self.slip_detected = False
        self.recovery_active = False
        self.health_status = "Excellent"
        self.confidence = 1.0
        self.cov_trace = 0.0

        # Speed scaling factor
        self.speed_scale = 1.0

        # SQLite logging queue and thread
        self.log_queue = queue.Queue()
        self.db_init()
        self.log_thread = threading.Thread(target=self.sqlite_worker)
        self.log_thread.daemon = True
        self.log_thread.start()

        # Dynamic Reconfigure Client for AMCL
        self.amcl_client = None
        try:
            self.amcl_client = dynamic_reconfigure.client.Client("amcl", timeout=2.0)
            rospy.loginfo("[LocalizationManager] Connected to AMCL dynamic reconfigure.")
        except Exception as e:
            rospy.logwarn(f"[LocalizationManager] AMCL dynamic reconfigure not available: {e}")

        # TF Listener
        self.tf_listener = tf.TransformListener()

        # Publishers
        self.pub_odom_filtered = rospy.Publisher('/diff_drive_controller/odom_filtered', Odometry, queue_size=5)
        self.pub_health = rospy.Publisher('/localization_health', String, queue_size=5, latch=True)
        self.pub_marker = rospy.Publisher('/localization/health_marker', Marker, queue_size=5, latch=True)

        # Service clients
        self.srv_global_loc = None
        rospy.loginfo("[LocalizationManager] Waiting for AMCL global relocalization service...")
        try:
            rospy.wait_for_service('/global_localization', timeout=3.0)
            self.srv_global_loc = rospy.ServiceProxy('/global_localization', Empty)
            rospy.loginfo("[LocalizationManager] Connected to AMCL global relocalization.")
        except rospy.ROSException:
            rospy.logwarn("[LocalizationManager] AMCL global relocalization service not available.")

        # Subscribers
        self.sub_ekf = rospy.Subscriber('/odometry/filtered', Odometry, self.ekf_callback, queue_size=1)
        self.sub_wheel = rospy.Subscriber('/diff_drive_controller/odom', Odometry, self.wheel_callback, queue_size=1)
        self.sub_imu = rospy.Subscriber('/imu/data', Imu, self.imu_callback, queue_size=1)
        self.sub_amcl = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_callback, queue_size=1)
        self.sub_particles = rospy.Subscriber('/particlecloud', PoseArray, self.particles_callback, queue_size=1)
        self.sub_gazebo = rospy.Subscriber('/gazebo/model_states', ModelStates, self.gazebo_callback, queue_size=1)

        # Control loop
        self.timer = rospy.Timer(rospy.Duration(0.1), self.control_loop)
        self.log_timer = rospy.Timer(rospy.Duration(1.0), self.logging_loop)

        rospy.loginfo("[LocalizationManager] Node fully started.")

    def db_init(self):
        try:
            conn = sqlite3.connect(self.db_path)
            cursor = conn.cursor()
            cursor.execute("""
            CREATE TABLE IF NOT EXISTS localization_logs (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                timestamp REAL,
                x_gt REAL, y_gt REAL, yaw_gt REAL,
                x_est REAL, y_est REAL, yaw_est REAL,
                x_err REAL, y_err REAL, yaw_err REAL,
                cov_trace REAL,
                health_status TEXT,
                confidence REAL,
                slip_detected INTEGER,
                recovery_active INTEGER
            )
            """)
            conn.commit()
            conn.close()
        except Exception as e:
            rospy.logerr(f"[LocalizationManager] SQLite init failed: {e}")

    def sqlite_worker(self):
        while not rospy.is_shutdown():
            try:
                data = self.log_queue.get(timeout=1.0)
                conn = sqlite3.connect(self.db_path)
                cursor = conn.cursor()
                cursor.execute("""
                INSERT INTO localization_logs 
                (timestamp, x_gt, y_gt, yaw_gt, x_est, y_est, yaw_est, x_err, y_err, yaw_err, cov_trace, health_status, confidence, slip_detected, recovery_active)
                VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                """, data)
                conn.commit()
                conn.close()
            except queue.Empty:
                continue
            except Exception as e:
                rospy.logwarn(f"[LocalizationManager] SQLite insert failed: {e}")

    def ekf_callback(self, msg):
        self.ekf_odom = msg

    def wheel_callback(self, msg):
        self.wheel_odom = msg
        # Pass-through and scale covariance if slip detected
        filtered_odom = msg
        
        # Original covariance from ackermann_controller: pose covariance is 0.001, twist is 0.001
        # If slip is detected, drastically inflate covariance so EKF disregards encoders
        if self.slip_detected:
            # Set X, Y, Z, Yaw twist covariances to huge values
            cov = list(msg.twist.covariance)
            cov[0] = 50.0  # linear x variance
            cov[35] = 50.0 # angular z variance
            filtered_odom.twist.covariance = tuple(cov)

            cov_pose = list(msg.pose.covariance)
            cov_pose[0] = 50.0
            cov_pose[35] = 50.0
            filtered_odom.pose.covariance = tuple(cov_pose)
            
        self.pub_odom_filtered.publish(filtered_odom)

    def imu_callback(self, msg):
        self.imu_data = msg

    def amcl_callback(self, msg):
        self.amcl_pose = msg

    def particles_callback(self, msg):
        self.particle_cloud = msg

    def gazebo_callback(self, msg):
        try:
            idx = msg.name.index('autocar')
            self.gt_pose = msg.pose[idx]
        except ValueError:
            pass


    def control_loop(self, event):
        # 1. Slip Detection
        self.detect_wheel_slip()

        # 2. Evaluate Localization Health & Confidence
        self.evaluate_health()

        # 3. Dynamic Parameter Adaptation
        self.adapt_parameters()

        # 4. Recovery Actions
        self.run_recovery_state_machine()

        # 5. Publish Health
        self.pub_health.publish(String(data=self.health_status))
        self.publish_health_marker()

    def publish_health_marker(self):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "localization_health"
        marker.id = 1
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 1.2
        marker.pose.orientation.w = 1.0
        
        marker.scale.z = 0.25 # Text height
        
        marker.text = f"Health: {self.health_status}\nConf: {self.confidence:.2%}\nSlip: {'YES' if self.slip_detected else 'NO'}"
        
        if self.health_status in ["Excellent", "Good"]:
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
        elif self.health_status in ["Warning", "Poor"]:
            marker.color.r = 1.0
            marker.color.g = 0.6
            marker.color.b = 0.0
        else: # Lost
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            
        marker.color.a = 1.0
        marker.lifetime = rospy.Duration(1.0)
        self.pub_marker.publish(marker)

    def detect_wheel_slip(self):
        if not self.wheel_odom or not self.imu_data or not self.ekf_odom:
            self.slip_detected = False
            return

        # Compare wheel odometry angular velocity against IMU yaw rate
        wheel_w = self.wheel_odom.twist.twist.angular.z
        imu_w = self.imu_data.angular_velocity.z
        w_diff = abs(wheel_w - imu_w)

        # Compare wheel speed against EKF speed
        wheel_v = self.wheel_odom.twist.twist.linear.x
        ekf_v = self.ekf_odom.twist.twist.linear.x
        v_diff = abs(wheel_v - ekf_v)

        # Flag slip if large discrepancy occurs during non-trivial movement
        if (w_diff > 0.35 and abs(wheel_v) > 0.05) or (v_diff > 0.4 and abs(wheel_v) > 0.1):
            if not self.slip_detected:
                rospy.logwarn(f"[LocalizationManager] Wheel slip detected! Yaw rate diff: {w_diff:.3f} rad/s, Speed diff: {v_diff:.3f} m/s")
            self.slip_detected = True
        else:
            self.slip_detected = False

    def evaluate_health(self):
        # Compute confidence score [0.0, 1.0]
        c_amcl = 1.0
        c_particles = 1.0
        c_consistency = 1.0
        c_slip = 1.0 if not self.slip_detected else 0.5

        # A. AMCL Covariance Trace
        if self.amcl_pose:
            cov = self.amcl_pose.pose.covariance
            self.cov_trace = cov[0] + cov[7] + cov[35] # x + y + yaw variances
            
            # Map covariance to score: trace < 0.15 is excellent, > 3.0 is lost
            if self.cov_trace < 0.15:
                c_amcl = 1.0
            elif self.cov_trace > 3.0:
                c_amcl = 0.0
            else:
                c_amcl = 1.0 - (self.cov_trace - 0.15) / 2.85
        else:
            c_amcl = 0.5

        # B. Particle Spread (Dispersion)
        if self.particle_cloud and len(self.particle_cloud.poses) > 0:
            poses = self.particle_cloud.poses
            xs = [p.position.x for p in poses]
            ys = [p.position.y for p in poses]
            std_x = np.std(xs)
            std_y = np.std(ys)
            dispersion = math.sqrt(std_x**2 + std_y**2)

            # Map dispersion to score: < 0.35m is tight, > 2.0m is spread
            if dispersion < 0.35:
                c_particles = 1.0
            elif dispersion > 2.0:
                c_particles = 0.1
            else:
                c_particles = 1.0 - (dispersion - 0.35) / 1.65
        else:
            c_particles = 0.5

        # C. EKF and AMCL Pose Consistency
        if self.amcl_pose:
            dist = 0.0
            try:
                self.tf_listener.waitForTransform('map', 'base_link', rospy.Time(0), rospy.Duration(0.05))
                (trans, rot) = self.tf_listener.lookupTransform('map', 'base_link', rospy.Time(0))
                x_tf = trans[0]
                y_tf = trans[1]
                amcl_x = self.amcl_pose.pose.pose.position.x
                amcl_y = self.amcl_pose.pose.pose.position.y
                dist = math.sqrt((x_tf - amcl_x)**2 + (y_tf - amcl_y)**2)
            except Exception:
                pass

            # Map distance inconsistency to score: < 0.35m is excellent, > 2.0m is lost
            if dist < 0.35:
                c_consistency = 1.0
            elif dist > 2.0:
                c_consistency = 0.0
            else:
                c_consistency = 1.0 - (dist - 0.35) / 1.65
        else:
            c_consistency = 0.5

        # D. Weighted confidence calculation
        self.confidence = 0.35 * c_amcl + 0.25 * c_particles + 0.25 * c_consistency + 0.15 * c_slip
        self.confidence = max(0.0, min(1.0, self.confidence))

        # E. Classify health
        if self.confidence >= 0.85:
            self.health_status = "Excellent"
        elif self.confidence >= 0.65:
            self.health_status = "Good"
        elif self.confidence >= 0.45:
            self.health_status = "Warning"
        elif self.confidence >= 0.25:
            self.health_status = "Poor"
        else:
            self.health_status = "Lost"

    def adapt_parameters(self):
        if not self.amcl_client or not self.ekf_odom:
            return

        current_speed = abs(self.ekf_odom.twist.twist.linear.x)
        current_w = abs(self.ekf_odom.twist.twist.angular.z)

        # Dynamic update bounds
        # At high speed or high yaw rate, increase update frequency to capture motion,
        # but reduce CPU usage by raising the distance/angle thresholds slightly if very fast
        if current_speed > 0.6 or current_w > 0.8:
            new_min_d = 0.25
            new_min_a = 0.25
        elif current_speed < 0.15 and current_w < 0.2:
            # Low speed: require high precision updates
            new_min_d = 0.05
            new_min_a = 0.05
        else:
            new_min_d = 0.12
            new_min_a = 0.12

        try:
            # Call dynamic reconfigure update asynchronously to prevent blocking
            config = {'update_min_d': new_min_d, 'update_min_a': new_min_a}
            self.amcl_client.update_configuration(config)
        except Exception as e:
            pass

    def run_recovery_state_machine(self):
        if self.health_status in ["Excellent", "Good"]:
            self.speed_scale = 1.0
            self.recovery_active = False

        elif self.health_status == "Warning":
            # Safety speed reduction
            self.speed_scale = 0.60
            self.recovery_active = True
            rospy.logwarn_throttle(2.0, "[LocalizationManager] Warning: Localization confidence degraded. Reducing velocity to 60%.")

        elif self.health_status == "Poor":
            # Strict safety speed reduction, increase amcl parameters manually
            self.speed_scale = 0.35
            self.recovery_active = True
            rospy.logwarn_throttle(2.0, "[LocalizationManager] Poor: Localization degraded severely! Reducing velocity to 35% and forcing AMCL update.")
            
            # Force AMCL update by updating configuration to lowest thresholds
            if self.amcl_client:
                try:
                    self.amcl_client.update_configuration({'update_min_d': 0.01, 'update_min_a': 0.01})
                except:
                    pass

        elif self.health_status == "Lost":
            self.speed_scale = 0.0 # E-stop halt
            self.recovery_active = True
            rospy.logerr_throttle(2.0, "[LocalizationManager] CRITICAL LOST: Localization is completely lost! HALTING robot and triggering global relocalization.")
            
            # Trigger AMCL global relocalization service
            if self.srv_global_loc:
                try:
                    self.srv_global_loc()
                except Exception as e:
                    rospy.logwarn(f"[LocalizationManager] Failed to trigger global relocalization service: {e}")

    def logging_loop(self, event):
        if not self.gt_pose:
            return

        # Fetch ground truth
        x_gt = self.gt_pose.position.x
        y_gt = self.gt_pose.position.y
        
        q_gt = [
            self.gt_pose.orientation.x,
            self.gt_pose.orientation.y,
            self.gt_pose.orientation.z,
            self.gt_pose.orientation.w
        ]
        yaw_gt = tf.transformations.euler_from_quaternion(q_gt)[2]

        # Fetch global estimated pose from TF (map -> base_link)
        x_est, y_est, yaw_est = 0.0, 0.0, 0.0
        try:
            (trans, rot) = self.tf_listener.lookupTransform('map', 'base_link', rospy.Time(0))
            x_est = trans[0]
            y_est = trans[1]
            yaw_est = tf.transformations.euler_from_quaternion(rot)[2]
        except Exception:
            if self.ekf_odom:
                x_est = self.ekf_odom.pose.pose.position.x
                y_est = self.ekf_odom.pose.pose.position.y
                q_est = [
                    self.ekf_odom.pose.pose.orientation.x,
                    self.ekf_odom.pose.pose.orientation.y,
                    self.ekf_odom.pose.pose.orientation.z,
                    self.ekf_odom.pose.pose.orientation.w
                ]
                yaw_est = tf.transformations.euler_from_quaternion(q_est)[2]
            else:
                return

        # Errors
        x_err = x_est - x_gt
        y_err = y_est - y_gt
        yaw_err = yaw_est - yaw_gt
        # Normalize yaw error
        yaw_err = math.atan2(math.sin(yaw_err), math.cos(yaw_err))

        # Pack data
        data = (
            rospy.Time.now().to_sec(),
            x_gt, y_gt, yaw_gt,
            x_est, y_est, yaw_est,
            x_err, y_err, yaw_err,
            self.cov_trace,
            self.health_status,
            self.confidence,
            1 if self.slip_detected else 0,
            1 if self.recovery_active else 0
        )
        self.log_queue.put(data)

if __name__ == '__main__':
    try:
        node = LocalizationManagerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
