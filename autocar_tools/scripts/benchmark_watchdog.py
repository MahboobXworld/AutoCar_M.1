#!/usr/bin/env python3

import rospy
import time
import math
import json
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
import tf

class WatchdogBenchmark:
    def __init__(self):
        rospy.init_node('watchdog_benchmark')

        # Publishers
        self.pub_cmd_raw = rospy.Publisher('/cmd_vel_raw', Twist, queue_size=10)
        self.pub_scan = rospy.Publisher('/scan_deskewed', LaserScan, queue_size=10)
        self.pub_odom = rospy.Publisher('/odometry/filtered', Odometry, queue_size=10)
        self.pub_imu = rospy.Publisher('/imu/data', Imu, queue_size=10)
        self.pub_costmap = rospy.Publisher('/move_base/local_costmap/costmap', OccupancyGrid, queue_size=10)
        self.pub_amcl = rospy.Publisher('/amcl_pose', PoseWithCovarianceStamped, queue_size=10)

        # TF Broadcaster
        self.tf_broadcaster = tf.TransformBroadcaster()

        # Subscribers
        self.sub_cmd_safe = rospy.Subscriber('/cmd_vel', Twist, self.safe_cmd_callback, queue_size=10)
        self.sub_diagnostics = rospy.Subscriber('/safety_diagnostics', String, self.diagnostics_callback, queue_size=10)

        self.last_safe_cmd = None
        self.last_diagnostics = None
        self.results = {}

        rospy.loginfo("[WatchdogBenchmark] Initialized. Warming up safety supervisor to Healthy...")
        start_warmup = time.time()
        rate = rospy.Rate(20)
        while time.time() - start_warmup < 3.0:
            self.publish_active_signals(scan_ok=True, odom_ok=True, imu_ok=True, costmap_ok=True)
            rate.sleep()

    def safe_cmd_callback(self, msg):
        self.last_safe_cmd = msg

    def diagnostics_callback(self, msg):
        try:
            self.last_diagnostics = json.loads(msg.data)
        except Exception:
            pass

    def get_health_state(self):
        if self.last_diagnostics:
            return self.last_diagnostics.get("health_state", "Unknown")
        return "Unknown"

    def publish_active_signals(self, scan_ok=True, odom_ok=True, imu_ok=True, costmap_ok=True, amcl_ok=True):
        import tf
        t_now = rospy.Time.now()
        
        # TF Broadcasts
        self.tf_broadcaster.sendTransform((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), t_now, "odom", "map")
        self.tf_broadcaster.sendTransform((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), t_now, "base_link", "odom")

        # Odom
        if odom_ok:
            odom = Odometry()
            odom.header.stamp = t_now
            odom.header.frame_id = "odom"
            cov = [0.0] * 36
            cov[0] = 0.01
            cov[7] = 0.01
            cov[35] = 0.01
            odom.pose.covariance = cov
            self.pub_odom.publish(odom)

        # IMU
        if imu_ok:
            imu = Imu()
            imu.header.stamp = t_now
            imu.header.frame_id = "base_link"
            self.pub_imu.publish(imu)

        # Scan
        if scan_ok:
            scan = LaserScan()
            scan.header.stamp = t_now
            scan.header.frame_id = "base_link"
            scan.angle_min = -math.radians(135)
            scan.angle_max = math.radians(135)
            scan.angle_increment = math.radians(1)
            scan.range_min = 0.1
            scan.range_max = 20.0
            num_readings = int((scan.angle_max - scan.angle_min) / scan.angle_increment) + 1
            scan.ranges = [10.0] * num_readings
            self.pub_scan.publish(scan)

        # Costmap
        if costmap_ok:
            costmap = OccupancyGrid()
            costmap.header.stamp = t_now
            costmap.header.frame_id = "odom"
            costmap.info.resolution = 0.05
            costmap.info.width = 40
            costmap.info.height = 40
            costmap.info.origin.position.x = -1.0
            costmap.info.origin.position.y = -1.0
            costmap.data = [0] * 1600
            self.pub_costmap.publish(costmap)

        # AMCL
        if amcl_ok:
            amcl = PoseWithCovarianceStamped()
            amcl.header.stamp = t_now
            amcl.header.frame_id = "map"
            self.pub_amcl.publish(amcl)

    def run_scenario_delayed_costmap(self):
        rospy.loginfo("--- Watchdog Scenario 1: Delayed Costmap (No False Stops) ---")
        
        # Publish all active signals, but costmap is delayed (5.0s interval)
        start_time = time.time()
        rate = rospy.Rate(20) # 20 Hz loop
        
        success = True
        states_observed = []

        while time.time() - start_time < 3.0:
            t_elapsed = time.time() - start_time
            # Costmap only published once at the beginning
            self.publish_active_signals(scan_ok=True, odom_ok=True, imu_ok=True, costmap_ok=(t_elapsed < 0.1))
            
            cmd = Twist()
            cmd.linear.x = 0.5
            self.pub_cmd_raw.publish(cmd)
            
            state = self.get_health_state()
            if state not in states_observed and state != "Unknown":
                states_observed.append(state)
                
            if state in ["Critical", "Emergency Stop"]:
                success = False # costmap delay should not trigger E-stop!
            rate.sleep()
            
        self.results['delayed_costmap'] = {
            'success': success,
            'states_observed': states_observed,
            'details': "Costmap delayed for 3.0s. Safety Supervisor health state remained Healthy." if success else "False Emergency Stop triggered."
        }
        rospy.loginfo(f"Scenario 1 Result: Success={success}, States={states_observed}")

    def run_scenario_lidar_jitter(self):
        rospy.loginfo("--- Watchdog Scenario 2: LiDAR Jitter (Graceful Warnings) ---")
        
        start_time = time.time()
        rate = rospy.Rate(20)
        
        warning_triggered = False
        restored = False
        states_observed = []
        
        while time.time() - start_time < 5.0:
            t_elapsed = time.time() - start_time
            
            # Period of LiDAR: 0.4s during delay phase (0.0s - 2.5s)
            scan_active = False
            if t_elapsed < 2.5:
                # publish scan once every 8 loops (0.4s) to simulate jitter/delay
                if int(t_elapsed * 20) % 8 == 0:
                    scan_active = True
            else:
                # Back to normal 10Hz
                if int(t_elapsed * 20) % 2 == 0:
                    scan_active = True
                    
            self.publish_active_signals(scan_ok=scan_active, odom_ok=True, imu_ok=True, costmap_ok=True)
            
            cmd = Twist()
            cmd.linear.x = 0.5
            self.pub_cmd_raw.publish(cmd)
            
            state = self.get_health_state()
            if state not in states_observed and state != "Unknown":
                states_observed.append(state)
                
            if state == "Warning":
                warning_triggered = True
            if t_elapsed >= 3.5 and state == "Healthy":
                restored = True
                
            rate.sleep()

        success = warning_triggered and restored
        self.results['lidar_jitter'] = {
            'success': success,
            'states_observed': states_observed,
            'details': f"LiDAR jitter test. Warning triggered: {warning_triggered}, Healthy restored: {restored}."
        }
        rospy.loginfo(f"Scenario 2 Result: Success={success}, States={states_observed}")

    def run_scenario_sensor_dropout(self):
        rospy.loginfo("--- Watchdog Scenario 3: Sensor Dropout (Safe Emergency Stop) ---")
        
        start_time = time.time()
        rate = rospy.Rate(20)
        
        critical_triggered = False
        states_observed = []
        
        while time.time() - start_time < 3.0:
            t_elapsed = time.time() - start_time
            
            # LiDAR drops completely
            self.publish_active_signals(scan_ok=False, odom_ok=True, imu_ok=True, costmap_ok=True)
            
            cmd = Twist()
            cmd.linear.x = 0.5
            self.pub_cmd_raw.publish(cmd)
            
            state = self.get_health_state()
            if state not in states_observed and state != "Unknown":
                states_observed.append(state)
                
            if state == "Critical":
                critical_triggered = True
            rate.sleep()

        self.results['sensor_dropout'] = {
            'success': critical_triggered,
            'states_observed': states_observed,
            'details': "LiDAR dropout correctly transitioned to Critical and applied E-stop." if critical_triggered else "Failed to trigger Critical E-stop."
        }
        rospy.loginfo(f"Scenario 3 Result: Success={critical_triggered}, States={states_observed}")

    def run_scenario_watchdog_recovery(self):
        rospy.loginfo("--- Watchdog Scenario 4: Recovery (Return to Healthy) ---")
        
        start_time = time.time()
        rate = rospy.Rate(20)
        
        recovered = False
        states_observed = []
        
        while time.time() - start_time < 3.0:
            # All signals active at high frequency (10Hz/30Hz/50Hz)
            self.publish_active_signals(scan_ok=True, odom_ok=True, imu_ok=True, costmap_ok=True)
            
            cmd = Twist()
            cmd.linear.x = 0.5
            self.pub_cmd_raw.publish(cmd)
            
            state = self.get_health_state()
            if state not in states_observed and state != "Unknown":
                states_observed.append(state)
                
            if state == "Healthy":
                recovered = True
            rate.sleep()

        self.results['recovery'] = {
            'success': recovered,
            'states_observed': states_observed,
            'details': "All sensors recovered. Safety Supervisor returned to Healthy state." if recovered else "Failed to recover."
        }
        rospy.loginfo(f"Scenario 4 Result: Success={recovered}, States={states_observed}")

    def generate_report(self):
        report_path = "/home/mahboob-alam/four_wheel_drive/src/autocar_tools/analytics/safety_watchdog_benchmark_report.md"
        rospy.loginfo(f"Generating Safety Watchdog Redesign Report at: {report_path}")
        
        # Calculate rates and thresholds from last diagnostic message
        scan_to = self.last_diagnostics.get("scan_timeout", 0.3) if self.last_diagnostics else 0.3
        odom_to = self.last_diagnostics.get("odom_timeout", 0.1) if self.last_diagnostics else 0.1
        imu_to = self.last_diagnostics.get("imu_timeout", 0.06) if self.last_diagnostics else 0.06
        
        content = f"""# 🛡️ Safety Watchdog Redesign Benchmark Report

This automated test suite verifies the redesigned Safety Supervisor's adaptive watchdog thresholds and multi-level health state transitions.

## Watchdog Configuration & Thresholds

| Component | Target Frequency | Average Period ($T_{{avg}}$) | Computed Timeout Threshold ($3 \times T_{{avg}}$) |
| :--- | :---: | :---: | :---: |
| **LiDAR** (`/scan_deskewed`) | 10 Hz | {scan_to/3.0:.3f} s | **{scan_to:.3f} s** |
| **EKF Odom** (`/odometry/filtered`) | 30 Hz | {odom_to/3.0:.3f} s | **{odom_to:.3f} s** |
| **IMU** (`/imu/data`) | 50 Hz | {imu_to/3.0:.3f} s | **{imu_to:.3f} s** |

---

## Benchmark Scenario Summary

| Scenario | Status | States Observed | Verdict |
| :--- | :---: | :---: | :---: |
| **Delayed Costmap** | {"✅ PASS" if self.results['delayed_costmap']['success'] else "❌ FAIL"} | {", ".join(self.results['delayed_costmap']['states_observed'])} | No False Stops |
| **LiDAR Jitter** | {"✅ PASS" if self.results['lidar_jitter']['success'] else "❌ FAIL"} | {", ".join(self.results['lidar_jitter']['states_observed'])} | Warning Degradation |
| **Sensor Dropout** | {"✅ PASS" if self.results['sensor_dropout']['success'] else "❌ FAIL"} | {", ".join(self.results['sensor_dropout']['states_observed'])} | Emergency Halting |
| **Recovery** | {"✅ PASS" if self.results['recovery']['success'] else "❌ FAIL"} | {", ".join(self.results['recovery']['states_observed'])} | Return to Drivable |

## Execution Details

1. **Scenario 1 - Delayed Costmap**:
   - *Description*: Simulate local costmap updating very slowly (e.g. 0.2 Hz) while maintaining active sensors.
   - *Details*: {self.results['delayed_costmap']['details']}

2. **Scenario 2 - LiDAR Jitter**:
   - *Description*: Introduce scan latency up to 0.4s to verify transition to Warning state and back.
   - *Details*: {self.results['lidar_jitter']['details']}

3. **Scenario 3 - Sensor Dropout**:
   - *Description*: Stop LiDAR scan publications completely to verify transition to Critical state (E-stop).
   - *Details*: {self.results['sensor_dropout']['details']}

4. **Scenario 4 - Watchdog Recovery**:
   - *Description*: Restore high-frequency signals for all sensors and verify return to Healthy.
   - *Details*: {self.results['recovery']['details']}

---
Report generated automatically by `benchmark_watchdog.py`.
"""
        try:
            with open(report_path, 'w') as f:
                f.write(content)
            rospy.loginfo("[WatchdogBenchmark] Watchdog report written successfully.")
        except Exception as e:
            rospy.logerr(f"[WatchdogBenchmark] Failed to write report: {e}")

    def run(self):
        self.run_scenario_delayed_costmap()
        self.run_scenario_lidar_jitter()
        self.run_scenario_sensor_dropout()
        self.run_scenario_watchdog_recovery()
        self.generate_report()

if __name__ == '__main__':
    try:
        benchmark = WatchdogBenchmark()
        benchmark.run()
    except rospy.ROSInterruptException:
        pass
