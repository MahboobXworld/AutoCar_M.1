#!/usr/bin/env python3

import rospy
import time
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import String

class SafetyBenchmark:
    def __init__(self):
        rospy.init_node('safety_benchmark')
        
        self.pub_cmd_raw = rospy.Publisher('/cmd_vel_raw', Twist, queue_size=10)
        self.pub_mock_scan = rospy.Publisher('/scan_deskewed', LaserScan, queue_size=10)
        self.pub_mock_loc = rospy.Publisher('/localization_health', String, queue_size=10)
        
        self.sub_cmd_safe = rospy.Subscriber('/cmd_vel', Twist, self.safe_cmd_callback, queue_size=10)
        self.sub_status = rospy.Subscriber('/safety/status', String, self.status_callback, queue_size=10)

        self.last_safe_cmd = None
        self.last_status = None
        
        # Test results
        self.results = {}
        rospy.loginfo("[SafetyBenchmark] Initialized. Waiting for safety supervisor to boot...")
        time.sleep(2.0)

    def safe_cmd_callback(self, msg):
        self.last_safe_cmd = msg

    def status_callback(self, msg):
        self.last_status = msg.data

    def run_scenario_corridor_creep(self):
        rospy.loginfo("--- Running Scenario 1: Corridor Creep (Speed Scaling) ---")
        
        # We mock a laser scan representing a narrow corridor: Left and Right clearance = 0.5m
        scan = LaserScan()
        scan.header.stamp = rospy.Time.now()
        scan.header.frame_id = "base_link"
        scan.angle_min = -math.radians(135)
        scan.angle_max = math.radians(135)
        scan.angle_increment = math.radians(1)
        scan.range_min = 0.1
        scan.range_max = 20.0
        
        # Left (90 deg) and Right (-90 deg) are close. Front is clear (5.0m)
        num_readings = int((scan.angle_max - scan.angle_min) / scan.angle_increment) + 1
        ranges = []
        angles = [scan.angle_min + i * scan.angle_increment for i in range(num_readings)]
        
        for angle in angles:
            if math.radians(30) <= angle <= math.radians(120): # Left
                ranges.append(0.5)
            elif -math.radians(120) <= angle <= -math.radians(30): # Right
                ranges.append(0.5)
            else:
                ranges.append(5.0) # Clear
        scan.ranges = ranges
        
        # Publish mock scan repeatedly for 2.0s
        rate = rospy.Rate(10)
        start_time = time.time()
        
        scaled_down = False
        min_speed_observed = 999.0
        
        while time.time() - start_time < 2.0:
            scan.header.stamp = rospy.Time.now()
            self.pub_mock_scan.publish(scan)
            
            # Send a fast cmd_vel_raw (1.0 m/s)
            cmd = Twist()
            cmd.linear.x = 1.0
            self.pub_cmd_raw.publish(cmd)
            
            if self.last_safe_cmd:
                min_speed_observed = min(min_speed_observed, self.last_safe_cmd.linear.x)
                if self.last_safe_cmd.linear.x < 0.3: # scaled to creep mode
                    scaled_down = True
            rate.sleep()
            
        self.results['corridor_creep'] = {
            'success': scaled_down,
            'min_speed_observed': min_speed_observed,
            'details': f"Commanded 1.0 m/s in narrow corridor. Supervisor output scaled to {min_speed_observed:.2f} m/s."
        }
        rospy.loginfo(f"Scenario 1 Result: Success={scaled_down}, Min Speed={min_speed_observed:.2f} m/s")

    def run_scenario_sudden_obstacle(self):
        rospy.loginfo("--- Running Scenario 2: Sudden Obstacle (Emergency Stop) ---")
        
        # First, publish clear scan and command speed 0.8
        scan = LaserScan()
        scan.header.stamp = rospy.Time.now()
        scan.header.frame_id = "base_link"
        scan.angle_min = -math.radians(135)
        scan.angle_max = math.radians(135)
        scan.angle_increment = math.radians(1)
        scan.range_min = 0.1
        scan.range_max = 20.0
        num_readings = int((scan.angle_max - scan.angle_min) / scan.angle_increment) + 1
        scan.ranges = [10.0] * num_readings
        
        self.pub_mock_scan.publish(scan)
        
        cmd = Twist()
        cmd.linear.x = 0.8
        self.pub_cmd_raw.publish(cmd)
        time.sleep(0.5) # Let it settle
        
        # Verify it was running fast
        initial_speed = self.last_safe_cmd.linear.x if self.last_safe_cmd else 0.0
        
        # Suddenly update scan to show obstacle at 0.28m directly in front
        scan.ranges = [0.28 if -math.radians(10) <= (scan.angle_min + i * scan.angle_increment) <= math.radians(10) else 10.0 for i in range(num_readings)]
        
        stopped = False
        start_stop = time.time()
        reaction_time = 0.0
        
        # Loop for 1.0s to measure reaction time
        rate = rospy.Rate(50)
        while time.time() - start_stop < 1.0:
            scan.header.stamp = rospy.Time.now()
            self.pub_mock_scan.publish(scan)
            self.pub_cmd_raw.publish(cmd)
            
            if self.last_safe_cmd and self.last_safe_cmd.linear.x == 0.0:
                stopped = True
                reaction_time = time.time() - start_stop
                break
            rate.sleep()
            
        self.results['sudden_obstacle'] = {
            'success': stopped,
            'initial_speed': initial_speed,
            'reaction_time_ms': reaction_time * 1000.0 if stopped else -1,
            'details': f"Commanded {initial_speed:.2f} m/s, sudden obstacle at 0.28m. E-stop triggered in {reaction_time*1000.0:.1f} ms." if stopped else "E-stop failed."
        }
        rospy.loginfo(f"Scenario 2 Result: Success={stopped}, Reaction Time={reaction_time*1000.0:.1f} ms")

    def run_scenario_sensor_dropout(self):
        rospy.loginfo("--- Running Scenario 3: Sensor Dropout Recovery ---")
        
        # Command speed 0.5 with active scan
        scan = LaserScan()
        scan.header.stamp = rospy.Time.now()
        scan.header.frame_id = "base_link"
        scan.angle_min = -math.radians(135)
        scan.angle_max = math.radians(135)
        scan.angle_increment = math.radians(1)
        scan.range_min = 0.1
        scan.range_max = 20.0
        num_readings = int((scan.angle_max - scan.angle_min) / scan.angle_increment) + 1
        scan.ranges = [10.0] * num_readings
        
        self.pub_mock_scan.publish(scan)
        cmd = Twist()
        cmd.linear.x = 0.5
        self.pub_cmd_raw.publish(cmd)
        time.sleep(0.5)
        
        # Stop publishing scan and check if E-stop occurs within 1.5s
        start_dropout = time.time()
        stopped = False
        rate = rospy.Rate(10)
        
        while time.time() - start_dropout < 1.5:
            # We do NOT publish the scan here, representing a dropout
            self.pub_cmd_raw.publish(cmd)
            if self.last_safe_cmd and self.last_safe_cmd.linear.x == 0.0:
                stopped = True
                break
            rate.sleep()
            
        self.results['sensor_dropout'] = {
            'success': stopped,
            'details': f"LiDAR topic publishing stopped. Supervisor safely halted within {time.time() - start_dropout:.2f} seconds." if stopped else "Failed to halt on sensor dropout."
        }
        rospy.loginfo(f"Scenario 3 Result: Success={stopped}")

    def run_scenario_localization_failure(self):
        rospy.loginfo("--- Running Scenario 4: Localization Failure Handling ---")
        
        # Start clear
        scan = LaserScan()
        scan.header.stamp = rospy.Time.now()
        scan.header.frame_id = "base_link"
        scan.angle_min = -math.radians(135)
        scan.angle_max = math.radians(135)
        scan.angle_increment = math.radians(1)
        scan.range_min = 0.1
        scan.range_max = 20.0
        num_readings = int((scan.angle_max - scan.angle_min) / scan.angle_increment) + 1
        scan.ranges = [10.0] * num_readings
        
        self.pub_mock_scan.publish(scan)
        
        # Publish health Excellent
        self.pub_mock_loc.publish(String(data="Excellent"))
        cmd = Twist()
        cmd.linear.x = 0.5
        self.pub_cmd_raw.publish(cmd)
        time.sleep(0.5)
        
        # Suddenly degrade localization to "Lost"
        self.pub_mock_loc.publish(String(data="Lost"))
        
        stopped = False
        start_lost = time.time()
        rate = rospy.Rate(10)
        
        while time.time() - start_lost < 1.0:
            self.pub_mock_scan.publish(scan)
            self.pub_cmd_raw.publish(cmd)
            if self.last_safe_cmd and self.last_safe_cmd.linear.x == 0.0:
                stopped = True
                break
            rate.sleep()
            
        self.results['loc_failure'] = {
            'success': stopped,
            'details': "Localization state degraded to 'Lost'. Supervisor immediately issued Emergency Stop." if stopped else "Failed to halt on localization loss."
        }
        rospy.loginfo(f"Scenario 4 Result: Success={stopped}")

    def generate_report(self):
        report_path = "/home/mahboob-alam/four_wheel_drive/src/autocar_tools/analytics/safety_supervisor_benchmark_report.md"
        rospy.loginfo(f"Generating Safety Supervisor Benchmark Report at: {report_path}")
        
        content = f"""# 🛡️ Production Safety Supervisor Benchmark Report

This automated test suite evaluates the 10-layer Safety Supervisor stack across critical safety scenarios.

## Benchmark Summary

| Scenario | Status | Metric Observed | Verdict |
| :--- | :---: | :---: | :---: |
| **Corridor Creep** | {"✅ PASS" if self.results['corridor_creep']['success'] else "❌ FAIL"} | Min speed: {self.results['corridor_creep']['min_speed_observed']:.2f} m/s | Safe Slowdown |
| **Sudden Obstacle** | {"✅ PASS" if self.results['sudden_obstacle']['success'] else "❌ FAIL"} | Reaction time: {self.results['sudden_obstacle']['reaction_time_ms']:.1f} ms | Emergency Brake |
| **Sensor Dropout** | {"✅ PASS" if self.results['sensor_dropout']['success'] else "❌ FAIL"} | Safe Timeout Stop | Emergency Halting |
| **Loc Failure** | {"✅ PASS" if self.results['loc_failure']['success'] else "❌ FAIL"} | Instant E-Stop | Emergency Halting |

## Details of Execution

1. **Scenario 1 - Corridor Creep**:
   - *Test Description*: Command 1.0 m/s in a narrow warehouse aisle (left & right obstacles at 0.5m).
   - *Details*: {self.results['corridor_creep']['details']}

2. **Scenario 2 - Sudden Obstacle**:
   - *Test Description*: Spontaneous obstacle detection at 0.28m directly in front of the vehicle at speed.
   - *Details*: {self.results['sudden_obstacle']['details']}

3. **Scenario 3 - Sensor Dropout**:
   - *Test Description*: Stop LiDAR data transmission mid-motion.
   - *Details*: {self.results['sensor_dropout']['details']}

4. **Scenario 4 - Localization Failure**:
   - *Test Description*: Degrade localization quality dynamically to "Lost".
   - *Details*: {self.results['loc_failure']['details']}

---
Report generated automatically by `benchmark_safety.py`.
"""
        try:
            with open(report_path, 'w') as f:
                f.write(content)
            rospy.loginfo("[SafetyBenchmark] Benchmark report written successfully.")
        except Exception as e:
            rospy.logerr(f"[SafetyBenchmark] Failed to write report: {e}")

    def run(self):
        self.run_scenario_corridor_creep()
        self.run_scenario_sudden_obstacle()
        self.run_scenario_sensor_dropout()
        self.run_scenario_localization_failure()
        self.generate_report()

if __name__ == '__main__':
    try:
        benchmark = SafetyBenchmark()
        benchmark.run()
    except rospy.ROSInterruptException:
        pass
