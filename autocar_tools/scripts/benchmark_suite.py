#!/usr/bin/env python3
import os
import time
import rospy
import actionlib
import sqlite3
import rospkg
from geometry_msgs.msg import PoseStamped
from autocar_interfaces.msg import NavigateBehaviorAction, NavigateBehaviorGoal
from std_srvs.srv import Empty

class BenchmarkSuite:
    def __init__(self):
        rospy.init_node('benchmark_suite')
        self.client = actionlib.SimpleActionClient('/navigate_behavior', NavigateBehaviorAction)
        
        rospack = rospkg.RosPack()
        self.pkg_path = rospack.get_path('autocar_ai')
        self.db_path = os.path.join(self.pkg_path, 'database', 'fleet_learning.db')
        
        # Test coordinates for evaluation scenarios
        self.test_scenarios = [
            {"x": 4.5, "y": 3.0, "yaw": 0.0, "name": "Scenario_1_Narrow_Passage"},
            {"x": -2.0, "y": 5.5, "yaw": 1.57, "name": "Scenario_2_Tight_Corner"},
            {"x": 8.0, "y": -4.0, "yaw": -3.14, "name": "Scenario_3_Reverse_Docking"}
        ]

    def set_use_rl(self, use_rl):
        rospy.set_param('/behavior_decision_manager/use_rl', use_rl)
        rospy.loginfo(f"[Benchmark] Configured arbitrator 'use_rl' to: {use_rl}")
        # Small sleep to let parameter propagate
        time.sleep(1.0)

    def run_scenario(self, scenario, mode_name):
        rospy.loginfo(f"[Benchmark] Starting {scenario['name']} under mode '{mode_name}'...")
        
        if not self.client.wait_for_server(rospy.Duration(5.0)):
            rospy.logerr("[Benchmark] /navigate_behavior action server not available!")
            return False

        goal = NavigateBehaviorGoal()
        goal.target_goal.header.frame_id = "map"
        goal.target_goal.header.stamp = rospy.Time.now()
        goal.target_goal.pose.position.x = scenario['x']
        goal.target_goal.pose.position.y = scenario['y']
        
        # Simple yaw to quaternion conversion
        q_z = float(np.sin(scenario['yaw'] / 2.0))
        q_w = float(np.cos(scenario['yaw'] / 2.0))
        goal.target_goal.pose.orientation.z = q_z
        goal.target_goal.pose.orientation.w = q_w

        self.client.send_goal(goal)
        self.client.wait_for_result()
        
        result = self.client.get_result()
        status = self.client.get_state()
        
        success = (status == actionlib.GoalStatus.SUCCEEDED)
        msg = result.message if result else "No result message"
        rospy.loginfo(f"[Benchmark] Finished {scenario['name']}. Success={success} | Message: {msg}")
        return success

    def run_suite(self):
        rospy.loginfo("[Benchmark] Commencing full benchmarking suite...")

        # 1. Evaluate Rule-based baseline decision tree
        self.set_use_rl(False)
        for s in self.test_scenarios:
            self.run_scenario(s, "baseline_tree")
            time.sleep(2.0)

        # 2. Evaluate RL Policy with fallback
        self.set_use_rl(True)
        for s in self.test_scenarios:
            self.run_scenario(s, "ppo_rl_with_fallback")
            time.sleep(2.0)

        rospy.loginfo("[Benchmark] All benchmark scenarios completed. Processing analytics...")
        
        # Run the analytics generator
        os.system("rosrun autocar_tools navigation_analytics.py")

if __name__ == '__main__':
    # Need numpy to compute quaternion, import inline
    import numpy as np
    suite = BenchmarkSuite()
    suite.run_suite()
