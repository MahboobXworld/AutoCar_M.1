#!/usr/bin/env python3
import rospy
import sqlite3
import rospkg
import os
from std_msgs.msg import String

class MissionManager:
    """
    High-Level Mission & Task Planner.
    Manages sequence execution (Pickup, Deliver, Charge) with retry/recovery logic
    and logs status updates to SQLite.
    """
    def __init__(self):
        rospy.init_node('mission_manager_node')

        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('autocar_ai')
        self.db_path = os.path.join(pkg_path, 'database', 'fleet_learning.db')
        self.init_database()

        # Define Mission Tasks
        self.tasks = [
            {"id": 1, "name": "Navigate to Pickup Zone", "state": "PENDING", "retries": 0},
            {"id": 2, "name": "Pallet Alignment", "state": "PENDING", "retries": 0},
            {"id": 3, "name": "Pick Up Pallet", "state": "PENDING", "retries": 0},
            {"id": 4, "name": "Navigate through Warehouse", "state": "PENDING", "retries": 0},
            {"id": 5, "name": "Avoid Obstacles", "state": "PENDING", "retries": 0},
            {"id": 6, "name": "Precise Docking", "state": "PENDING", "retries": 0},
            {"id": 7, "name": "Drop Pallet", "state": "PENDING", "retries": 0},
            {"id": 8, "name": "Return to Charger", "state": "PENDING", "retries": 0}
        ]
        self.current_task_idx = 0
        self.max_retries = 3

        # Publisher
        self.pub_mission_status = rospy.Publisher('/mission_status', String, queue_size=10)

        rospy.loginfo("[MissionManager] Hierarchical Mission Manager Node ready.")

    def init_database(self):
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        cursor.execute("""
        CREATE TABLE IF NOT EXISTS mission_tasks (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            timestamp REAL,
            task_name TEXT,
            state TEXT,
            retries INTEGER
        )""")
        conn.commit()
        conn.close()

    def log_task(self, name, state, retries):
        try:
            conn = sqlite3.connect(self.db_path)
            cursor = conn.cursor()
            cursor.execute("""
            INSERT INTO mission_tasks (timestamp, task_name, state, retries)
            VALUES (?, ?, ?, ?)
            """, (rospy.Time.now().to_sec(), name, state, retries))
            conn.commit()
            conn.close()
        except Exception as e:
            rospy.logwarn(f"[MissionManager] SQL task logging failed: {e}")

    def execute_mission(self):
        rate = rospy.Rate(0.5) # Process task steps every 2 seconds
        
        while not rospy.is_shutdown() and self.current_task_idx < len(self.tasks):
            task = self.tasks[self.current_task_idx]
            task["state"] = "RUNNING"
            self.log_task(task["name"], task["state"], task["retries"])

            status_str = f"Executing Task {self.current_task_idx + 1}/{len(self.tasks)}: {task['name']}"
            rospy.loginfo(f"[MissionManager] {status_str}")
            self.pub_mission_status.publish(status_str)

            # Simulate task execution with stochastic success
            rospy.sleep(3.0)
            success = float(np.random.rand()) > 0.15 # 85% success rate

            if success:
                task["state"] = "COMPLETED"
                self.log_task(task["name"], task["state"], task["retries"])
                rospy.loginfo(f"[MissionManager] Task succeeded: {task['name']}")
                self.current_task_idx += 1
            else:
                task["retries"] += 1
                rospy.logwarn(f"[MissionManager] Task failed: {task['name']}. Retry {task['retries']}/{self.max_retries}")
                
                if task["retries"] >= self.max_retries:
                    task["state"] = "ABORTED"
                    self.log_task(task["name"], task["state"], task["retries"])
                    rospy.logerr(f"[MissionManager] Max retries hit. Mission Aborted at task: {task['name']}")
                    break
                else:
                    task["state"] = "RECOVERY_RETRY"
                    self.log_task(task["name"], task["state"], task["retries"])
                    # Brief recovery sleep
                    rospy.sleep(2.0)

            rate.sleep()

        if self.current_task_idx == len(self.tasks):
            rospy.loginfo("[MissionManager] Deliver Pallet Mission Complete! All tasks succeeded.")
            self.pub_mission_status.publish("Mission Complete: All tasks successfully finished.")

if __name__ == '__main__':
    import numpy as np
    try:
        manager = MissionManager()
        rospy.sleep(2.0)
        manager.execute_mission()
    except rospy.ROSInterruptException:
        pass
