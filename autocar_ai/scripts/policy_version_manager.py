#!/usr/bin/env python3
import os
import shutil
import sqlite3
import datetime
import subprocess
import rospkg

class PolicyVersionManager:
    def __init__(self):
        rospack = rospkg.RosPack()
        self.pkg_path = rospack.get_path('autocar_ai')
        self.db_path = os.path.join(self.pkg_path, 'database', 'fleet_learning.db')
        self.models_dir = os.path.join(self.pkg_path, 'models')
        os.makedirs(self.models_dir, exist_ok=True)
        self.init_db()

    def init_db(self):
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        cursor.execute("""
        CREATE TABLE IF NOT EXISTS policy_versions (
            version TEXT PRIMARY KEY,
            training_date TEXT,
            dataset_size INTEGER,
            num_episodes INTEGER,
            avg_reward REAL,
            success_rate REAL,
            collision_rate REAL,
            avg_mission_time REAL,
            avg_goal_error REAL,
            model_path TEXT,
            git_commit TEXT,
            training_parameters TEXT
        )
        """)
        conn.commit()
        conn.close()

    def get_git_commit(self):
        try:
            commit = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=self.pkg_path).decode('utf-8').strip()
            return commit
        except Exception:
            return "unknown"

    def register_version(self, version_name, dataset_size=1000, num_episodes=20, avg_reward=0.0,
                         success_rate=0.0, collision_rate=0.0, avg_mission_time=0.0, avg_goal_error=0.0,
                         training_parameters="lr=3e-4,epochs=20"):
        
        # Source weights file
        src_path = os.path.join(self.pkg_path, 'models', 'ppo_policy.pth')
        if not os.path.exists(src_path):
            print(f"[Version Manager] Error: Active policy file not found at {src_path}!")
            return False

        # Target folder for version
        version_dir = os.path.join(self.models_dir, version_name)
        os.makedirs(version_dir, exist_ok=True)
        dest_path = os.path.join(version_dir, 'ppo_policy.pth')

        # Copy model weights
        shutil.copy2(src_path, dest_path)
        git_commit = self.get_git_commit()
        training_date = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")

        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        cursor.execute("""
        INSERT OR REPLACE INTO policy_versions (version, training_date, dataset_size, num_episodes,
        avg_reward, success_rate, collision_rate, avg_mission_time, avg_goal_error, model_path, git_commit, training_parameters)
        VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
        """, (version_name, training_date, dataset_size, num_episodes, avg_reward, success_rate,
              collision_rate, avg_mission_time, avg_goal_error, dest_path, git_commit, training_parameters))
        conn.commit()
        conn.close()

        print(f"[Version Manager] Successfully registered model version '{version_name}' in SQLite database.")
        return True

    def deploy_best_policy(self):
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        cursor.execute("SELECT version, model_path, success_rate, collision_rate FROM policy_versions")
        rows = cursor.fetchall()
        conn.close()

        if not rows:
            print("[Version Manager] No policies registered yet. Cannot deploy.")
            return False

        # Sort: Highest success rate, lowest collision rate
        # Sorting key: success_rate (descending), collision_rate (ascending)
        best_policy = sorted(rows, key=lambda x: (x[2], -x[3]), reverse=True)[0]
        best_version, best_path, best_succ, best_coll = best_policy

        if not os.path.exists(best_path):
            print(f"[Version Manager] Error: Model weights file not found at path {best_path}!")
            return False

        # Copy to active path
        active_path = os.path.join(self.pkg_path, 'models', 'ppo_policy.pth')
        shutil.copy2(best_path, active_path)
        
        print(f"[Version Manager] Successfully deployed '{best_version}' (Success: {best_succ:.2f}, Collisions: {best_coll:.2f}) to active path.")
        return True

    def print_versions(self):
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        cursor.execute("SELECT version, training_date, success_rate, collision_rate, git_commit FROM policy_versions")
        rows = cursor.fetchall()
        conn.close()

        print("\n--- Registered Policy Versions ---")
        if not rows:
            print("No policies registered.")
        for r in rows:
            print(f"Version: {r[0]} | Date: {r[1]} | Success Rate: {r[2]:.2f} | Collision Rate: {r[3]:.2f} | Commit: {r[4][:7]}")
        print("----------------------------------\n")

if __name__ == '__main__':
    import sys
    manager = PolicyVersionManager()
    
    if len(sys.argv) < 2:
        print("Usage:")
        print("  rosrun autocar_ai policy_version_manager.py list")
        print("  rosrun autocar_ai policy_version_manager.py register <version_name>")
        print("  rosrun autocar_ai policy_version_manager.py deploy_best")
        sys.exit(0)

    cmd = sys.argv[1]
    if cmd == 'list':
        manager.print_versions()
    elif cmd == 'register' and len(sys.argv) > 2:
        manager.register_version(sys.argv[2])
    elif cmd == 'deploy_best':
        manager.deploy_best_policy()
    else:
        print("Invalid command.")
