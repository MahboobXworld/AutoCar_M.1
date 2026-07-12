#!/usr/bin/env python3
import os
import time
import sqlite3
import shutil
import torch
import rospy
import sys
import rospkg
from std_msgs.msg import String
from std_srvs.srv import Trigger
rospack = rospkg.RosPack()
sys.path.append(os.path.join(rospack.get_path('autocar_ai'), 'training'))
from train_ppo import ActorCritic

class FleetSyncDaemon:
    """
    Fleet Synchronization Daemon.
    Periodically:
    1. Uploads local transitions to a mock central fleet repository.
    2. Checks for newly approved global production policies.
    3. Downloads and validates new policy models (via sanity check forward pass).
    4. Triggers hot-reload (/reload_policy service) and synchronizes metadata.
    """
    def __init__(self):
        rospy.init_node('fleet_sync_daemon')
        
        rospack = rospkg.RosPack()
        self.pkg_path = rospack.get_path('autocar_ai')
        self.db_path = os.path.join(self.pkg_path, 'database', 'fleet_learning.db')
        self.production_path = os.path.join(self.pkg_path, 'models', 'ppo_policy.pth')
        
        # Central fleet locations
        self.fleet_dir = os.path.join(self.pkg_path, 'database', 'fleet')
        self.fleet_db_path = os.path.join(self.fleet_dir, 'central_fleet.db')
        self.fleet_policy_path = os.path.join(self.fleet_dir, 'ppo_policy_global.pth')
        
        os.makedirs(self.fleet_dir, exist_ok=True)
        self.init_fleet_database()
        
        # Configuration
        self.sync_interval = rospy.get_param('~sync_interval', 10.0) # 10 seconds for testing
        self.robot_id = rospy.get_param('~robot_id', 'robot_01')
        
        # Publisher to notify policy changes
        self.pub_status = rospy.Publisher('/fleet_sync/status', String, queue_size=10)
        
        rospy.loginfo(f"[FleetSync] Sync Daemon initialized for {self.robot_id}.")

    def init_fleet_database(self):
        """Initializes the central/fleet mock SQLite DB tables if not present."""
        try:
            conn = sqlite3.connect(self.fleet_db_path)
            cursor = conn.cursor()
            cursor.execute("""
            CREATE TABLE IF NOT EXISTS transition_tuples (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                mission_id TEXT,
                step_index INTEGER,
                state_vector TEXT,
                action INTEGER,
                reward REAL,
                next_state_vector TEXT,
                terminal INTEGER,
                novelty_flag INTEGER DEFAULT 0
            )""")
            cursor.execute("""
            CREATE TABLE IF NOT EXISTS policy_sync_log (
                version_id TEXT PRIMARY KEY,
                applied_timestamp REAL,
                status TEXT
            )""")
            conn.commit()
            conn.close()
        except Exception as e:
            rospy.logwarn(f"[FleetSync] Failed to initialize fleet DB: {e}")

    def upload_experiences(self):
        """Syncs unique local database transition tuples to the central fleet database."""
        if not os.path.exists(self.db_path):
            return
        
        try:
            # Connect local and fleet databases
            conn_local = sqlite3.connect(self.db_path)
            cursor_local = conn_local.cursor()
            
            # Fetch local transitions
            cursor_local.execute("SELECT name FROM sqlite_master WHERE type='table' AND name='transition_tuples'")
            if not cursor_local.fetchone():
                conn_local.close()
                return
                
            cursor_local.execute("SELECT id, mission_id, step_index, state_vector, action, reward, next_state_vector, terminal, novelty_flag FROM transition_tuples")
            local_rows = cursor_local.fetchall()
            conn_local.close()
            
            if not local_rows:
                return

            conn_fleet = sqlite3.connect(self.fleet_db_path)
            cursor_fleet = conn_fleet.cursor()
            
            # Retrieve already synced mission_id list to avoid duplicate insertions
            cursor_fleet.execute("SELECT DISTINCT mission_id FROM transition_tuples")
            synced_missions = {r[0] for r in cursor_fleet.fetchall()}
            
            uploaded_count = 0
            for row in local_rows:
                local_id, mid, step, s_vec, action, rew, sn_vec, term, novel = row
                
                # Append robot identifier to mission ID for uniqueness in fleet DB
                fleet_mid = f"{self.robot_id}_{mid}"
                if fleet_mid not in synced_missions:
                    cursor_fleet.execute("""
                        INSERT INTO transition_tuples (mission_id, step_index, state_vector, action, reward, next_state_vector, terminal, novelty_flag)
                        VALUES (?, ?, ?, ?, ?, ?, ?, ?)
                    """, (fleet_mid, step, s_vec, action, rew, sn_vec, term, novel))
                    uploaded_count += 1
            
            conn_fleet.commit()
            conn_fleet.close()
            
            if uploaded_count > 0:
                rospy.loginfo(f"[FleetSync] Successfully uploaded {uploaded_count} unique transitions to central fleet database.")
        except Exception as e:
            rospy.logwarn(f"[FleetSync] Experience upload failed: {e}")

    def validate_policy_weights(self, path):
        """Performs structural and mathematical sanity checks on a downloaded model."""
        if not os.path.exists(path):
            return False
            
        try:
            # Instantiate model and load state_dict
            model = ActorCritic(state_dim=12, action_dim=9)
            state_dict = torch.load(path)
            model.load_state_dict(state_dict)
            model.eval()
            
            # Sanity forward pass check
            test_input = torch.randn(1, 12)
            logits, value = model(test_input)
            
            if logits.shape == (1, 9) and value.shape == (1, 1):
                return True
            else:
                rospy.logwarn(f"[FleetSync] Model structural validation failed for {path}. Shape mismatch: logits={logits.shape}, val={value.shape}")
                return False
        except Exception as e:
            rospy.logwarn(f"[FleetSync] Model weight validation failed for {path}: {e}")
            return False

    def sync_production_policy(self):
        """Downloads newly approved production policies, validates, and activates them."""
        # Check if a new fleet global policy is available
        if not os.path.exists(self.fleet_policy_path):
            return
            
        # Is the fleet policy different or newer than the local one?
        if os.path.exists(self.production_path):
            # Check if files differ (size or modification time)
            local_mtime = os.path.getmtime(self.production_path)
            fleet_mtime = os.path.getmtime(self.fleet_policy_path)
            if fleet_mtime <= local_mtime:
                # No new policy, skip reload
                return

        rospy.loginfo("[FleetSync] New global fleet policy detected. Initiating download and validation...")
        
        # Validate downloaded weights before promotion
        if self.validate_policy_weights(self.fleet_policy_path):
            try:
                # Apply/Deploy to local path
                shutil.copy2(self.fleet_policy_path, self.production_path)
                rospy.loginfo(f"[FleetSync] Downloaded model promoted to local active policy: {self.production_path}")
                
                # Update SQLite sync log
                conn = sqlite3.connect(self.fleet_db_path)
                cursor = conn.cursor()
                version_id = f"fleet_global_{int(os.path.getmtime(self.fleet_policy_path))}"
                cursor.execute("INSERT OR REPLACE INTO policy_sync_log VALUES (?, ?, ?)", (version_id, time.time(), "SUCCESS"))
                conn.commit()
                conn.close()
                
                # Trigger local hot-reload
                rospy.wait_for_service('/reload_policy', timeout=3.0)
                reload_srv = rospy.ServiceProxy('/reload_policy', Trigger)
                resp = reload_srv()
                
                # Publish status update
                status_msg = String()
                status_msg.data = f"FLEET_SYNC_SUCCESS:{version_id}"
                self.pub_status.publish(status_msg)
                
                rospy.loginfo(f"[FleetSync] Hot-reload triggered. Service response: {resp.message}")
            except Exception as e:
                rospy.logwarn(f"[FleetSync] Promotion/Hot-reload failed during sync: {e}")
        else:
            rospy.logerr("[FleetSync] Downloaded global policy failed validation! Keeping current local policy.")
            
            # Log failure in sync log
            try:
                conn = sqlite3.connect(self.fleet_db_path)
                cursor = conn.cursor()
                version_id = f"fleet_global_{int(os.path.getmtime(self.fleet_policy_path))}"
                cursor.execute("INSERT OR REPLACE INTO policy_sync_log VALUES (?, ?, ?)", (version_id, time.time(), "FAILED_VALIDATION"))
                conn.commit()
                conn.close()
            except Exception:
                pass

    def spin(self):
        rate = rospy.Rate(1.0 / self.sync_interval)
        while not rospy.is_shutdown():
            self.upload_experiences()
            self.sync_production_policy()
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                break

if __name__ == '__main__':
    try:
        daemon = FleetSyncDaemon()
        daemon.spin()
    except rospy.ROSInterruptException:
        pass
