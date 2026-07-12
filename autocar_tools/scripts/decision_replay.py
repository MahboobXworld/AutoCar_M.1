#!/usr/bin/env python3
import os
import sqlite3
import sys
import time
import rospkg

class DecisionReplay:
    def __init__(self):
        rospack = rospkg.RosPack()
        self.pkg_path = rospack.get_path('autocar_ai')
        self.db_path = os.path.join(self.pkg_path, 'database', 'fleet_learning.db')

    def replay_mission(self):
        if not os.path.exists(self.db_path):
            print(f"[Replay] Error: SQLite database not found at {self.db_path}")
            return

        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        
        # Check if table exists
        cursor.execute("SELECT name FROM sqlite_master WHERE type='table' AND name='decision_trace'")
        if not cursor.fetchone():
            print("[Replay] Error: No decision_trace table found. Run a mission first.")
            conn.close()
            return

        # Fetch all traces
        cursor.execute("SELECT id, timestamp, robot_pose_x, robot_pose_y, robot_pose_yaw, "
                       "heading_error, cross_track_error, front_clearance, rear_clearance, "
                       "selected_action, confidence, reward, decision_latency FROM decision_trace ORDER BY id ASC")
        rows = cursor.fetchall()
        conn.close()

        if not rows:
            print("[Replay] Error: No decisions recorded in decision_trace table.")
            return

        print(f"\n=======================================================")
        print(f"       DECISION TRACE REPLAY TOOL (Total steps: {len(rows)})")
        print(f"=======================================================")
        print("Press [Enter] for next step, [q] to quit, or [a] to play automatically.")
        
        actions = ['FOLLOW_PATH', 'FORWARD_ALIGNMENT', 'REVERSE_ALIGNMENT', 'U_TURN', 'THREE_POINT_TURN', 'REPLAN', 'RECOVERY', 'GOAL_ALIGNMENT', 'STOP']
        
        auto_play = False
        for idx, row in enumerate(rows):
            tid, timestamp, rx, ry, ryaw, herr, cte, fc, rc, action_idx, confidence, reward, latency = row
            action_name = actions[action_idx] if action_idx < len(actions) else f"UNKNOWN ({action_idx})"

            print(f"\nStep {idx+1}/{len(rows)} | Trace ID: {tid} | Time: {timestamp}")
            print(f"-------------------------------------------------------")
            print(f"  Robot Pose     : x={rx:6.2f}, y={ry:6.2f}, yaw={ryaw:5.2f} rad")
            print(f"  Path Error     : Heading Error={herr:5.2f} rad, CTE={cte:5.2f} m")
            print(f"  Clearances     : Front={fc:4.2f}m, Rear={rc:4.2f}m")
            print(f"  Decision       : Action={action_name} | Confidence={confidence:4.2f}")
            print(f"  Perf Metrics   : Latency={latency*1000:5.2f} ms | Step Reward={reward:6.2f}")
            print(f"-------------------------------------------------------")

            if auto_play:
                time.sleep(0.3)
            else:
                user_input = input("Next step? ")
                if user_input.lower() == 'q':
                    break
                elif user_input.lower() == 'a':
                    auto_play = True

        print("\nReplay finished.")

if __name__ == '__main__':
    replay = DecisionReplay()
    replay.replay_mission()
