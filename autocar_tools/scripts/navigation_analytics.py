#!/usr/bin/env python3
import os
import sqlite3
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import rospkg

class NavigationAnalytics:
    def __init__(self):
        rospack = rospkg.RosPack()
        self.pkg_path = rospack.get_path('autocar_tools')
        self.db_path = os.path.join(rospack.get_path('autocar_ai'), 'database', 'fleet_learning.db')
        self.output_dir = os.path.join(self.pkg_path, 'analytics')
        os.makedirs(self.output_dir, exist_ok=True)

    def load_data(self):
        if not os.path.exists(self.db_path):
            print(f"[Analytics] Warning: SQLite database {self.db_path} not found. Generating dummy reports for analytics output validation.")
            self.generate_dummy_data()

        conn = sqlite3.connect(self.db_path)
        self.df_reports = pd.read_sql_query("SELECT * FROM mission_performance_reports", conn)
        self.df_trace = pd.read_sql_query("SELECT * FROM decision_trace", conn)
        conn.close()

    def generate_dummy_data(self):
        # Creates mockup database for testing if it doesn't exist
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        
        # Create table
        cursor.execute("""
        CREATE TABLE IF NOT EXISTS mission_performance_reports (
            mission_id TEXT PRIMARY KEY, policy_version TEXT, robot_name TEXT, warehouse_map TEXT,
            start_time TEXT, end_time TEXT, duration REAL, success INTEGER, failure_reason TEXT,
            distance_travelled REAL, avg_speed REAL, max_speed REAL, avg_steering REAL, max_steering REAL,
            avg_heading_err REAL, max_heading_err REAL, avg_cross_track_err REAL, max_cross_track_err REAL,
            goal_position_err REAL, goal_orientation_err REAL, num_forward INTEGER, num_reverse INTEGER,
            num_uturn INTEGER, num_threepoint INTEGER, num_replan INTEGER, num_recovery INTEGER,
            collision_count INTEGER, near_collision_count INTEGER, estop_count INTEGER, avg_clearance REAL,
            min_clearance REAL, avg_covariance REAL, max_covariance REAL, energy_estimate REAL,
            total_reward REAL, avg_reward REAL, discounted_return REAL
        )""")
        
        cursor.execute("""
        CREATE TABLE IF NOT EXISTS decision_trace (
            id INTEGER PRIMARY KEY AUTOINCREMENT, timestamp TEXT, robot_pose_x REAL, robot_pose_y REAL,
            robot_pose_yaw REAL, goal_pose_x REAL, goal_pose_y REAL, goal_pose_yaw REAL, heading_error REAL,
            cross_track_error REAL, front_clearance REAL, rear_clearance REAL, left_clearance REAL,
            right_clearance REAL, selected_action INTEGER, confidence REAL, reward REAL, next_state TEXT,
            decision_latency REAL
        )""")
        
        # Insert mockup records
        for i in range(1, 11):
            cursor.execute(f"""
            INSERT OR REPLACE INTO mission_performance_reports VALUES (
                'mission_{i}', 'policy_v{1 if i<6 else 2}', 'autocar_01', 'warehouse_1',
                '171960000{i}', '171960005{i}', {30 + i*2}, {1 if i!=3 else 0}, 'Goal Reached' if i!=3 else 'Timeout',
                {10.0 + i*0.5}, 0.5, 1.2, 0.1, 0.6, 0.05, 0.2, 0.08, 0.25, 0.1, 0.05,
                {20 + i}, {i}, 1, 0, 1, 0, {0 if i!=5 else 1}, 2, 0, 1.5, 0.4, 0.05, 0.15, {15.0 + i},
                {25.0 - i}, 0.8, 12.0
            )""")
            
        conn.commit()
        conn.close()

    def generate_plots(self):
        # 1. Publication-quality Matplotlib figures configuration
        plt.style.use('seaborn-v0_8-paper' if 'seaborn-v0_8-paper' in plt.style.available else 'default')
        fig, axs = plt.subplots(3, 2, figsize=(12, 14))

        # Color Palette
        colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b', '#e377c2', '#7f7f7f', '#bcbd22']

        # A. Mission Success Rate over time (Rolling average)
        self.df_reports['success_roll'] = self.df_reports['success'].rolling(window=3, min_periods=1).mean()
        axs[0, 0].plot(self.df_reports.index + 1, self.df_reports['success_roll'], marker='o', color=colors[0], linewidth=2.5)
        axs[0, 0].set_title('Mission Success Rate (Rolling Window)', fontsize=12, fontweight='bold')
        axs[0, 0].set_xlabel('Mission Run Index')
        axs[0, 0].set_ylabel('Success Rate')
        axs[0, 0].grid(True, linestyle='--')

        # B. Average Reward over time
        axs[0, 1].plot(self.df_reports.index + 1, self.df_reports['total_reward'], marker='s', color=colors[1], linewidth=2.5)
        axs[0, 1].set_title('Total Mission Reward over Time', fontsize=12, fontweight='bold')
        axs[0, 1].set_xlabel('Mission Run Index')
        axs[0, 1].set_ylabel('Total Reward')
        axs[0, 1].grid(True, linestyle='--')

        # C. Collision & Recovery Rates
        x_indices = np.arange(len(self.df_reports))
        axs[1, 0].bar(x_indices - 0.2, self.df_reports['collision_count'], width=0.4, label='Collisions', color=colors[3])
        axs[1, 0].bar(x_indices + 0.2, self.df_reports['num_recovery'], width=0.4, label='Recoveries', color=colors[2])
        axs[1, 0].set_title('Collisions & Recoveries Per Mission', fontsize=12, fontweight='bold')
        axs[1, 0].set_xlabel('Mission Run Index')
        axs[1, 0].set_ylabel('Count')
        axs[1, 0].set_xticks(x_indices)
        axs[1, 0].set_xticklabels(self.df_reports.index + 1)
        axs[1, 0].legend()
        axs[1, 0].grid(True, linestyle='--')

        # D. Heading and Cross Track Error
        axs[1, 1].plot(self.df_reports.index + 1, self.df_reports['avg_cross_track_err'], label='Avg CTE', color=colors[4], marker='^', linewidth=2)
        axs[1, 1].plot(self.df_reports.index + 1, self.df_reports['avg_heading_err'], label='Avg Heading Err', color=colors[5], marker='v', linewidth=2)
        axs[1, 1].set_title('Path Following & Heading Errors', fontsize=12, fontweight='bold')
        axs[1, 1].set_xlabel('Mission Run Index')
        axs[1, 1].set_ylabel('Error (meters / radians)')
        axs[1, 1].legend()
        axs[1, 1].grid(True, linestyle='--')

        # E. Behavior Distribution (Pie Chart of aggregate action frequency)
        actions = ['FOLLOW_PATH', 'FORWARD_ALIGNMENT', 'REVERSE_ALIGNMENT', 'U_TURN', 'THREE_POINT_TURN', 'REPLAN', 'RECOVERY', 'GOAL_ALIGNMENT', 'STOP']
        counts = [
            self.df_reports['num_forward'].sum(),
            0, # FORWARD_ALIGNMENT placeholder
            self.df_reports['num_reverse'].sum(),
            self.df_reports['num_uturn'].sum(),
            self.df_reports['num_threepoint'].sum(),
            self.df_reports['num_replan'].sum(),
            self.df_reports['num_recovery'].sum(),
            0, # GOAL_ALIGNMENT placeholder
            self.df_reports['estop_count'].sum()
        ]
        
        # Filter zero-valued behaviors
        labels = [a for a, c in zip(actions, counts) if c > 0]
        vals = [c for c in counts if c > 0]
        
        if vals:
            axs[2, 0].pie(vals, labels=labels, autopct='%1.1f%%', colors=colors[:len(vals)], startangle=140)
            axs[2, 0].set_title('Overall Behavior Selection Share', fontsize=12, fontweight='bold')
        else:
            axs[2, 0].text(0.5, 0.5, 'No Behavior Data Stored', ha='center', va='center')

        # F. Energy Usage vs Distance Travelled
        axs[2, 1].scatter(self.df_reports['distance_travelled'], self.df_reports['energy_estimate'], color=colors[6], s=100, edgecolors='black')
        axs[2, 1].set_title('Energy Consumption vs Distance', fontsize=12, fontweight='bold')
        axs[2, 1].set_xlabel('Distance Travelled (m)')
        axs[2, 1].set_ylabel('Energy Estimate (Wh)')
        axs[2, 1].grid(True, linestyle='--')

        plt.tight_layout()

        # Save to PNG and PDF
        png_path = os.path.join(self.output_dir, 'fleet_navigation_analytics.png')
        pdf_path = os.path.join(self.output_dir, 'fleet_navigation_analytics.pdf')
        plt.savefig(png_path, dpi=300)
        plt.savefig(pdf_path)
        plt.close()
        print(f"[Analytics] Plots successfully saved to:\n  - {png_path}\n  - {pdf_path}")

    def export_csv(self):
        csv_path = os.path.join(self.output_dir, 'mission_analytics.csv')
        self.df_reports.to_csv(csv_path, index=False)
        print(f"[Analytics] Successfully exported mission data CSV to: {csv_path}")

    def run(self):
        self.load_data()
        self.generate_plots()
        self.export_csv()

if __name__ == '__main__':
    analytics = NavigationAnalytics()
    analytics.run()
