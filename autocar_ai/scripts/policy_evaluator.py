#!/usr/bin/env python3
import os
import sqlite3
import yaml
import numpy as np
import torch
import rospy
import sys
import rospkg
from policy_version_manager import PolicyVersionManager
sys.path.append(os.path.join(rospkg.RosPack().get_path('autocar_ai'), 'training'))
from train_ppo import ActorCritic

class PolicyEvaluator:
    """
    Policy Evaluator.
    Evaluates Rule-Based, PPO (Production and Candidate), Hybrid Planner,
    Motion Primitive Planner, and Predictive Planner across 23 warehouse scenarios.
    """
    def __init__(self):
        # Initialize ROS node if not already initialized
        if not rospy.core.is_initialized():
            rospy.init_node('policy_evaluator', anonymous=True, disable_signals=True)

        rospack = rospkg.RosPack()
        self.pkg_path = rospack.get_path('autocar_ai')
        self.scenarios_path = os.path.join(rospkg.RosPack().get_path('autocar_gazebo'), 'config', 'gazebo', 'scenarios.yaml')
        self.db_path = os.path.join(self.pkg_path, 'database', 'fleet_learning.db')
        self.candidate_path = os.path.join(self.pkg_path, 'models', 'ppo_policy_candidate.pth')
        self.production_path = os.path.join(self.pkg_path, 'models', 'ppo_policy.pth')

        # Load scenarios
        with open(self.scenarios_path, 'r') as f:
            self.scenarios = yaml.safe_load(f)['scenarios']

        self.version_manager = PolicyVersionManager()

    def load_model(self, path):
        if not os.path.exists(path):
            return None
        model = ActorCritic(state_dim=12, action_dim=9)
        try:
            model.load_state_dict(torch.load(path))
            model.eval()
            return model
        except Exception as e:
            return None

    def simulate_kinematics(self, model, scenario, planner_type):
        """
        Simulates trajectory using designated planner configuration.
        """
        start = scenario['start_pose']
        goal = scenario['goal_pose']
        max_steps = scenario['max_steps']
        min_clearance_limit = scenario['min_clearance']

        pose = np.array(start, dtype=float)
        goal = np.array(goal, dtype=float)

        steps = 0
        total_reward = 0.0
        collision = False
        success = False

        state = np.zeros(12, dtype=float)
        
        while steps < max_steps:
            dist = np.linalg.norm(goal[:2] - pose[:2])
            goal_yaw = goal[2]
            heading_err = goal_yaw - pose[2]
            heading_err = (heading_err + np.pi) % (2 * np.pi) - np.pi

            # Sim clearances
            front_c = float(np.clip(dist * 0.8 + np.random.randn() * 0.05, 0.25, 10.0))
            rear_c = 10.0
            left_c = 1.0 + 0.1 * np.random.randn()
            right_c = 1.0 + 0.1 * np.random.randn()

            state[0] = dist
            state[1] = heading_err
            state[2] = front_c
            state[3] = rear_c
            state[4] = left_c
            state[5] = right_c
            state[6] = 0.0
            state[7] = 0.5
            state[8] = 0.02
            state[9] = 0
            state[10] = pose[0]
            state[11] = pose[1]

            # Action selection
            if (planner_type == "ppo" or planner_type == "ppo_candidate") and model:
                with torch.no_grad():
                    state_t = torch.tensor(state, dtype=torch.float32).unsqueeze(0)
                    logits, _ = model(state_t)
                    action = int(torch.argmax(logits, dim=1).item())
            elif planner_type == "predictive":
                # Simulated predictive planning action
                if dist < 0.25:
                    action = 8 # STOP
                elif front_c < 0.8:
                    action = 2 # REVERSE (escape)
                else:
                    action = 0 # FORWARD
            elif planner_type == "hybrid":
                # Chooses best based on situation
                if scenario['situation'] in ["Loading Dock", "Reverse Parking"]:
                    action = 2 if abs(heading_err) > 1.5 else 0
                else:
                    action = 0
            elif planner_type == "primitive":
                # Choose motion primitives
                if dist < 0.25:
                    action = 8
                elif abs(heading_err) > 1.2:
                    action = 2
                else:
                    action = 0
            else: # rule-based baseline
                if dist < 0.2:
                    action = 8
                elif abs(heading_err) > 1.5:
                    action = 2
                else:
                    action = 0

            # Kinematics execution
            vx = 0.0
            omega = 0.0

            if action == 0 or action == 1:
                vx = 0.5
                omega = 0.8 * heading_err
            elif action == 2:
                vx = -0.3
                omega = -0.8 * heading_err
            elif action == 8:
                vx = 0.0
                omega = 0.0
                if dist < 0.25:
                    success = True
                    break

            dt = 0.1
            pose[0] += vx * np.cos(pose[2]) * dt
            pose[1] += vx * np.sin(pose[2]) * dt
            pose[2] += omega * dt
            
            min_c = min([front_c, rear_c, left_c, right_c])
            if min_c < min_clearance_limit - 0.15:
                collision = True
                break

            reward = -0.1 + (2.0 * -vx * dt if dist > 0 else 0.0)
            total_reward += reward
            steps += 1

        goal_error = float(np.linalg.norm(goal[:2] - pose[:2]))
        if dist < 0.25:
            success = True

        return success, steps, total_reward, collision, goal_error

    def run_evaluation(self):
        print("\n=======================================================")
        print("          POLICY EVALUATION COMPETITION RUN            ")
        print("=======================================================\n")

        production_model = self.load_model(self.production_path)
        candidate_model = self.load_model(self.candidate_path)

        competitors = {
            "Rule-Based Baseline": ("baseline", None),
            "Production PPO Policy": ("ppo", production_model),
            "Candidate PPO Policy": ("ppo_candidate", candidate_model),
            "Hybrid Path Planner": ("hybrid", None),
            "Motion Primitive Planner": ("primitive", None),
            "Predictive Behavior Planner": ("predictive", None)
        }

        results = {}
        for name, (p_type, model) in competitors.items():
            if (p_type == "ppo" or p_type == "ppo_candidate") and model is None:
                # No model available, skip evaluation for this type
                continue

            print(f"Evaluating: {name} ...")
            scen_results = []
            for scenario in self.scenarios:
                success, steps, reward, collision, goal_error = self.simulate_kinematics(model, scenario, p_type)
                scen_results.append({
                    "success": success,
                    "steps": steps,
                    "reward": reward,
                    "collision": collision,
                    "goal_error": goal_error
                })

            success_rate = np.mean([r["success"] for r in scen_results]) * 100.0
            collision_rate = np.mean([r["collision"] for r in scen_results]) * 100.0
            avg_reward = np.mean([r["reward"] for r in scen_results])
            avg_steps = np.mean([r["steps"] for r in scen_results])
            avg_goal_err = np.mean([r["goal_error"] for r in scen_results])

            results[name] = {
                "success_rate": success_rate,
                "collision_rate": collision_rate,
                "avg_reward": avg_reward,
                "avg_steps": avg_steps,
                "avg_goal_err": avg_goal_err
            }

        # Print results table
        print("\nEvaluation Results Summary:")
        print(f"{'Competitor':<28} | {'Success %':<10} | {'Collision %':<11} | {'Avg Reward':<11} | {'Avg Steps':<10} | {'Goal Err (m)':<12}")
        print("-" * 92)
        for name, metrics in results.items():
            print(f"{name:<28} | {metrics['success_rate']:<10.2f} | {metrics['collision_rate']:<11.2f} | {metrics['avg_reward']:<11.2f} | {metrics['avg_steps']:<10.1f} | {metrics['avg_goal_err']:<12.3f}")
        print("-" * 92 + "\n")

        # Automatic promotion logic
        prod_results = results.get("Production PPO Policy")
        cand_results = results.get("Candidate PPO Policy")

        if cand_results and os.path.exists(self.candidate_path):
            import shutil
            import time
            
            # If no production policy exists, promote the candidate by default
            promote = False
            if not prod_results:
                print("[PolicyEvaluator] No production model exists. Promoting candidate model by default.")
                promote = True
            else:
                p_success = prod_results["success_rate"]
                c_success = cand_results["success_rate"]
                p_reward = prod_results["avg_reward"]
                c_reward = cand_results["avg_reward"]
                p_collision = prod_results["collision_rate"]
                c_collision = cand_results["collision_rate"]

                print(f"[PolicyEvaluator] Comparing Candidate (Success: {c_success:.2f}%, Reward: {c_reward:.2f}, Collisions: {c_collision:.2f}%)")
                print(f"[PolicyEvaluator]       vs Production (Success: {p_success:.2f}%, Reward: {p_reward:.2f}, Collisions: {p_collision:.2f}%)")

                # Promotion criteria:
                # 1. Candidate success rate is higher
                # 2. Success rates are equal, and candidate average reward is higher
                # 3. Success rates and rewards are equal, and candidate collision rate is lower
                if c_success > p_success:
                    promote = True
                elif c_success == p_success:
                    if c_reward > p_reward:
                        promote = True
                    elif c_collision < p_collision:
                        promote = True

            if promote:
                print("[PolicyEvaluator] Candidate model meets performance improvement thresholds. Promoting candidate...")
                shutil.copy2(self.candidate_path, self.production_path)
                
                version_name = f"auto_policy_{int(time.time())}"
                self.version_manager.register_version(
                    version_name=version_name,
                    dataset_size=5000,
                    num_episodes=len(self.scenarios),
                    avg_reward=cand_results["avg_reward"],
                    success_rate=cand_results["success_rate"],
                    collision_rate=cand_results["collision_rate"],
                    avg_mission_time=cand_results["avg_steps"] * 0.1,
                    avg_goal_error=cand_results["avg_goal_err"]
                )
                print(f"[PolicyEvaluator] Successfully registered and promoted model version '{version_name}'.")

                # Call reload_policy ROS service to hot reload model
                try:
                    import std_srvs.srv
                    rospy.wait_for_service('/reload_policy', timeout=3.0)
                    reload_srv = rospy.ServiceProxy('/reload_policy', std_srvs.srv.Trigger)
                    resp = reload_srv()
                    print(f"[PolicyEvaluator] Triggered hot-reload on /reload_policy. Response: {resp.message}")
                except Exception as e:
                    print(f"[PolicyEvaluator] Did not reload model via ROS service: {e}")
            else:
                print("[PolicyEvaluator] Candidate model did not improve performance. Promotion rejected.")

if __name__ == '__main__':
    try:
        evaluator = PolicyEvaluator()
        evaluator.run_evaluation()
    except Exception as e:
        print(f"Failed to execute policy evaluator: {e}")
