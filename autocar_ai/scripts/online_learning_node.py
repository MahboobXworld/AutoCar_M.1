#!/usr/bin/env python3
import os
import sqlite3
import random
import threading
import subprocess
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
import rospy
import rospkg
from autocar_interfaces.msg import BehaviorState
from autocar_interfaces.msg import Situation
from std_msgs.msg import Float64

# Import ActorCritic network definition from train_ppo (or redefine here to be self-contained)
class ActorCritic(nn.Module):
    def __init__(self, state_dim=12, action_dim=9):
        super(ActorCritic, self).__init__()
        self.actor = nn.Sequential(
            nn.Linear(state_dim, 64),
            nn.Tanh(),
            nn.Linear(64, 64),
            nn.Tanh(),
            nn.Linear(64, action_dim)
        )
        self.critic = nn.Sequential(
            nn.Linear(state_dim, 64),
            nn.Tanh(),
            nn.Linear(64, 64),
            nn.Tanh(),
            nn.Linear(64, 1)
        )

    def forward(self, state):
        logits = self.actor(state)
        value = self.critic(state)
        return logits, value

class PrioritizedReplayBuffer:
    def __init__(self, capacity=5000, alpha=0.6):
        self.capacity = capacity
        self.alpha = alpha
        self.buffer = []
        self.pos = 0
        self.priorities = np.zeros((capacity,), dtype=np.float32)

    def push(self, transition, priority):
        max_p = self.priorities.max() if self.buffer else 1.0
        p = max(max_p, priority)
        
        if len(self.buffer) < self.capacity:
            self.buffer.append(transition)
        else:
            self.buffer[self.pos] = transition
        
        self.priorities[self.pos] = p
        self.pos = (self.pos + 1) % self.capacity

    def sample(self, batch_size, beta=0.4):
        if len(self.buffer) == 0:
            return [], [], []
            
        prios = self.priorities[:len(self.buffer)]
        probs = prios ** self.alpha
        probs /= probs.sum()
        
        indices = np.random.choice(len(self.buffer), batch_size, p=probs)
        samples = [self.buffer[idx] for idx in indices]
        
        total = len(self.buffer)
        weights = (total * probs[indices]) ** (-beta)
        weights /= weights.max()
        weights = np.array(weights, dtype=np.float32)
        
        return samples, indices, weights

    def update_priorities(self, batch_indices, batch_priorities):
        for idx, prio in zip(batch_indices, batch_priorities):
            self.priorities[idx] = prio

    def __len__(self):
        return len(self.buffer)

class OnlineLearningNode:
    """
    Online RL Node with Priority Experience Replay, cached SQLite database storage,
    and automatic background model retraining/evaluation triggers.
    """
    def __init__(self):
        rospy.init_node('online_learning_node')

        rospack = rospkg.RosPack()
        self.pkg_path = rospack.get_path('autocar_ai')
        self.db_path = os.path.join(self.pkg_path, 'database', 'fleet_learning.db')
        self.model_path = os.path.join(self.pkg_path, 'models', 'ppo_policy.pth')
        self.candidate_path = os.path.join(self.pkg_path, 'models', 'ppo_policy_candidate.pth')

        # Load active policy to compute value predictions & TD error priorities
        self.state_dim = 12
        self.action_dim = 9
        self.model = ActorCritic(self.state_dim, self.action_dim)
        if os.path.exists(self.model_path):
            try:
                self.model.load_state_dict(torch.load(self.model_path))
                rospy.loginfo(f"[OnlineRL] Loaded weights from {self.model_path}")
            except Exception as e:
                rospy.logwarn(f"[OnlineRL] Failed to load model weights: {e}")
        self.model.eval()
        self.gamma = 0.99

        # Buffers and Caches
        self.buffer = PrioritizedReplayBuffer(capacity=5000)
        self.load_replay_buffer_from_db()
        self.recent_cache = []
        self.cache_lock = threading.Lock()

        # Novelty score and situation tracking
        self.current_situation = "Unknown Situation"
        self.novelty_score = 0.0

        # Sync/Training variables
        self.steps_since_save = 0
        self.steps_since_train = 0
        self.save_threshold = 30  # write to DB every 30 steps
        self.train_threshold = 200 # trigger training every 200 new transitions
        self.is_training = False

        # Subscribers
        self.sub_behavior = rospy.Subscriber('/behavior_state', BehaviorState, self.behavior_callback)
        self.sub_situation = rospy.Subscriber('/navigation_situation', Situation, self.situation_callback)
        self.sub_novelty = rospy.Subscriber('/novelty_score', Float64, self.novelty_callback)

        self.last_state_msg = None

        rospy.loginfo("[OnlineRL] Node initialized and ready.")

    def load_replay_buffer_from_db(self):
        """Loads the latest transitions from SQLite to initialize the PER buffer."""
        if not os.path.exists(self.db_path):
            rospy.loginfo("[OnlineRL] Database does not exist yet. Starting with an empty replay buffer.")
            return
        
        try:
            conn = sqlite3.connect(self.db_path)
            cursor = conn.cursor()
            # Ensure transition_tuples table exists
            cursor.execute("SELECT name FROM sqlite_master WHERE type='table' AND name='transition_tuples'")
            if not cursor.fetchone():
                conn.close()
                rospy.loginfo("[OnlineRL] Table transition_tuples does not exist. Starting with an empty buffer.")
                return

            cursor.execute("""
                SELECT state_vector, action, reward, next_state_vector, terminal 
                FROM transition_tuples 
                ORDER BY id DESC LIMIT ?
            """, (self.buffer.capacity,))
            rows = cursor.fetchall()
            conn.close()

            if not rows:
                rospy.loginfo("[OnlineRL] No previous transitions found in SQLite.")
                return

            rospy.loginfo(f"[OnlineRL] Loading {len(rows)} transitions from SQLite to initialize replay buffer...")
            
            # Push in chronological order (reverse of retrieved ORDER BY id DESC)
            for row in reversed(rows):
                s = np.array([float(x) for x in row[0].split(',')])
                a = int(row[1])
                r = float(row[2])
                s_next = np.array([float(x) for x in row[3].split(',')])
                t = int(row[4])
                
                # Compute approximate TD-error priority
                with torch.no_grad():
                    s_t = torch.tensor(s, dtype=torch.float32).unsqueeze(0)
                    sn_t = torch.tensor(s_next, dtype=torch.float32).unsqueeze(0)
                    _, v_s = self.model(s_t)
                    _, v_sn = self.model(sn_t)
                    td_error = float(abs(r + self.gamma * v_sn.item() * (1 - t) - v_s.item()))
                priority = td_error + 1e-5
                self.buffer.push((s, a, r, s_next, t), priority)
                
            rospy.loginfo(f"[OnlineRL] Replay buffer successfully warmed up with {len(self.buffer)} transitions.")
        except Exception as e:
            rospy.logwarn(f"[OnlineRL] Failed to load replay buffer from database: {e}")

    def situation_callback(self, msg):
        self.current_situation = msg.situation

    def novelty_callback(self, msg):
        self.novelty_score = msg.data

    def compute_adaptive_reward(self, state_msg, prev_state_msg):
        """
        Phase 10 / Context-Dependent Adaptive Reward Function.
        """
        dist_change = prev_state_msg.distance_to_goal - state_msg.distance_to_goal
        r_progress = 2.0 * dist_change
        
        min_clearance = min([state_msg.front_clearance, state_msg.rear_clearance, state_msg.left_clearance, state_msg.right_clearance])
        r_safety = -1.0 * np.exp(-min_clearance)
        
        # Base penalty for collision
        p_collision = -10.0 if min_clearance < 0.20 else 0.0
        p_timeout = -0.1

        # Adaptive terms based on situation
        r_adaptive = 0.0
        if self.current_situation == "Loading Dock":
            # Higher reward for alignment accuracy
            r_adaptive = 3.0 * np.cos(state_msg.heading_error) if state_msg.distance_to_goal < 1.5 else 0.0
        elif self.current_situation == "Wide Open Area":
            # Reward higher speed
            r_adaptive = 2.0 * state_msg.current_velocity
        elif self.current_situation == "Narrow Corridor":
            # Higher safety penalty, reward center alignment
            r_adaptive = -2.0 * np.exp(-min_clearance) - 1.0 * abs(state_msg.left_clearance - state_msg.right_clearance)
        elif self.current_situation == "Dead End":
            # Reward progress out of dead end / efficient recovery
            r_adaptive = 1.5 * dist_change if dist_change > 0 else -0.5
        elif self.current_situation == "Dynamic Obstacle Ahead":
            # Reward smooth avoidance (steering minimized, clearance kept high)
            r_adaptive = -0.5 * abs(state_msg.current_steering_angle) + 1.0 * min_clearance

        return r_progress + r_safety + p_collision + p_timeout + r_adaptive

    def behavior_callback(self, msg):
        if self.last_state_msg is None:
            self.last_state_msg = msg
            return

        # Calculate reward
        reward = self.compute_adaptive_reward(msg, self.last_state_msg)
        terminal = 1 if msg.last_action == 8 else 0

        # Construct transitions
        s = np.array(self.last_state_msg.state_vector)
        s_next = np.array(msg.state_vector)
        a = msg.last_action

        # Calculate live TD-error to set PER priority
        with torch.no_grad():
            s_t = torch.tensor(s, dtype=torch.float32).unsqueeze(0)
            sn_t = torch.tensor(s_next, dtype=torch.float32).unsqueeze(0)
            _, v_s = self.model(s_t)
            _, v_sn = self.model(sn_t)
            td_error = float(abs(reward + self.gamma * v_sn.item() * (1 - terminal) - v_s.item()))

        # If novelty is high or situation is Unknown, boost priority as requested by Phase 8
        priority = td_error + 1e-5
        if self.novelty_score > 0.6 or self.current_situation == "Unknown Situation":
            priority *= 5.0 # Boost replay priority for novel transitions

        # Push to PER and Cache
        transition = (s, a, reward, s_next, terminal)
        self.buffer.push(transition, priority)

        with self.cache_lock:
            self.recent_cache.append((self.last_state_msg.header.stamp.to_sec(), s, a, reward, s_next, terminal))

        self.steps_since_save += 1
        self.steps_since_train += 1

        # Check triggers
        if self.steps_since_save >= self.save_threshold:
            self.flush_cache_to_sqlite()
            self.steps_since_save = 0

        if self.steps_since_train >= self.train_threshold and not self.is_training:
            self.is_training = True
            threading.Thread(target=self.run_background_training).start()
            self.steps_since_train = 0

        self.last_state_msg = msg

    def flush_cache_to_sqlite(self):
        """
        Phase 7 / Automatic Dataset Growth. Writes cached transitions to SQL.
        """
        with self.cache_lock:
            if not self.recent_cache:
                return
            
            try:
                conn = sqlite3.connect(self.db_path)
                cursor = conn.cursor()
                
                # Check table existence
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

                for timestamp, s, a, r, s_next, t in self.recent_cache:
                    s_str = ",".join([str(x) for x in s])
                    sn_str = ",".join([str(x) for x in s_next])
                    mid = f"online_{int(timestamp)}"
                    
                    # Novelty flag based on current detector status
                    novel_flag = 1 if self.novelty_score > 0.6 else 0

                    cursor.execute("""
                    INSERT INTO transition_tuples (mission_id, step_index, state_vector, action, reward, next_state_vector, terminal, novelty_flag)
                    VALUES (?, ?, ?, ?, ?, ?, ?, ?)
                    """, (mid, 0, s_str, int(a), float(r), sn_str, int(t), novel_flag))

                conn.commit()
                conn.close()
                rospy.loginfo(f"[OnlineRL] Flushed {len(self.recent_cache)} transitions to SQL database.")
                self.recent_cache = []
            except Exception as e:
                rospy.logwarn(f"[OnlineRL] Failed to write cache to database: {e}")

    def run_background_training(self):
        """
        Scheduled background retraining of PPO model.
        """
        rospy.loginfo("[OnlineRL] Scheduled retraining triggered. Running training in background...")
        try:
            # We trigger the train_ppo script to produce a candidate pth file.
            # To ensure it doesn't overwrite ppo_policy.pth directly, we can pass parameter or copy it.
            # Let's execute train_ppo.py and then copy it to ppo_policy_candidate.pth.
            script_path = os.path.join(self.pkg_path, 'scripts', 'train_ppo.py')
            result = subprocess.run(['python3', script_path], stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=True)
            
            if os.path.exists(self.model_path):
                # Copy train output to candidate
                import shutil
                shutil.copy2(self.model_path, self.candidate_path)
                rospy.loginfo(f"[OnlineRL] Background PPO Training Completed. Saved to {self.candidate_path}")
                
                # Enqueue the policy for evaluation
                self.evaluate_candidate_policy()
        except Exception as e:
            rospy.logwarn(f"[OnlineRL] Background training failed: {e}")
        finally:
            self.is_training = False

    def evaluate_candidate_policy(self):
        """
        Policy Evaluation Queue.
        Runs all benchmark scenarios to compare:
        - Rule-Based Baseline
        - Previous Production Version
        - Candidate PPO Version
        """
        rospy.loginfo("[OnlineRL] Launching policy evaluation competition...")
        try:
            # Spawn policy_evaluator.py script
            eval_script = os.path.join(self.pkg_path, 'scripts', 'policy_evaluator.py')
            if os.path.exists(eval_script):
                subprocess.run(['python3', eval_script], check=True)
                rospy.loginfo("[OnlineRL] Policy evaluation completed successfully.")
            else:
                rospy.logwarn(f"[OnlineRL] Policy evaluator script not found at {eval_script}")
        except Exception as e:
            rospy.logwarn(f"[OnlineRL] Policy evaluation competition failed: {e}")

if __name__ == '__main__':
    try:
        node = OnlineLearningNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
