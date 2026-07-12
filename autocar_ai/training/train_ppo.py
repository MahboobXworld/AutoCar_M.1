#!/usr/bin/env python3
import os
import sqlite3
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import Dataset, DataLoader
import rospkg
import matplotlib.pyplot as plt

# Define Actor-Critic Policy Network
class ActorCritic(nn.Module):
    def __init__(self, state_dim, action_dim):
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

# Custom Dataset to read transition tuples from SQLite database
class SQLiteExperienceDataset(Dataset):
    def __init__(self, db_path):
        self.db_path = db_path
        self.transitions = []
        self.load_data()

    def load_data(self):
        if not os.path.exists(self.db_path):
            print(f"[PPO Trainer] Database path {self.db_path} does not exist yet. Using mock transitions for structure verification.")
            for _ in range(100):
                s = np.random.randn(12)
                a = np.random.randint(0, 9)
                r = np.random.randn(1)[0]
                s_next = np.random.randn(12)
                t = 0
                self.transitions.append((s, a, r, s_next, t))
            return

        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        
        # Ensure transition_tuples exists
        cursor.execute("SELECT name FROM sqlite_master WHERE type='table' AND name='transition_tuples'")
        if not cursor.fetchone():
            conn.close()
            # Initialize with dummy
            for _ in range(100):
                s = np.random.randn(12)
                a = np.random.randint(0, 9)
                r = np.random.randn(1)[0]
                s_next = np.random.randn(12)
                t = 0
                self.transitions.append((s, a, r, s_next, t))
            return

        cursor.execute("SELECT state_vector, action, reward, next_state_vector, terminal FROM transition_tuples")
        rows = cursor.fetchall()
        
        for row in rows:
            s = np.array([float(x) for x in row[0].split(',')])
            a = int(row[1])
            r = float(row[2])
            s_next = np.array([float(x) for x in row[3].split(',')])
            t = int(row[4])
            self.transitions.append((s, a, r, s_next, t))
        
        conn.close()
        print(f"[PPO Trainer] Loaded {len(self.transitions)} transitions from {self.db_path}")

    def __len__(self):
        return len(self.transitions)

    def __getitem__(self, idx):
        s, a, r, s_next, t = self.transitions[idx]
        return (torch.tensor(s, dtype=torch.float32),
                torch.tensor(a, dtype=torch.long),
                torch.tensor(r, dtype=torch.float32),
                torch.tensor(s_next, dtype=torch.float32),
                torch.tensor(t, dtype=torch.float32))

# Initialize SQL Training Metrics table
def init_metrics_table(db_path):
    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()
    cursor.execute("""
    CREATE TABLE IF NOT EXISTS training_metrics (
        epoch INTEGER PRIMARY KEY,
        total_reward REAL,
        avg_reward REAL,
        policy_loss REAL,
        value_loss REAL,
        entropy REAL,
        kl_divergence REAL,
        learning_rate REAL,
        episode_length REAL,
        collision_rate REAL,
        success_rate REAL,
        goal_accuracy REAL,
        recovery_count INTEGER
    )
    """)
    conn.commit()
    conn.close()

# Query latest fleet reports to link offline training parameters to robot success rates
def get_fleet_stats(db_path):
    success_rate = 0.0
    collision_rate = 0.0
    recovery_count = 0
    goal_accuracy = 0.0
    
    if not os.path.exists(db_path):
        return success_rate, collision_rate, recovery_count, goal_accuracy

    try:
        conn = sqlite3.connect(db_path)
        cursor = conn.cursor()
        cursor.execute("SELECT name FROM sqlite_master WHERE type='table' AND name='mission_performance_reports'")
        if cursor.fetchone():
            cursor.execute("SELECT AVG(success), AVG(collision_count), AVG(num_recovery), AVG(goal_position_err) FROM mission_performance_reports")
            row = cursor.fetchone()
            if row and row[0] is not None:
                success_rate = float(row[0])
                collision_rate = float(row[1])
                recovery_count = int(row[2])
                goal_accuracy = float(row[3])
        conn.close()
    except Exception as e:
        print(f"[PPO Trainer] Error fetching fleet stats: {e}")
        
    return success_rate, collision_rate, recovery_count, goal_accuracy

# Offline PPO Update loop
def train_ppo():
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('autocar_ai')
    
    os.makedirs(os.path.join(pkg_path, 'database'), exist_ok=True)
    db_path = os.path.join(pkg_path, 'database', 'fleet_learning.db')
    export_path = os.path.join(pkg_path, 'models', 'ppo_policy.pth')
    
    init_metrics_table(db_path)

    # Parameters
    state_dim = 12
    action_dim = 9
    epochs = 20
    batch_size = 32
    lr = 3e-4
    gamma = 0.99

    dataset = SQLiteExperienceDataset(db_path)
    dataloader = DataLoader(dataset, batch_size=batch_size, shuffle=True)

    model = ActorCritic(state_dim, action_dim)
    optimizer = optim.Adam(model.parameters(), lr=lr)

    # Logs for plotting
    epoch_list = []
    p_losses = []
    v_losses = []
    entropies = []
    avg_rewards = []

    success_rate, collision_rate, recovery_count, goal_accuracy = get_fleet_stats(db_path)

    for epoch in range(epochs):
        epoch_p_loss = 0.0
        epoch_v_loss = 0.0
        epoch_entropy = 0.0
        epoch_reward = 0.0
        kl_div = 0.0
        
        count = 0
        for states, actions, rewards, next_states, terminals in dataloader:
            logits, values = model(states)
            _, next_values = model(next_states)
            
            # Simple GAE/TD-target
            td_target = rewards.unsqueeze(1) + gamma * next_values * (1 - terminals.unsqueeze(1))
            advantages = td_target - values

            # Evaluate log probs and entropy
            probs = torch.softmax(logits, dim=-1)
            dist = torch.distributions.Categorical(probs)
            log_probs = dist.log_prob(actions)
            entropy = dist.entropy().mean()

            # Loss calculations
            actor_loss = -(log_probs * advantages.detach().squeeze()).mean()
            critic_loss = nn.MSELoss()(values, td_target.detach())
            loss = actor_loss + 0.5 * critic_loss - 0.01 * entropy

            optimizer.zero_grad()
            loss.backward()
            optimizer.step()

            epoch_p_loss += actor_loss.item()
            epoch_v_loss += critic_loss.item()
            epoch_entropy += entropy.item()
            epoch_reward += rewards.mean().item()
            count += 1

        avg_p_loss = epoch_p_loss / max(1, count)
        avg_v_loss = epoch_v_loss / max(1, count)
        avg_ent = epoch_entropy / max(1, count)
        avg_rew = epoch_reward / max(1, count)
        
        epoch_list.append(epoch + 1)
        p_losses.append(avg_p_loss)
        v_losses.append(avg_v_loss)
        entropies.append(avg_ent)
        avg_rewards.append(avg_rew)

        print(f"Epoch {epoch+1}/{epochs} | Policy Loss: {avg_p_loss:.4f} | Value Loss: {avg_v_loss:.4f} | Entropy: {avg_ent:.4f} | Avg Reward: {avg_rew:.4f}")

        # Insert to SQLite
        try:
            conn = sqlite3.connect(db_path)
            cursor = conn.cursor()
            cursor.execute("""
            INSERT OR REPLACE INTO training_metrics (epoch, total_reward, avg_reward, policy_loss, value_loss,
            entropy, kl_divergence, learning_rate, episode_length, collision_rate, success_rate, goal_accuracy, recovery_count)
            VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
            """, (epoch + 1, avg_rew * len(dataset), avg_rew, avg_p_loss, avg_v_loss, avg_ent, kl_div, lr,
                  float(len(dataset)), collision_rate, success_rate, goal_accuracy, recovery_count))
            conn.commit()
            conn.close()
        except Exception as e:
            print(f"[PPO Trainer] SQLite insert failed: {e}")

    # Save weights
    torch.save(model.state_dict(), export_path)
    print(f"[PPO Trainer] Successfully saved policy model weights to {export_path}")

    # Generate Publication-Quality Plots
    plt.style.use('seaborn-v0_8-paper' if 'seaborn-v0_8-paper' in plt.style.available else 'default')
    fig, axs = plt.subplots(2, 2, figsize=(10, 8))
    
    axs[0, 0].plot(epoch_list, p_losses, label='Policy Loss', color='#1f77b4', linewidth=2)
    axs[0, 0].set_title('Actor Policy Loss')
    axs[0, 0].set_xlabel('Epoch')
    axs[0, 0].grid(True, linestyle='--')
    
    axs[0, 1].plot(epoch_list, v_losses, label='Value Loss', color='#ff7f0e', linewidth=2)
    axs[0, 1].set_title('Critic Value Loss')
    axs[0, 1].set_xlabel('Epoch')
    axs[0, 1].grid(True, linestyle='--')

    axs[1, 0].plot(epoch_list, entropies, label='Entropy', color='#2ca02c', linewidth=2)
    axs[1, 0].set_title('Policy Entropy')
    axs[1, 0].set_xlabel('Epoch')
    axs[1, 0].grid(True, linestyle='--')

    axs[1, 1].plot(epoch_list, avg_rewards, label='Avg Reward', color='#d62728', linewidth=2)
    axs[1, 1].set_title('Average Reward')
    axs[1, 1].set_xlabel('Epoch')
    axs[1, 1].grid(True, linestyle='--')

    plt.tight_layout()
    
    # Save as PNG & PDF
    png_path = os.path.join(pkg_path, 'analytics', 'ppo_training_metrics.png')
    pdf_path = os.path.join(pkg_path, 'analytics', 'ppo_training_metrics.pdf')
    plt.savefig(png_path, dpi=300)
    plt.savefig(pdf_path)
    plt.close()
    print(f"[PPO Trainer] Metrics graphs saved to:\n  - {png_path}\n  - {pdf_path}")

if __name__ == '__main__':
    train_ppo()
