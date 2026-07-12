# autocar_ai

Implements the reinforcement learning decision stack, including model files, training scripts, policy deployment managers, and multi-agent fleet synchronizers.

## 📂 Structure
- **`scripts/`**
  - `policy_inference_node.py`: Python service server `/get_rl_action` which evaluates policy outputs using PyTorch. Supports hot-reloading on `/reload_policy`.
  - `online_learning_node.py`: Accumulates transitions and trains candidates in the background.
  - `policy_evaluator.py`: Compares policies against baseline metrics.
  - `policy_version_manager.py`: Command line tool to register, list, and promote models in SQLite.
  - `fleet_sync_daemon.py`: Periodically synchronizes local transitions and global models across the fleet.
- **`training/`**
  - `train_ppo.py`: Offline reinforcement learning training using a PyTorch PPO implementation.
- **`models/`**
  - Path for model weight files (`ppo_policy.pth`).
- **`database/`**
  - Contains SQLite experience database (`fleet_learning.db`) and fleet DB subdirectory.
