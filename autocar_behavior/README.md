# autocar_behavior

Implements the high-level task decision and behavior arbitration pipeline of the autonomous vehicle. It controls how the robot switches between path following, alignment, reversing, and recovery modes.

## 📂 Structure
- **`src/`**
  - `behavior_decision_manager_node.cpp`: C++ ROS wrapper for behavior arbitration.
  - `behavior_decision_manager.cpp`: Logic to choose between reinforcement learning (PPO) action outputs and baseline rules.
  - `decision_tree_baseline.cpp`: Hardcoded backup rule-based decision tree when RL confidence drops below a threshold.
- **`include/`**
  - Header files for decision tree nodes and BDM structures.
- **`scripts/`**
  - `situation_classifier_node.py`: Classifies local geometric situations (e.g., Narrow Passage, Open Space) to assist behavior selection.
  - `mission_manager.py`: Python script managing active waypoint schedules.
