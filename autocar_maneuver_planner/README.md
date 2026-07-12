# autocar_maneuver_planner

Contains local trajectory generators and primitive maneuver tracking nodes. It translates high-level decisions (e.g., U-Turn, Reverse Alignment) into specific low-level command velocity profiles.

## 📂 Structure
- **`scripts/`**
  - `maneuver_planner_node.py`: Executes sequential maneuver primitives (reversing, pre-alignment) using PID and geometric tracking.
  - `path_alignment_planner_node.py`: Computes off-angle pre-alignment curves to align the robot's heading with global paths when commencing new navigation missions.
- **`config/`**
  - Maneuver controller default settings.
