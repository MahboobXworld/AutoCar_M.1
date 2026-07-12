# autocar_navigation

Handles path planning, obstacle avoidance, and dynamic obstacle monitoring for the non-holonomic Ackermann steering vehicle.

## 📂 Structure
- **`config/navigation/`**
  - `costmap_common_params.yaml`: Shared inflation, observation, and obstacle layer parameters.
  - `global_costmap_params.yaml`: Configuration for the global planning costmap.
  - `local_costmap_params.yaml`: Configuration for the local planning costmap.
  - `dwa_local_planner_params.yaml`: DWA local planner settings (velocity limits, cost weights).
  - `teb_local_planner_params.yaml`: TEB local planner settings configured for Ackermann kinematics.
- **`scripts/`**
  - `safety_supervisor_node.py`: Collision avoidance node that intercepts raw planner velocities (`/cmd_vel_raw`) and applies dynamic speed scaling or Estops.
- **`src/`**
  - `ackermann_controller.cpp`: Converts standard diff-drive velocity commands (`cmd_vel`) to Ackermann steering message inputs.
