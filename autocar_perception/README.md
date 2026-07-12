# autocar_perception

Processes raw sensor data to construct high-level semantic representations of the robot's local environment.

## 📂 Structure
- **`scripts/`**
  - `semantic_world_model_node.py`: Subscribes to LiDAR scans, segments obstacles, identifies lanes/corridors, and publishes the `/semantic_world_model` message containing clearances and relative obstacle locations.
