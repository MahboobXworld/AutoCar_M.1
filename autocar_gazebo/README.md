# autocar_gazebo

Contains the Gazebo Classic simulation environments, high-fidelity worlds, spawn scripts, and simulation-specific launch files.

## 📂 Structure
- **`launch/`**
  - `car_gazebo.launch`: Main simulation loader which initializes Gazebo, loads a world, and starts joint controllers.
  - `spawn_ackermann.launch`: Helper file to spawn the robot model at a specific $(x, y, yaw)$ position.
  - `campus_world.launch`: Specific launcher for the large-scale campus digital twin environment.
- **`worlds/`**
  - `warehouse.world`: High-density obstacle mapping world.
  - `campus.world`: College campus twin world.
- **`config/`**
  - `gazebo/`: Contains scenarios configuration YAMLs.
- **`models/`**
  - Simulation assets and customized Gazebo building models.
