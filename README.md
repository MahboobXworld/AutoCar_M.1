# ROS Ackermann Steering Robot: Simulation, Localization & Navigation Stack

This repository contains the modularized ROS workspace for a 4-wheeled Ackermann steering robot (`autocar`) configured for physics-based simulation in Gazebo, sensor-fused state estimation, global localization, and autonomous path planning.

---

## 🛠️ System Architecture & Node Graph
The navigation pipeline intercepts user navigation goals using the **Path Alignment & Maneuver Planner** to pre-align the Ackermann steering vehicle. Control is then passed to the **Behavior Decision Manager (BDM)** which acts as an arbitrator between mission goals and `move_base`. A C++ controller translates planner commands to wheel speed and knuckle position controls. The evaluation framework records telemetry, traces decisions, logs training runs, and evaluates policies:

```mermaid
graph TD
    User[RViz / User Goal] -->|NavigateBehaviorAction| PAP[Path Alignment & Maneuver Planner]
    PAP -->|NavigateBehaviorAction (BDM)| BDM[Behavior Decision Manager]
    BDM -->|Goal Pose| MoveBase[move_base Planner]
    BDM -->|/behavior_state| ExpLogger[Experience Logger]
    BDM -->|/behavior_state| PerfLogger[Mission Performance Logger]
    BDM -->|/behavior_state| TraceLogger[Decision Trace Logger]
    
    UserGoal2D[RViz 2D Nav Goal] -->|/move_base_simple/goal| VizNode[Visualizer Node]
    VizNode -->|/parking_spot_marker| RViz[RViz Display]
    
    ExpLogger -->|Save Tuples| SQLite[(SQLite db: fleet_learning.db)]
    PerfLogger -->|Save Reports| SQLite
    TraceLogger -->|Save Traces & Fallbacks| SQLite
    
    SQLite -->|Load Tuples| PPO[train_ppo.py]
    PPO -->|Export Weights| ModelWeights[ppo_policy.pth]
    ModelWeights -->|Register / Version| VerManager[policy_version_manager.py]
    VerManager -->|Deploy Active Weights| ActiveWeights[ppo_policy.pth]
    
    ActiveWeights -->|Load Model| InferenceNode[policy_inference_node]
    InferenceNode -->|/get_rl_action Service| BDM
    
    SQLite -->|Generate Visualizations| Analytics[navigation_analytics.py]
    SQLite -->|Replay Step-by-Step| Replay[decision_replay.py]
    
    Gazebo[Gazebo Simulation] -->|/scan| BDM
    Gazebo -->|/imu/data| EKF[EKF Localization Node]
    Gazebo -->|/joint_states| AckermannController[C++ Ackermann Controller]
    
    AckermannController -->|/diff_drive_controller/odom| EKF
    EKF -->|/odometry/filtered| BDM
    
    MoveBase -->|/cmd_vel| AckermannController
    AckermannController -->|Actuators Control| Gazebo
```

---

## 📂 Modular Project Structure
The repository is split into **11 decoupled ROS packages**:

```text
four_wheel_drive/
└── src/
    ├── autocar_interfaces/          # Custom messages, services, and actions
    │   ├── action/
    │   │   └── NavigateBehavior.action
    │   ├── msg/
    │   │   ├── BehaviorState.msg
    │   │   ├── Situation.msg
    │   │   ├── SemanticWorldModel.msg
    │   │   ├── SemanticObject.msg
    │   │   └── ManeuverSequence.msg
    │   └── srv/
    │       └── GetRLAction.srv
    │
    ├── autocar_description/         # URDF, mesh assets, and RViz configurations
    │   ├── meshes/
    │   ├── rviz/
    │   └── urdf/
    │       └── car.xacro
    │
    ├── autocar_gazebo/              # Simulation worlds and launch fragments
    │   ├── config/
    │   ├── launch/
    │   └── worlds/
    │
    ├── autocar_localization/        # State estimators, SLAM, and deskewing
    │   ├── config/
    │   ├── maps/
    │   └── scripts/
    │       ├── lidar_deskew_node.py
    │       ├── localization_manager_node.py
    │       └── benchmark_localization.py
    │
    ├── autocar_navigation/          # Move_base costmaps & local/global planners
    │   ├── config/
    │   ├── scripts/
    │   │   └── safety_supervisor_node.py
    │   └── src/
    │       └── ackermann_controller.cpp
    │
    ├── autocar_behavior/            # High-level arbitration & waypoint trackers
    │   ├── scripts/
    │   │   └── mission_manager.py
    │   └── src/
    │       ├── behavior_decision_manager.cpp
    │       ├── behavior_decision_manager_node.cpp
    │       └── decision_tree_baseline.cpp
    │
    ├── autocar_maneuver_planner/    # Movement tracking, pre-alignment maneuvers
    │   └── scripts/
    │       ├── maneuver_planner_node.py
    │       └── path_alignment_planner_node.py
    │
    ├── autocar_ai/                  # Reinforcement Learning & Database logs
    │   ├── database/
    │   │   └── fleet_learning.db    # Central SQLite database
    │   ├── models/
    │   │   └── ppo_policy.pth       # Active PyTorch weights
    │   ├── scripts/
    │   │   ├── policy_inference_node.py
    │   │   ├── policy_version_manager.py
    │   │   ├── online_learning_node.py
    │   │   └── fleet_sync_daemon.py
    │   └── training/
    │       └── train_ppo.py
    │
    ├── autocar_perception/          # Semantic environment reconstruction
    │   └── scripts/
    │
    ├── autocar_tools/               # Performance evaluations & replays
    │   ├── scripts/
    │   │   ├── navigation_analytics.py
    │   │   └── decision_replay.py
    │   └── src/
    │       ├── experience_logger_node.cpp
    │       ├── mission_performance_logger.cpp
    │       └── decision_trace_logger.cpp
    │
    └── autocar_bringup/             # Centralized system launch orchestrators
        └── launch/
            ├── full_system.launch
            ├── warehouse.launch
            └── campus.launch
```

---

## ⚙️ Prerequisites & Dependencies
Ensure you have the following packages installed on your ROS Noetic machine (Ubuntu 20.04):
```bash
sudo apt-get update
sudo apt-get install ros-noetic-desktop-full \
                     ros-noetic-gazebo-ros-control \
                     ros-noetic-navigation \
                     ros-noetic-robot-localization \
                     ros-noetic-teb-local-planner \
                     ros-noetic-teleop-twist-keyboard \
                     sqlite3 libsqlite3-dev
pip3 install torch torchvision onnx pandas matplotlib
```

---

## 🚀 Installation & Build Guide

1. **Clone the Repository** and place the `src` folder or contents into your ROS workspace:
   ```bash
   mkdir -p ~/four_wheel_drive/src
   cd ~/four_wheel_drive/src
   git clone https://github.com/MahboobXworld/AutoCar_M.1.git .
   ```
2. **Build the Workspace**:
   ```bash
   cd ~/four_wheel_drive
   catkin_make
   ```
3. **Source the Environment**:
   ```bash
   source devel/setup.bash
   ```

---

## 🕹️ How to Run

### 1. Launch Simulation and Navigation Stack
Launch the overall system using `autocar_bringup`. This spins up the Gazebo simulation (defaults to the warehouse world), joint state publishers, EKF fusion, scan matcher, AMCL localization, and `move_base`.
* **TEB Planner (Recommended for Ackermann constraints):**
  ```bash
  roslaunch autocar_bringup full_system.launch local_planner:=teb
  ```
* **DWA Planner:**
  ```bash
  roslaunch autocar_bringup full_system.launch local_planner:=dwa
  ```

To launch with RViz and without Gazebo GUI (headless simulation):
```bash
roslaunch autocar_bringup full_system.launch local_planner:=teb gui:=false rviz:=true
```

### 2. Launch Behavior Decision Manager & Loggers
The Decision Manager, policy server, and SQLite loggers are automatically launched via `full_system.launch`. You can customize arbitration parameters:
* **Option A: C++ Baseline Rules (Default)**
  ```bash
  roslaunch autocar_bringup full_system.launch use_rl:=false
  ```
* **Option B: Reinforcement Learning (PPO with Confidence Fallback)**
  ```bash
  roslaunch autocar_bringup full_system.launch use_rl:=true confidence_threshold:=0.4
  ```

### 3. Run Offline RL PPO Training
Trains the policy offline using transitions stored in the SQLite database, writes epoch progress metrics to SQLite, and exports training curves:
```bash
rosrun autocar_ai train_ppo.py
```

### 4. Manage Model Versions
* **List Registered Versions**:
  ```bash
  rosrun autocar_ai policy_version_manager.py list
  ```
* **Register a New Model Version**:
  ```bash
  rosrun autocar_ai policy_version_manager.py register v1
  ```
* **Deploy the Best Policy** (based on success rates):
  ```bash
  rosrun autocar_ai policy_version_manager.py deploy_best
  ```

### 5. Run Automated Benchmarking / Evaluation
Runs automated scenarios to compare the baseline decision tree against production and candidate PPO policies, triggering automatic promotion if performance thresholds are met:
```bash
rosrun autocar_ai policy_evaluator.py
```

### 6. Run Continuous Fleet Sync Daemon
Launches the background synchronization daemon to merge local transition databases, download new approved global policies, validate weights, and trigger hot-reloading:
```bash
rosrun autocar_ai fleet_sync_daemon.py
```

### 7. Generate Analytics Dashboard
Creates publication-quality Matplotlib figures of success rate, cross-track error, collision frequency, energy usage, and behavior shares:
```bash
rosrun autocar_tools navigation_analytics.py
```

### 8. Interactive Step-by-Step Replay
Launches the CLI tool to playback logged decision traces step-by-step:
```bash
rosrun autocar_tools decision_replay.py
```

### 9. Run Localization Benchmarks & Diagnostics
Starts the automated localization stress-testing suite (High-Speed Straight, Aggressive Turns, Reverse maneuvers, and Figure-Eight paths), calculates RMSE tracking errors, logs to SQLite, and generates a markdown performance report:
```bash
rosrun autocar_localization benchmark_localization.py
```
* **Performance Report:** The generated report is saved at `autocar_localization/config/analytics/localization_benchmark_report.md`.
* **Health Visualizer:** Real-time health status, confidence metrics, and wheel slip warnings can be viewed directly in RViz on the `/localization/health_marker` topic.
