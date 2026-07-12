# autocar_bringup

Serves as the unified launch orchestration layer and primary entry point for launching the autonomous Ackermann vehicle system in simulation or physical deployments.

## 📂 Launch Orchestrators
- **`full_system.launch`**: Master launcher aggregating all modular launch fragments.
- **`simulation.launch`**: Launches Gazebo Classic with the specified world type.
- **`localization.launch`**: Launches the Map Server, EKF fusion node, AMCL particle filter, Hector scan-matcher, and LiDAR deskewing.
- **`navigation.launch`**: Loads move_base, costmaps, local planners, and the safety supervisor.
- **`behavior.launch`**: Starts the behavior manager, situation classifier, and mission scheduler.
- **`ai.launch`**: Runs the policy inference server and background synchronization daemons.
- **`perception.launch`**: Runs the semantic environment constructor.

## 🕹️ Convenience World Launches
To run the full system in a specific simulated world layout with pre-configured parameters:
```bash
# Launch inside the warehouse world
roslaunch autocar_bringup warehouse.launch

# Launch inside the campus world twin
roslaunch autocar_bringup campus.launch
```
