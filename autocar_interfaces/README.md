# autocar_interfaces

Provides the unified definitions of custom ROS messages (`.msg`), services (`.srv`), and actions (`.action`) used throughout the workspace. Centralizing interface definitions prevents circular dependencies and provides a clean, single source of truth for robot communication.

## 📂 Structure
- **`msg/`**
  - `BehaviorState.msg`: Publishes the active high-level behavior state of the robot.
  - `Maneuver.msg`: Defines a primitive maneuver (type, speed, duration, path).
  - `ManeuverSequence.msg`: Ordered array of maneuvers to execute.
  - `SemanticObject.msg`: Defines a perceived object (id, class, pose, clearance).
  - `SemanticWorldModel.msg`: List of perceived objects in the local area.
  - `Situation.msg`: Describes classified situation/scenarios (e.g., Narrow Passage).
- **`srv/`**
  - `GetRLAction.srv`: Request-response interface between the Behavior Decision Manager and the Policy Inference Server.
- **`action/`**
  - `NavigateBehavior.action`: Main action goal definition for triggering autonomous missions.

## ⚙️ Building
Since other packages depend on these targets, `autocar_interfaces` is configured to build first.
```bash
catkin_make --pkg autocar_interfaces
```
