# AutoCar Teleoperation & Remote Control

A ROS Noetic package for remote AutoCar teleoperation via a premium responsive mobile web application. 

This package hosts a local Flask web server that serves a mobile-friendly dashboard containing a virtual touch joystick and control overlay, translating touch drag events into real-time command topics published to `/Throttle_Topic` and `/Steering_Topic` on the ROS master.

---

## 🏗 System Architecture

The package bridges web-based user interaction with ROS middleware:

```
[ Mobile/PC Browser ] 
        │ (10Hz Control Loop / HTTP POST / JSON)
        ▼
[ Flask REST Endpoint ] (Running inside `teleop_node.py`)
        │ (Command translation & verification)
        ▼
[ ROS Topic Publisher ]
        ├── /Throttle_Topic  --> [ Float64MultiArray ]
        └── /Steering_Topic  --> [ Float64MultiArray ]
```

---

## 📂 Package Directory Structure

```
ros_teleop_control_agv/
├── CMakeLists.txt
├── package.xml
├── README.md                # This file
├── TELEOP_REPORT.md         # Detailed technical design report
├── launch/
│   └── teleop.launch        # Starts the teleop receiver node
└── src/
    └── reciver/
        ├── teleop_node.py   # Flask server & ROS node publisher
        └── templates/
            └── index.html   # Web dashboard and control interface
```

---

## 🚀 Getting Started

### 1. Prerequisites
Ensure you have the following installed on your robot/controller:
- ROS Noetic
- Python 3 with `flask` and `flask-cors`
```bash
pip3 install flask flask-cors
```

### 2. Network Configuration
To access the control GUI on your mobile phone:
1. Connect your phone and the robot to the same Wi-Fi network.
2. Find the robot's local IP address (e.g., `192.168.1.106`) by running:
   ```bash
   hostname -I
   ```
3. The Flask web server binds to `0.0.0.0:3000` to accept incoming connections from any device on the local network.

### 3. Launching the Teleoperation Service
Launch the teleop node:
```bash
roslaunch ros_teleop_control_agv teleop.launch
```

### 4. Accessing the GUI
Open a browser (e.g., Google Chrome) on your phone and navigate to:
```
http://<ROBOT_IP>:3000/
```
*(Example: `http://192.168.1.106:3000/`)*

---

## 🔒 Authentication & Single-Operator Mutex

To prevent unauthorized control and safety hazards, the web application incorporates a secure access framework:
* **Credentials**: Username `admin`, Password `password123`.
* **Single-Operator Lock**: Only one operator can control the AutoCar at a time. If a second user attempts to log in or send commands, they are blocked with an HTTP `403 Forbidden` (`busy` status) response.
* **Session Watchdog**: If the active operator closes their browser or loses connectivity, the server automatically releases the lock and engages a safe stop after `2.0` seconds of inactivity.
* **Audit Logging**: All connection activities (successful/failed logins, watchdog timeouts, busy rejections) along with timestamps, client IPs, and device User-Agent details are written to `src/reciver/teleop_connections.log` on the vehicle.

---

## ⚡ Command Slew-Rate Limiting (Ramping)

To protect the AutoCar's physical steering motors and electrical drivetrains, the ROS node implements a dynamic slew-rate limiting filter:
- **Throttle**: Ramped at a maximum rate of change of `20.0` units/second.
- **Steering**: Ramped at a maximum rate of change of `45.0` degrees/second.
- **Manual Override Stops**: Manual mode activation bypasses the ramping filters, bringing the vehicle to an immediate halt.

---

## 🎮 Controller Layout & Controls

- **Switch to Remote Mode**: Toggles the AutoCar between **Manual** mode and **Remote** control mode. Real-time driving commands are ignored unless remote mode is engaged.
- **Virtual Joystick**: Drag the blue circle to steer and drive:
  - **Vertical axis**: Controls throttle direction and magnitude.
  - **Horizontal axis**: Controls steering angle.
- **Max Speed Slider**: Allows limiting the maximum drive speed command (scalable from 0% to 50%, defaults to 20%).
- **Mast Lift Controls**: Press and hold the up/down arrows to raise or lower the mast.

---

## Developer Information
* **Developer**: Mahboob alam
* **Role**: Robotics and AI engineer
* **Email**: [ma.mahboob2002@gmail.com](mailto:ma.mahboob2002@gmail.com)
* **Location**: India
* **Copyright**: Copyright (c) 2026 Mahboob alam. All rights reserved.

