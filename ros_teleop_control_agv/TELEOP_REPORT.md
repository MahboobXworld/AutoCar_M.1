# Technical Design & Implementation Report: AGV Remote Teleoperation

This report outlines the technical design, protocols, mapping configurations, and implementation details for the `ros_teleop_control_agv` package. It details how the system achieves low-latency remote command transmission while enforcing authorization and session controls.

---

## 1. System Overview

The `ros_teleop_control_agv` package implements an end-to-end teleoperation system allowing a web client (e.g., a smartphone connected to the local Wi-Fi network) to drive an Automated Guided Vehicle (AGV) and control its mast lift.

The architecture comprises three main layers:
1. **PWA User Interface**: A responsive HTML5/CSS/JavaScript web dashboard featuring a virtual touch joystick and real-time telemetry.
2. **Flask REST Server**: Embedded inside the ROS node, processing incoming JSON payloads and mapping commands.
3. **ROS Node Interface**: Publishes translated drive parameters to `/Throttle_Topic` and `/Steering_Topic` at 10Hz.

---

## 2. Technical Specifications & Protocols

### A. Touch Vector Mapping (Joystick)
The virtual joystick calculates touch coordinate translations relative to the center of its container element:
- **Polar Coordinate Calculations**:
  - Distance $d = \sqrt{dx^2 + dy^2}$
  - Angle $\theta = \operatorname{atan2}(-dy, dx)$
- **Clipping Boundaries**: The drag boundary is capped at a maximum radius ($R = 72\text{px}$). If $d > R$, coordinates are scaled back onto the circle perimeter.
- **Drive Vector Mappings**:
  - **Throttle**: Calculated from the normalized vertical offset. Capped by the `maxSpeed` slider value:
    $$\text{Throttle Value} = \left(-\frac{dy}{R}\right) \times \text{maxSpeed}$$
    *(Range: $[-50.0, 50.0]$)*
  - **Steering**: Map horizontal deflection to angle degrees where $90^\circ$ is straight, $45^\circ$ is hard left, and $135^\circ$ is hard right:
    $$\text{Steering Value} = 90.0 + \left(\frac{dx}{R} \times 45.0\right)$$
    *(Range: $[45.0, 135.0]$)*

### B. High-Frequency REST API
The client browser runs a `setInterval` loop executing at 10Hz (every 100ms), making HTTP `POST` requests to:
`POST /bot/updateoperation/CL-00001/LOC-00000001/BOT-000001`

#### JSON Request Schema:
```json
{
  "operation_details": {
    "throttle": 0.0,
    "steering": 90.0,
    "mast": 0.0
  },
  "manual_switch": false
}
```
*Note: `manual_switch` toggle is omitted from the 10Hz stream and is only included when the user performs explicit state-change clicks.*

#### JSON Response Schema:
```json
{
  "status": "received",
  "manual_switch": true,
  "mast_virtual_pos": 0.0
}
```

---

## 3. Concurrency, Security, & Client Controls

### A. Client Session Validation
To prevent unauthenticated browser tabs (e.g. at the login screen) from transmitting default override values that block other clients, a rigorous check is enforced in the JavaScript control loop:
```javascript
if (sessionStorage.getItem('authenticated') !== 'true') {
  return;
}
```
Only when the user successfully completes authentication does the client begin REST transmissions.

### B. Forced Login on Launch
To prevent session hijacking or bypass upon page refresh, the dashboard explicitly clears cached authentication items on initialization:
```javascript
sessionStorage.removeItem('authenticated');
checkAuthentication();
```
This guarantees that every new connection or page refresh prompts the user for login credentials.

### C. Mobile Keyboard Input Protections
To prevent mobile keyboards from modifying inputs (e.g., auto-capitalizing the username to `Admin` instead of `admin` or adding trailing spaces during autocompletion), the input fields are guarded with explicit parameters:
```html
<input type="text" id="username" autocapitalize="none" autocorrect="off" spellcheck="false">
```
Additionally, JavaScript normalizes values before authentication:
```javascript
const user = document.getElementById('username').value.trim().toLowerCase();
```

### D. Exclusive Session Mutex & Connection Audit Logging
To prevent mechanical damage and safety hazards caused by multiple operators sending conflicting drive vectors simultaneously, the system enforces a strict single-active-operator mutual exclusion policy:
1. **Dynamic Session Tokens**: Upon successful authentication at `/bot/login`, the server generates a unique UUID-based session token. The client stores this in `sessionStorage` and attaches it as `session_token` in all subsequent 10Hz command payloads.
2. **Mutual Exclusion Lock**: The active operator holds a server-side lock. Any subsequent operator attempting to log in or transmit commands will be rejected with an HTTP `403 Forbidden` (`busy` status) and kicked back to the login screen.
3. **Automatic Watchdog Release**: If the active operator closes their tab or loses network connectivity, the server's watchdog timer triggers after `2.0` seconds of inactivity. This automatically:
   - Releases the active session lock.
   - Forces the vehicle into **Manual Override/Safe Stop Mode** (`rosparam set /manual_switch true`).
4. **Audit Logger**: The server maintains a persistent local log at `teleop_connections.log` recording all security and connection events:
   - **Timestamp**: Exact event local time.
   - **Event Type**: `LOGIN_SUCCESS`, `LOGIN_FAILED`, `LOGIN_BUSY`, `COMMAND_REJECTED_BUSY`, `COMMAND_REJECTED_UNAUTHORIZED`, or `WATCHDOG_DISCONNECT`.
   - **Operator Identification**: Username, Session UUID.
   - **Device & Origin Details**: Client IP address and browser User-Agent string.

---

## 4. ROS Topic Mappings

Drive commands are translated and published on the following ROS topics:

1. **`/Throttle_Topic`** (`std_msgs/Float64MultiArray`)
   - Payload: `[throttle_command, direction]`
   - Direction values: `1.0` (Forward), `2.0` (Reverse)
   - Magnitude: Absolute throttle value scaled by the max speed limit.

2. **`/Steering_Topic`** (`std_msgs/Float64MultiArray`)
   - Payload: `[steering_command, steering_direction]`
   - Straight: `[0.0, 1.0]`
   - Left: Steering angle offset relative to straight, direction `1.0`
   - Right: Steering angle offset relative to straight, direction `2.0`

---

## 5. Verification & Diagnostics

To verify system operations, standard ROS debugging commands can be executed:
- **Monitor Command Reception**:
  ```bash
  rostopic echo /Throttle_Topic
  rostopic echo /Steering_Topic
  ```
- **Inspect Flask Server Output**:
  Check `/home/mahboob-alam/.ros/log/latest/teleop_receiver-*.log` to view Werkzeug HTTP server response status codes and incoming payloads.

---

## Developer Information
* **Developer**: Mahboob alam
* **Role**: Robotics and AI engineer
* **Email**: [ma.mahboob2002@gmail.com](mailto:ma.mahboob2002@gmail.com)
* **Location**: India
* **Copyright**: Copyright (c) 2026 Mahboob alam. All rights reserved.

