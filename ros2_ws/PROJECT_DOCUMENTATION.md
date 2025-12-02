# AutoCharge EV Robot: Technical Documentation & System Analysis

## 1. Executive Summary

**Project Name:** AutoCharge EV Robot  
**Type:** Autonomous Mobile Robot (AMR) for Electric Vehicle Charging  
**Platform:** ROS 2 (Robot Operating System) on Raspberry Pi 4  

The **AutoCharge EV Robot** is an autonomous mobile system designed to navigate parking infrastructure, locate electric vehicles, and automatically dock with them to provide charging services. It eliminates the need for fixed charging stations at every parking spot, allowing for flexible, on-demand charging.

**Key Capabilities:**
*   **Omnidirectional Mobility:** Uses mecanum wheels to move in any direction (holonomic drive).
*   **Autonomous Navigation:** Uses SLAM (Simultaneous Localization and Mapping) and the Nav2 stack to navigate dynamic environments.
*   **Precision Docking:** Utilizes computer vision and AprilTags to align with charging ports with millimeter precision.
*   **Web-Based Control:** Features a comprehensive Mission Control Dashboard for remote monitoring, mapping, and operation.
*   **Safety Systems:** Integrated emergency stops, collision avoidance, and system health monitoring.

---

## 2. System Architecture

The system is built on a layered architecture separating hardware abstraction, control logic, and user interaction.

### 2.1 Hardware Layer
*   **Compute:** Raspberry Pi 4 (Ubuntu Server with ROS 2).
*   **Drive System:** 4x DC Motors with Encoders + Custom Motor Controller Board (I2C address `0x34`).
*   **Sensors:** 
    *   **LiDAR:** YDLiDAR X2 (360° scanning for mapping/navigation).
    *   **Camera:** USB Camera (AprilTag detection).
    *   **Encoders:** Wheel feedback for odometry.
*   **Power:** Battery management system with voltage monitoring.

### 2.2 Software Layer (ROS 2)
The software is organized into modular ROS 2 packages:

| Package | Purpose | Key Nodes |
| :--- | :--- | :--- |
| **`robot`** | Core system orchestration | `mode_manager_node`, `state_manager` |
| **`motor`** | Hardware abstraction | `mecanum_controller`, `mecanum_odometry` |
| **`ev_docking`** | Vision & Autonomy | `docking_controller`, `camera_publisher`, `system_monitor` |
| **`interfaces`** | Custom Data Types | Service (`.srv`) definitions |
| **`ydlidar_ros2_driver`** | Sensor Driver | `ydlidar_ros2_driver_node` |

### 2.3 Interface Layer
*   **Web Dashboard:** Node.js/Express application serving a responsive UI.
*   **Communication:** `rosbridge_server` (WebSocket) connects the browser to the ROS 2 core.

---

## 3. Startup & Operation

### 3.1 Standard Launch Sequence
To bring the full system online, execute the following commands in separate terminals:

**1. Hardware & Core Systems**
Initializes motors, LiDAR, odometry, and robot description.
```bash
ros2 launch robot bringup_all.launch.py
```

**2. Application Layer & Web UI**
Starts the mode manager, state manager, web server, and ROS bridge.
```bash
ros2 launch robot mode_manager.launch.py
```

**3. Mode Manager Logic**
Activates the logic to switch between Mapping and Navigation modes.
```bash
ros2 run robot mode_manager_node
```

**4. Vision System**
Starts the camera driver for docking.
```bash
ros2 run ev_docking camera_publisher
```

### 3.2 Optional Visualization Tools

**Foxglove Studio (Advanced Visualization)**
Used for high-performance data visualization (point clouds, camera feeds).
```bash
# Ensure foxglove bridge is installed
# sudo apt install ros-humble-foxglove-bridge
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

**ROSBoard (Web Visualization)**
A lightweight, browser-based visualizer included in the workspace.
```bash
ros2 run rosboard rosboard_node
```
*Access at: `http://<robot-ip>:8888`*

---

## 4. Package Deep Dive

### 4.1 Package: `robot`
**Role:** The "Brain" of the system. Manages high-level state, mode switching, and map data.

#### **Nodes**

1.  **`mode_manager_node`**
    *   **Description:** Manages the lifecycle of heavy subprocesses. It ensures that Mapping (SLAM) and Navigation (Nav2/AMCL) never run simultaneously, preventing resource conflicts.
    *   **Key Logic:**
        *   Listens for mode switch requests.
        *   Gracefully terminates the current process (e.g., SLAM).
        *   Starts the new process (e.g., Nav2) with appropriate launch files.
        *   Pipes subprocess logs to the ROS 2 logger.

2.  **`state_manager`**
    *   **Description:** The central database and state machine. It aggregates data from various sources to provide a single "truth" for the Web UI.
    *   **Update Rate:** Publishes `/robot_state` at **10Hz**.
    *   **Key Logic:**
        *   Monitors TF tree for robot pose.
        *   Manages the `~/ros2_ws/maps` directory for saving/loading maps.
        *   Manages a JSON database of parking spots for each map.
        *   Acts as an intermediary for Nav2 Action Servers.

3.  **`encoders_to_joint_state`**
    *   **Description:** A helper node that converts raw encoder values into `JointState` messages.
    *   **Purpose:** Allows the `robot_state_publisher` to visualize wheel rotation in URDF/Rviz.

#### **Topics**

| Topic | Type | Description |
| :--- | :--- | :--- |
| `/robot_state` | `std_msgs/String` (JSON) | Unified state containing: mode, map name, Nav2 status, and robot pose (x, y, theta). |
| `/joint_states` | `sensor_msgs/JointState` | Wheel positions for visualization. |

#### **Services**

| Service | Type | Description |
| :--- | :--- | :--- |
| `/switch_mode` | `SwitchMode` | **Internal Use.** Low-level request to `mode_manager` to kill/start processes. |
| `/set_mode` | `SetMode` | **Public API.** High-level request to change mode. Handles map loading if switching to Nav. |
| `/list_maps` | `ListMaps` | Returns a list of all saved map files. |
| `/load_map` | `LoadMap` | Loads a specific map file into the state manager. |
| `/save_map` | `SaveMap` | Triggers SLAM Toolbox to save the current map to disk. |
| `/navigate_to_goal` | `NavigateToGoal` | Sends a target pose (x, y, theta) to the Nav2 stack. |
| `/save_parking_spot` | `SaveParkingSpot` | Saves the current robot pose as a named parking spot. |
| `/list_parking_spots` | `ListParkingSpots` | Returns all saved spots for the current map. |
| `/delete_parking_spot`| `DeleteParkingSpot` | Removes a specific parking spot. |

---

### 4.2 Package: `motor`
**Role:** The "Muscles" of the system. Handles low-level hardware abstraction and motion control.

#### **Nodes**

1.  **`mecanum_controller`**
    *   **Description:** The primary driver node. It translates high-level velocity commands into individual motor PWM signals.
    *   **Update Rate:** **50Hz** (20ms control loop).
    *   **Key Logic:**
        *   **Inverse Kinematics:** Calculates required speed for 4 wheels based on $V_x$, $V_y$, and $\omega_z$.
        *   **Acceleration Limiting:** Ramps motor speeds up/down to prevent wheel slip and current spikes.
        *   **Deadband:** Ignores inputs below a threshold to prevent drift.
        *   **I2C Communication:** Writes to the motor board at address `0x34` with retry logic.

2.  **`mecanum_odometry`**
    *   **Description:** Calculates the robot's position relative to its starting point (Odometry).
    *   **Update Rate:** **50Hz**.
    *   **Key Logic:**
        *   Reads encoder counters from the motor board.
        *   Computes forward kinematics to determine delta movement.
        *   Integrates deltas to update global pose ($x, y, \theta$).
        *   Publishes the `odom` -> `base_link` TF transform.

#### **Topics**

| Topic | Type | Description |
| :--- | :--- | :--- |
| `/cmd_vel` | `geometry_msgs/Twist` | Target velocity command (Linear X/Y, Angular Z). |
| `/encoder_values` | `std_msgs/Int32MultiArray` | Raw tick counts from all 4 encoders. |
| `/odom` | `nav_msgs/Odometry` | Estimated position and velocity based on wheel encoders. |
| `/emergency_stop_state`| `std_msgs/Bool` | Current status of the software emergency stop. |

#### **Services**

| Service | Type | Description |
| :--- | :--- | :--- |
| `/emergency_stop` | `EmergencyStop` | Toggles the emergency stop state. When active, `mecanum_controller` forces 0 velocity. |

---

### 4.3 Package: `ev_docking`
**Role:** The "Eyes" of the system. Handles computer vision and autonomous docking.

#### **Nodes**

1.  **`camera_publisher`**
    *   **Description:** A high-performance camera driver designed for the Raspberry Pi.
    *   **Update Rate:** **30Hz**.
    *   **Key Logic:**
        *   Uses a separate thread for frame capturing to prevent blocking.
        *   Configures the camera for MJPEG format to reduce USB bandwidth usage.
        *   Publishes both raw (for CV) and compressed (for Web UI) streams.

2.  **`docking_controller`**
    *   **Description:** The autonomous docking agent.
    *   **Key Logic:**
        *   Subscribes to `/camera/image_raw`.
        *   Uses `cv2.aruco` to detect AprilTags (36h11 family).
        *   **Visual Servoing:** Calculates the error between the robot and the tag.
        *   **P-Controller:**
            *   Linear Velocity $\propto$ Distance Error ($K_p = 1.8$)
            *   Angular Velocity $\propto$ Lateral Error ($K_p = 3.0$)
        *   Stops when `distance < target_dist` (0.05m).

3.  **`system_monitor`**
    *   **Description:** Monitors the health of the Raspberry Pi.
    *   **Update Rate:** **1Hz**.
    *   **Key Logic:**
        *   Checks CPU/RAM/Disk usage via `psutil`.
        *   Checks Voltage and Throttling flags via `vcgencmd` (Pi specific).
        *   Reads the last 20 lines of `kern.log` for system errors.

#### **Topics**

| Topic | Type | Description |
| :--- | :--- | :--- |
| `/camera/image_raw` | `sensor_msgs/Image` | Raw BGR8 image data for processing. |
| `/camera/image_raw/compressed` | `sensor_msgs/CompressedImage` | JPEG compressed stream for the Web UI. |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | Intrinsic calibration parameters. |
| `/docking/status` | `std_msgs/String` (JSON) | Docking state: `{detected: bool, distance: float, is_docking: bool}`. |
| `/docking/debug_image/compressed` | `sensor_msgs/CompressedImage` | Annotated image showing detected tags and axes. |
| `/system_stats` | `std_msgs/String` (JSON) | System health metrics. |

---

### 4.4 Package: `interfaces`
**Role:** Defines the custom service types used for inter-node communication.

#### **Service Definitions**

*   **`EmergencyStop.srv`**
    *   **Request:** `bool enable`
    *   **Response:** `bool success`, `bool is_stopped`, `string message`
*   **`SwitchMode.srv`**
    *   **Request:** `string mode`, `string map_file`
    *   **Response:** `bool success`, `string message`
*   **`SetMode.srv`**
    *   **Request:** `string mode`, `string map_name`
    *   **Response:** `bool success`, `string message`
*   **`ListMaps.srv`**
    *   **Request:** *(Empty)*
    *   **Response:** `string[] maps`
*   **`LoadMap.srv`**
    *   **Request:** `string map_name`
    *   **Response:** `bool success`, `string message`
*   **`SaveMap.srv`**
    *   **Request:** `string map_name`
    *   **Response:** `bool success`, `string file_path`, `string message`
*   **`NavigateToGoal.srv`**
    *   **Request:** `float64 x`, `float64 y`, `float64 theta`, `string label`, `string spot_id`
    *   **Response:** `bool success`, `string message`
*   **`SaveParkingSpot.srv`**
    *   **Request:** `string spot_id`, `string label`, `string map_name`, `float64 x`, `float64 y`, `float64 theta`
    *   **Response:** `bool success`, `string message`
*   **`ListParkingSpots.srv`**
    *   **Request:** `string map_name`
    *   **Response:** `string[] spot_ids`, `string[] labels`, `float64[] x`, `float64[] y`, `float64[] theta`
*   **`DeleteParkingSpot.srv`**
    *   **Request:** `string spot_id`
    *   **Response:** `bool success`

---

## 5. Web Dashboard Architecture

The dashboard provides a user-friendly interface for the robot.

**Location:** `~/web-ros-controller`  
**URL:** `http://<robot-ip>:80`

### Key Files
*   **`server.js`**: Express backend. Serves the frontend and provides the `config.js` endpoint for dynamic IP configuration.
*   **`public/app.js`**: The frontend logic core (1300+ lines).
    *   **ROS Connection:** Uses `roslibjs` to talk to `rosbridge`.
    *   **Map Rendering:** Custom HTML5 Canvas renderer for SLAM maps and robot position.
    *   **Joystick:** Virtual joystick (`nipplejs`) for touch control.
*   **`public/style.css`**: Cyber-aesthetic styling using CSS Grid and Flexbox.

### Features
*   **Mission Console (`index.html`):** Read-only monitoring (Battery, Camera Feed, System Logs).
*   **Setup & Control (`setup.html`):** Interactive control (Teleop, Map Management, Mode Switching).

---

## 6. Operational Workflows

### 6.1 Mapping a New Area
1.  Go to **Setup & Control** on the dashboard.
2.  Click **Mapping Mode**. The robot will launch SLAM Toolbox.
3.  Use the **Teleop Joystick** to drive the robot around the entire area.
4.  Ensure the map looks complete on the dashboard.
5.  Enter a name (e.g., "Garage_L1") and click the **Save Map** icon.

### 6.2 Autonomous Navigation
1.  Go to **Setup & Control**.
2.  Click **Nav Mode**.
3.  Select a saved map from the list.
4.  Wait for the **"Nav2 Ready"** badge (green).
5.  Select a saved parking spot or click on the map to send a goal.

### 6.3 Saving Parking Spots
1.  While in **Nav Mode**, drive the robot manually to a parking spot.
2.  Enter a label (e.g., "Spot 101").
3.  Click the **Save Spot** (Pin) icon.
4.  This spot is now available for one-click navigation.

### 6.4 Autonomous Docking
1.  Navigate the robot to a position where the charging port AprilTag is visible.
2.  The **Visual Docking** panel will show "Tag Detected: YES".
3.  The `docking_controller` automatically takes over control.
4.  The robot aligns and approaches until it reaches the docking distance.
5.  Status changes to **DOCKED**.

---

## 7. Troubleshooting

| Symptom | Probable Cause | Solution |
| :--- | :--- | :--- |
| **Robot won't move** | Emergency Stop Active | Check the red "STOP" button on the UI. |
| **"Nav2 Stopped"** | Wrong Mode | Switch to "Nav Mode" and select a map. |
| **Map is empty/black** | LiDAR not spinning | Check `ros2 topic echo /scan`. Verify LiDAR USB connection. |
| **Camera feed blank** | USB Bandwidth / Connection | Check connection. Restart `camera_publisher`. |
| **Web UI Disconnected** | ROSBridge down | Verify `mode_manager.launch.py` is running. Check port 9090. |
| **Drifting Odometry** | Calibration needed | Run `ros2 run motor measure_tpr` to recalibrate encoders. |

---

## 8. Configuration & Tuning

### Docking Parameters
Edit: `src/ev_docking/ev_docking/docking_controller.py`
```python
self.declare_parameter('target_dist', 0.05)  # Stop distance (meters)
self.declare_parameter('linear_kp', 1.8)     # Forward speed aggression
self.declare_parameter('angular_kp', 3.0)    # Turning speed aggression
```

### Navigation Parameters
Edit: `src/robot/config/nav2_params.yaml`
*   **`max_vel_x`**: Maximum forward speed.
*   **`inflation_radius`**: Safety distance from obstacles.

### Motor Calibration
Edit: `src/motor/motor/mecanum_odometry.py`
*   **`wheel_radius`**: Default 0.0485m.
*   **`base_length`**: Distance between wheels (Lx + Ly).
