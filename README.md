# AutoCharge — Autonomous Wireless EV Charging Robot

AutoCharge is a ROS2-based project that implements an autonomous, mobile wireless EV charging robot. It integrates LiDAR SLAM (YDLIDAR), Nav2 for navigation (AMCL), mecanum drive hardware, a camera-based AprilTag docking system, and a web dashboard for monitoring and control via rosbridge.

This README provides: quick setup instructions for development and deployment, how to run the project, what the components are, and tips for testing and troubleshooting.

---

## Highlights

- ROS 2 Jazzy-based robotics stack (SLAM, Nav2, AMCL)
- Mecanum wheel base with encoder-based odometry
- YDLIDAR G4 for mapping and navigation
- AprilTag-camera docking for centimeter-level alignment
- Web-based dashboard (Node.js) that interacts with ROS via `rosbridge_server`
- ESP-NOW telemetry integration between EV-side ESP32 and robot
- Emergency stop system with a test script (`test_emergency_stop.py`)

---

## Repo Layout (key files and folders)

- `ros2_ws/`: Main ROS 2 workspace
  - `src/`: Packages (ev_docking, robot, motor, interfaces, ydlidar driver, etc.)
  - `launch_robot.sh`: Combined bringup script for rapid deployment
  - `maps/`: Saved maps and parking metadata
  - `install/`, `build/`, `log/`: Colcon build outputs (ignored in `.gitignore`)
- `web-ros-controller/`: Node.js/Express dashboard to connect to `rosbridge_server`
  - `server.js` hosts the dashboard and auto-injects JSON websocket config for ROS bridge
- `EMERGENCY_STOP_SYSTEM.md`, `EMERGENCY_STOP_QUICK_REF.md`: Emergency stop documentation
- `capstone_text.txt`, `PROJECT_DOCUMENTATION.md`: Project description and reports
- `test_emergency_stop.py`: Example integration test for emergency stop system

---

## Prerequisites (recommended)

- OS: Ubuntu 22.04 (recommended for ROS 2 compatibility) or equivalent Linux with ROS 2 Jazzy support
- ROS 2 Jazzy installed (follow the official ROS 2 Jazzy installation guide: [ROS 2 Jazzy install docs](https://docs.ros.org/en/jazzy/index.html))
- Colcon for building the ROS workspace
- Node.js and npm for the dashboard (LTS 18+ recommended)
- Python 3.10+ (for ROS 2 and various packages)
- Hardware: Raspberry Pi 5 (recommended), YDLIDAR G4, mecanum robot base, camera, ESP32 modules, wireless charging pad, power rails and wiring
 - `lsof` utility (used by `launch_robot.sh` to check ports and kill processes if necessary)

 - `lsof` utility (used by `launch_robot.sh` to check ports and kill processes if necessary)

---

## Setup — ROS 2 Workspace (Quick start)

Note: `ros2_ws` is the ROS workspace.


1. Install ROS 2 Jazzy and required system dependencies as per ROS 2 Jazzy official docs.

1. Install Node.js & NPM for the web dashboard:

```bash
# Install on Ubuntu
sudo apt update
curl -fsSL https://deb.nodesource.com/setup_18.x | sudo -E bash -
sudo apt-get install -y nodejs
```

1. Build the ROS 2 workspace:

```bash
cd ros2_ws
# Source system ROS 2 install (adjust path if different)
source /opt/ros/jazzy/setup.bash
# Build with colcon
colcon build --symlink-install
# Source local install overlay
source install/setup.bash
```

1. Optional: Ensure the camera and LiDAR are physically connected and accessible (e.g., `/dev/video0` and the LiDAR serial port or USB device).

---

## Running the system

### Bringup (combined script)

A helper script `launch_robot.sh` starts the camera node, the state manager, the robot bringup, URDF description, rosbridge and mode manager. Run it like below:

```bash
cd ros2_ws
# Make executable first time
chmod +x launch_robot.sh
# Source ROS 2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
# Start the robot stack
./launch_robot.sh
```

The script will print logs and keeps the processes in the foreground. Press Ctrl+C to gracefully stop everything (SIGINT triggers a clean shutdown).

### Web Dashboard (local network access)

The Node.js dashboard connects to `rosbridge_server` (defaults to WebSocket on port 9090). It serves the UI on port 80 by default.

```bash
cd ../web-ros-controller
npm install
# To run (on Linux with port 80, run with sudo if you must listen on 80)
sudo npm start
```

Open a browser and visit `http://<robot-ip>/` (server.js prints the network IP and *bridge* URL).

#### Notes

- To avoid `sudo` with port 80, change the `PORT` value in `server.js` (e.g., 8080) and restart with normal user privileges.
- If `lsof` is missing the launch script may fail; install it with `sudo apt install lsof`.
- If you run on a Pi or behind a firewall, ensure ports 80 and 9090 are open for your local network.

---

## Common developer workflows

### Save a map (when in mapping mode)

When mapping, you can save the current occupancy grid to `maps/` using the `state_manager`'s service:

```bash
# Save map using the public service
ros2 service call /save_map interfaces/srv/SaveMap "{ map_name: 'floor_a' }"
```

`state_manager` will call `map_saver_cli` which writes the image and YAML under the `maps/` folder.

### Switch modes (mapping <-> navigation)

The `state_manager` service `/set_mode` changes system mode:

```bash
# Switch to mapping mode
ros2 service call /set_mode interfaces/srv/SetMode "{ mode: 'mapping', map_name: '' }"

# Switch to navigation mode with a saved map
ros2 service call /set_mode interfaces/srv/SetMode "{ mode: 'navigation', map_name: 'floor_a' }"
```

### Save a parking spot

The dashboard or service can save a parking spot at the current pose. Example:

```bash
# Save current pose as a parking spot label
ros2 service call /save_parking_spot interfaces/srv/SaveParkingSpot "{ spot_id: 'B1-01', label: 'Spot-B1-01', map_name: 'floor_a', x: 1.0, y: 0.5, theta: 0.0 }"
```

### Navigate to a saved spot

Once in navigation mode:

```bash
ros2 service call /navigate_to_goal interfaces/srv/NavigateToGoal "{ x: 1.0, y: 0.5, theta: 0.0, label: 'Spot-B1-01', spot_id: 'B1-01' }"
```

### ROS Bridge (WebSocket)

The `mode_manager.launch.py` sets up `rosbridge_server` by launching `rosbridge_websocket_launch.xml`. If you're debugging, you can start it separately:

```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

Open your web browser UI and confirm `ws://<robot-ip>:9090` is reachable.

---

## Testing

### Emergency stop test

A quick test script `test_emergency_stop.py` is present in `ros2_ws/` that exercises the `/emergency_stop` service and prints the `/emergency_stop_state` topic contents.

To run it:

```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 test_emergency_stop.py
```

This script runs several tests (activate/deactivate E-Stop), and prints pass/fail messages to the terminal.

### Unit / integration tests

- ROS 2 packages include `ament`-based tests. Run `colcon test` to run configured tests. The repo uses `ament_*` lints and Python tests where present.

```bash
# In ros2_ws
colcon test --event-handlers console_cohesion+
colcon test-result --verbose
```

---

## Troubleshooting & Tips

- Camera not found: Ensure the camera is plugged in and appears at `/dev/video0`, and the user has permissions to read from it. `launch_robot.sh` tries to set permissions and add user to `video` group (you may need to re-login for group changes to take effect).
- Rosbridge disconnected: Verify `rosbridge_server` is running and reachable on port `9090` and that the UI is pointing to the right IP (server.js dynamically injects the ROS bridge IP in the frontend via `config.js`).
- Ports in use: `launch_robot.sh` tries to kill processes listening on ports `80`, `9090`, and `8080`. If you want to avoid that, change the port in the `web-ros-controller/server.js` file or remove the `sudo kill` lines in `launch_robot.sh`.
- LiDAR no readings: Confirm YDLIDAR is connected and its serial port matches the driver parameters. Check `ydlidar_ros2_driver` parameters and the LiDAR power supply.
- Nav2 fails to start: Ensure the map exists under `maps/` if switching to navigation and that `AMCL` has the correct parameters and localization topics.

---

## Contribution and Development

- Add your code in a new branch and issue a PR when ready.
- Follow existing package structure under `ros2_ws/src/` and keep package dependency lists updated in `package.xml` (use `ament_python` for Python-based packages and `ament_cmake` for native drivers).
- Keep new third-party dependencies documented in `PROJECT_DOCUMENTATION.md`.

---

## Acknowledgements & References

- ROS2 Jazzy, Nav2, SLAM Toolbox, YDLIDAR driver
- Node.js/Express for the dashboard
- Example code and configuration adapted from various ROS 2 and YDLIDAR samples

---

## Next steps / Optional Enhancements

- Add Docker or a systemd startup unit for easier deployment on headless Pis
- Provide a `setup.sh` script that automates installing Node, Python deps, and colcon build dependencies
- Add a dev README in `ros2_ws/` documenting package-by-package responsibilities and ROS topics/services used

---

If you want, I can now create a separate `docs/` with deeper per-package guides (how each package works, how to add new maps, how to calibrate the camera). Would you like me to add that?
