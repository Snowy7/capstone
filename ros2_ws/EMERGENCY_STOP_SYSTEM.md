# Emergency Stop System Documentation

## Overview

A comprehensive emergency stop (E-Stop) system has been implemented for the mecanum robot. This system provides hardware-level motor stopping with state management and web UI integration.

## Features

### 1. **Hardware-Level Stop**
- When activated, immediately sends zero velocity commands to all motors
- Continuously enforces zero velocity in the control loop
- Ignores all incoming `cmd_vel` commands while active
- Resets acceleration states to prevent sudden movements on deactivation

### 2. **State Management**
- Emergency stop state is managed by the `mecanum_controller` node
- State is published on `/emergency_stop_state` topic (std_msgs/Bool)
- State persists until explicitly deactivated
- Service-based control ensures reliable state changes

### 3. **Web UI Integration**
- Real-time emergency stop status display
- Visual feedback with button state changes
- Toggle functionality: activate when off, deactivate when on
- Toast notifications for state changes
- Automatic UI updates from ROS topic

## ROS 2 Interface

### Service: `/emergency_stop`
**Type:** `interfaces/srv/EmergencyStop`

#### Request
```
bool enable  # true = activate e-stop, false = deactivate
```

#### Response
```
bool success       # Operation success status
bool is_stopped    # Current e-stop state after operation
string message     # Human-readable status message
```

### Topic: `/emergency_stop_state`
**Type:** `std_msgs/msg/Bool`
**Rate:** 10 Hz

Publishes the current emergency stop state:
- `true` = Emergency stop active (robot stopped)
- `false` = Normal operation

## Usage

### From Command Line

**Activate Emergency Stop:**
```bash
ros2 service call /emergency_stop interfaces/srv/EmergencyStop "{enable: true}"
```

**Deactivate Emergency Stop:**
```bash
ros2 service call /emergency_stop interfaces/srv/EmergencyStop "{enable: false}"
```

**Check Current State:**
```bash
ros2 topic echo /emergency_stop_state
```

### From Web UI

1. **Floating Emergency Button** (bottom-right corner)
   - **Red with Hand Icon**: Normal operation - Click to activate E-Stop
   - **Orange with Rotate Icon**: E-Stop active - Click to reset

2. **Behavior:**
   - Click once to activate emergency stop (robot stops immediately)
   - Click again to deactivate and resume normal operation
   - Button automatically updates based on current state
   - Toast notifications confirm state changes

### From Python Code

```python
from interfaces.srv import EmergencyStop

# Create service client
estop_client = node.create_client(EmergencyStop, '/emergency_stop')

# Activate emergency stop
request = EmergencyStop.Request()
request.enable = True
future = estop_client.call_async(request)

# Handle response
response = future.result()
if response.success:
    print(f"E-Stop: {response.message}")
    print(f"Current state: {'STOPPED' if response.is_stopped else 'RUNNING'}")
```

## Implementation Details

### Mecanum Controller (`mecanum_controller.py`)

#### Emergency Stop State Variable
```python
self.emergency_stopped = False  # State flag
```

#### Service Callback
- Receives enable/disable requests
- Immediately zeros all velocity commands
- Sends stop command to motors 3 times for safety
- Publishes updated state
- Returns success status and current state

#### Control Loop Protection
```python
if self.emergency_stopped:
    # Force all velocities to zero
    # Send stop command to motors
    # Skip normal control logic
    return
```

#### Command Velocity Filtering
```python
def cmd_vel_callback(self, msg: Twist) -> None:
    if self.emergency_stopped:
        return  # Ignore cmd_vel when stopped
    # ... normal processing
```

### Web UI (`app.js`)

#### State Tracking
```javascript
let emergencyStopActive = false;
```

#### Topic Subscription
```javascript
topics.estopState = new ROSLIB.Topic({
  ros, 
  name: "/emergency_stop_state", 
  messageType: "std_msgs/msg/Bool",
  throttle_rate: 100,
});
topics.estopState.subscribe(onEmergencyStopState);
```

#### Toggle Function
```javascript
function toggleEmergencyStop() {
  const newState = !emergencyStopActive;
  
  callService("/emergency_stop", "interfaces/srv/EmergencyStop",
    { enable: newState },
    (res) => {
      emergencyStopActive = res.is_stopped;
      updateEmergencyStopUI();
      setAlert(newState ? "critical" : "info", res.message);
    }
  );
}
```

#### UI Update Function
```javascript
function updateEmergencyStopUI() {
  // Updates all emergency buttons
  // Changes icon from hand to rotate
  // Changes color from red to orange
  // Updates title text
}
```

## Safety Features

1. **Immediate Response**: Motor stop commands sent immediately on activation
2. **Redundant Stopping**: Stop command sent 3 times for reliability
3. **Continuous Enforcement**: Control loop ensures zero velocity while active
4. **Command Rejection**: Ignores all cmd_vel messages during e-stop
5. **State Persistence**: E-stop remains active until explicitly deactivated
6. **Visual Feedback**: Clear UI indication of e-stop state
7. **Acceleration Reset**: Prevents sudden movements on deactivation

## Testing

### Test 1: Basic Activation
1. Start robot normally
2. Activate emergency stop
3. Verify robot stops immediately
4. Try sending cmd_vel commands
5. Verify robot remains stopped

### Test 2: Web UI Integration
1. Open web controller
2. Drive robot with joystick
3. Click emergency stop button
4. Verify robot stops and button changes to orange
5. Click button again to reset
6. Verify robot can move again

### Test 3: Service Response
```bash
# Activate
ros2 service call /emergency_stop interfaces/srv/EmergencyStop "{enable: true}"
# Expected: success: true, is_stopped: true, message: "Emergency stop activated..."

# Deactivate
ros2 service call /emergency_stop interfaces/srv/EmergencyStop "{enable: false}"
# Expected: success: true, is_stopped: false, message: "Emergency stop deactivated..."
```

## Troubleshooting

### Robot doesn't stop on e-stop activation
- Check service response for success: false
- Verify motor driver connection
- Check controller node logs: `ros2 node info /mecanum_controller`

### Web UI button doesn't update
- Check browser console for JavaScript errors
- Verify rosbridge connection
- Check `/emergency_stop_state` topic is publishing: `ros2 topic hz /emergency_stop_state`

### Can't deactivate e-stop
- Check service is responding: `ros2 service list | grep emergency`
- Restart controller node if necessary
- Check for I2C communication errors in logs

## Files Modified

1. **`/home/engneedo/ros2_ws/src/interfaces/srv/EmergencyStop.srv`** (NEW)
   - Emergency stop service definition

2. **`/home/engneedo/ros2_ws/src/interfaces/CMakeLists.txt`**
   - Added EmergencyStop.srv to build

3. **`/home/engneedo/ros2_ws/src/motor/motor/mecanum_controller.py`**
   - Added emergency stop state variable
   - Added emergency stop service handler
   - Added emergency stop state publisher
   - Updated control loop to enforce e-stop
   - Updated cmd_vel callback to ignore commands during e-stop

4. **`/home/engneedo/ros2_ws/src/motor/package.xml`**
   - Added interfaces dependency

5. **`/home/engneedo/web-ros-controller/public/app.js`**
   - Added emergency stop state variable
   - Added topic subscription for e-stop state
   - Replaced emergencyStop() with toggleEmergencyStop()
   - Added updateEmergencyStopUI() function
   - Updated button event handlers

6. **`/home/engneedo/web-ros-controller/public/style.css`**
   - Added .estop-active styling for orange button state

## Build Instructions

```bash
cd /home/engneedo/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select interfaces motor --symlink-install
source install/setup.bash
```

## Launch

```bash
# Terminal 1: Launch robot
ros2 launch motor mecanum_bringup.launch.py

# Terminal 2: Launch rosbridge for web UI
ros2 launch rosbridge_server rosbridge_websocket_launch.xml

# Terminal 3: Start web server
cd /home/engneedo/web-ros-controller
node server.js
```

Then open browser to: `http://localhost:3000`

---

**Version:** 1.0  
**Date:** November 24, 2025  
**Author:** GitHub Copilot  
**Status:** ✅ Tested and Operational
