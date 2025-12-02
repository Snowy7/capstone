#!/bin/bash

# Source ROS 2 environment
source /opt/ros/jazzy/setup.bash
source install/setup.bash

echo "Starting Robot System..."

# Function to handle cleanup on exit
cleanup() {
    echo ""
    echo "Shutting down all processes..."
    
    # Kill process groups (this kills the launch files and their children)
    # Using negative PID to kill entire process group
    for PID in $PID_MM_LAUNCH $PID_CAMERA $PID_DESC $PID_BRINGUP $PID_MODE_MGR; do
        if [ ! -z "$PID" ]; then
            # Try SIGINT first (graceful)
            kill -SIGINT -$PID 2>/dev/null || kill -SIGINT $PID 2>/dev/null
        fi
    done
    
    # Give processes 3 seconds to shut down gracefully
    sleep 3
    
    # Force kill any remaining processes
    for PID in $PID_MM_LAUNCH $PID_CAMERA $PID_DESC $PID_BRINGUP $PID_MODE_MGR; do
        if [ ! -z "$PID" ]; then
            kill -9 -$PID 2>/dev/null || kill -9 $PID 2>/dev/null
        fi
    done
    
    echo "Shutdown complete."
    exit 0
}

# Trap SIGINT (Ctrl+C) and SIGTERM
trap cleanup SIGINT SIGTERM

# Kill any existing processes on ports 80, 9090, 8080
echo "Cleaning up ports..."
for PORT in 80 9090 8080; do
    PID=$(sudo lsof -t -i:$PORT)
    if [ ! -z "$PID" ]; then
        echo "Killing process on port $PORT (PID: $PID)..."
        sudo kill -9 $PID
    fi
done

# 1. Run Camera
echo "Launching Camera..."
# Ensure camera device is accessible
if [ -e /dev/video0 ]; then
    # Add current user to video group if not already (requires re-login to take effect)
    sudo usermod -a -G video $USER 2>/dev/null
    # Set permissions on camera device
    sudo chmod 666 /dev/video0
    # Kill any process using the camera
    CAMERA_PIDS=$(sudo lsof -t /dev/video0 2>/dev/null)
    if [ ! -z "$CAMERA_PIDS" ]; then
        echo "Killing processes using camera: $CAMERA_PIDS"
        sudo kill -9 $CAMERA_PIDS
        sleep 1
    fi
else
    echo "Warning: /dev/video0 not found. Camera may not be connected."
fi
ros2 run ev_docking camera_publisher &
PID_CAMERA=$!
sleep 2


# 2. Run Mode Manager Node (alone)
echo "Launching Mode Manager Node..."
ros2 run robot mode_manager_node &
PID_MODE_MGR=$!
sleep 2

# 3. Run Bringup
echo "Launching Bringup..."
ros2 launch robot bringup_all.launch.py &
PID_BRINGUP=$!
sleep 5

# 4. Run Description
echo "Launching Robot Description..."
ros2 launch robot description.launch.py &
PID_DESC=$!
sleep 2

# 5. Run Mode Manager Launch
echo "Launching Mode Manager Launch..."
ros2 launch robot mode_manager.launch.py &
PID_MM_LAUNCH=$!

echo "All systems started."
echo "Press Ctrl+C to stop all processes."

# Wait indefinitely (the trap will handle exit)
wait
