#!/bin/bash

# Source ROS2 and workspace
source /opt/ros/jazzy/setup.bash
source ~/smore_bot_ws/install/setup.bash

# Store PIDs of launched processes
PIDS=()

kill_process() {
    local PID=$1
    if ps -p $PID > /dev/null; then
        echo "Killing process $PID"
        kill -TERM $PID 2>/dev/null
        sleep 0.2
        # If still running, force kill
        if ps -p $PID > /dev/null; then
            kill -KILL $PID 2>/dev/null
        fi
    fi
}

kill_by_port() {
    local PORT=$1
    local PORT_PIDS=$(lsof -t -i:$PORT 2>/dev/null)
    
    if [ ! -z "$PORT_PIDS" ]; then
        echo "Killing processes on port $PORT: $PORT_PIDS"
        for PID in $PORT_PIDS; do
            kill -TERM $PID 2>/dev/null
            sleep 0.2
            if ps -p $PID > /dev/null; then
                kill -KILL $PID 2>/dev/null
            fi
        done
    fi
}

cleanup() {
    echo "Shutting down all processes..."
    
    for PID in "${PIDS[@]}"; do
        kill_process $PID
    done
    
    kill_by_port 9090
    kill_by_port 8080
    
    pkill -f rosbridge_websocket 2>/dev/null
    pkill -f smore_bot_core 2>/dev/null
    pkill -f web_server 2>/dev/null
    
    echo "Cleanup complete"
    exit 0
}

# Set trap to catch termination signals
trap cleanup SIGINT SIGTERM EXIT

# Start rosbridge
echo "Starting rosbridge server..."
ros2 launch rosbridge_server rosbridge_websocket_launch.xml > /dev/null 2>&1 &
ROSBRIDGE_PID=$!
PIDS+=($ROSBRIDGE_PID)
echo "Rosbridge started with PID: $ROSBRIDGE_PID"
sleep 2

# Start state manager
echo "Starting state manager..."
ros2 run smore_bot_core state_manager > /dev/null 2>&1 &
STATE_MANAGER_PID=$!
PIDS+=($STATE_MANAGER_PID)
echo "State manager started with PID: $STATE_MANAGER_PID"
sleep 1

# Start web server
echo "Starting web server..."
ros2 run smore_bot_web web_server > /dev/null 2>&1 &
WEB_SERVER_PID=$!
PIDS+=($WEB_SERVER_PID)
echo "Web server started with PID: $WEB_SERVER_PID"

echo "All components started!"
echo "Access the dashboard at: http://localhost:8080"
echo "Press Ctrl+C to shutdown all components"

# Keep script running until Ctrl+C
wait -n || true

# Handle cleanup
cleanup
