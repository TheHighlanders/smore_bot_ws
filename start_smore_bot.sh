#!/bin/bash

source /opt/ros/jazzy/setup.bash
source ~/smore_bot_ws/install/setup.bash

cleanup() {
    echo "Shutting down all processes..."
    kill $ROSBRIDGE_PID $STATE_MANAGER_PID $WEB_SERVER_PID 2>/dev/null
    exit 0
}

# Set trap to catch SIGINT (Ctrl+C) and other termination signals
trap cleanup SIGINT SIGTERM

echo "Starting rosbridge server..."
ros2 launch rosbridge_server rosbridge_websocket_launch.xml &
ROSBRIDGE_PID=$!
sleep 2  # Give rosbridge time to start up

echo "Starting state manager..."
ros2 run smore_bot_core state_manager &
STATE_MANAGER_PID=$!
sleep 1

echo "Starting web server..."
ros2 run smore_bot_web web_server &
WEB_SERVER_PID=$!

echo "All components started!"
echo "Access the dashboard at: http://localhost:8080"
echo "Press Ctrl+C to shutdown all components"

# Wait for any process to exit
wait -n

# If any process exits, clean up the others
cleanup
