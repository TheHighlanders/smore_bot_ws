ROS: [Jazzy Jalisco](https://docs.ros.org/en/jazzy/Releases/Release-Jazzy-Jalisco.html)
Linux: [Ubuntu 24.04](https://ubuntu.com/download/desktop)
GPIO: [LGPIO](https://pypi.org/project/lgpio/)

ROS Diagram:
![SMOREBOT2-ROSDIAGRAM](https://github.com/user-attachments/assets/b6c1f9b7-fe85-4e9a-a9c7-c604d9300402)

Automated setup for Linux Dev:
```bash
cd ~/smore_bot_ws

chmod +x install_ros2_jazzy.sh

./install_ros2_jazzy.sh
```

Manual setup for Linux Dev:
1. Install ROS ([deb packages](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html))
    A. Add UTF-8 Locale
    ```bash
    echo "Checking current locale settings:" && locale | grep -i utf-8 && echo "UTF-8 locale detected" || (echo "Setting up UTF-8 locale..." && sudo apt update && sudo apt install -y locales && sudo locale-gen en_US en_US.UTF-8 && sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && export LANG=en_US.UTF-8 && echo "New locale settings:" && locale)
    ```

    B. Ubuntu Universe
    ```bash
    sudo apt install software-properties-common
    
    sudo add-apt-repository universe
    ```

    C. Add ROS 2 GPG key
    ```bash
    sudo apt update && sudo apt install curl
    
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    ```

    D. Add ROS 2 apt repository
    ```bash
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    ```

    E. Install Dev Tools
    ```bash
    sudo apt update && sudo apt install ros-dev-tools
    ```

    F. Install ROS 2
    ```bash
    sudo apt upgrade
    
    sudo apt install ros-jazzy-desktop
    ```

2. Add Sourcing to .bashrc
    ```bash
    echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
    
    echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc
    
    source ~/.bashrc
    ```

3. Check setup:
    ```bash
    printenv | grep -i ROS | grep -q "ROS_VERSION=2" && printenv | grep -i ROS | grep -q "ROS_PYTHON_VERSION=3" && printenv | grep -i ROS | grep -q "ROS_DISTRO=jazzy" && printenv | grep -i ROS | grep -q "ROS_DOMAIN_ID=0" && echo "All ROS environment variables are correctly set!" || echo "Some ROS environment variables are missing or incorrect."
    ```

4. Check ROS works:
    ```bash
    ros2 run demo_nodes_cpp talker
    ```

    Then in another terminal:

    ```bash
    ros2 run demo_nodes_py listener
    ```
    
    Wait for the listener to print "I heard: [Hello World: xx]"

## Running the SMORE Bot System

### Option 1: Start Components Individually

To run the complete SMORE Bot system, you need to start three components:

1. **State Manager** - The core node that manages robot state:
    ```bash
    # Terminal 1
    ros2 run smore_bot_core state_manager
    ```

2. **Rosbridge Server** - Connects ROS to web applications:
    ```bash
    # Terminal 2
    pkill -f rosbridge_websocket
    ros2 launch rosbridge_server rosbridge_websocket_launch.xml
    ```

3. **Web Frontend** - Provides a web dashboard:
    ```bash
    # Terminal 3
    ros2 run smore_bot_web web_server
    ```

After starting all three components, open your web browser to http://localhost:8080 to view the dashboard.

### Option 2: Using the All-in-One Startup Script

A convenience script is provided to start all components in a single terminal:

1. **Make the script executable** (first time only):
    ```bash
    chmod +x ~/smore_bot_ws/start_smore_bot.sh
    ```

2. **Run the script**:
    ```bash
    cd ~/smore_bot_ws
    ./start_smore_bot.sh
    ```

The script will:
- Start all three components (rosbridge, state_manager, web_server)
- Show all output in a single terminal
- Allow you to stop everything with a single Ctrl+C
- Automatically clean up all processes when terminated

This is the recommended way to start the system for regular use.

### Troubleshooting

If you see a "Connecting to ROS..." message that doesn't change:
1. Check that rosbridge is running on port 9090
2. Open browser developer tools (F12) to look for JavaScript errors
3. Make sure all three terminals show no error messages

If you encounter shared memory errors in the terminal:
```bash
rm -rf /dev/shm/fastrtps_*
```

If you encounter "Unable to start server: [Errno 98] Address already in use Retrying in 5.0s.":
```bash
pkill -f rosbridge_websocket
sudo lsof -i :9090
# Look for the PID of the process using port 9090
sudo kill -s 4 <PID>
```

If you encounter throttling on Firefox: _EDIT: This is a active bug in the code and this will not fix it_
- Type about:config in the Firefox address bar
- Accept the warning
- Search for dom.timeout.throttling_delay
- Set its value to 0
- Optionally search for and set privacy.reduceTimerPrecision to false
