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

To run the complete SMORE Bot system, you need to start three components:

1. **State Manager** - The core node that manages robot state:
    ```bash
    # Terminal 1
    ros2 run smore_bot_core state_manager
    ```

2. **Rosbridge Server** - Connects ROS to web applications:
    ```bash
    # Terminal 2
    ros2 launch rosbridge_server rosbridge_websocket_launch.xml
    ```

3. **Web Frontend** - Provides a web dashboard:
    ```bash
    # Terminal 3
    ros2 run smore_bot_web web_server
    ```

After starting all three components, open your web browser to http://localhost:8080 to view the dashboard.

### Troubleshooting

If you see a "Connecting to ROS..." message that doesn't change:
1. Check that rosbridge is running on port 9090
2. Open browser developer tools (F12) to look for JavaScript errors
3. Make sure all three terminals show no error messages

If you encounter shared memory errors in the terminal:
```bash
# Clean up any stale shared memory segments
rm -rf /dev/shm/fastrtps_*
```
