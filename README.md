ROS: [Jazzy Jalisco](https://docs.ros.org/en/jazzy/Releases/Release-Jazzy-Jalisco.html)
Linux: [Ubuntu 24.04](https://ubuntu.com/download/desktop)
GPIO: [LGPIO](https://pypi.org/project/lgpio/)

ROS Diagram:

![SMOREBOT2-ROSDIAGRAM](https://github.com/user-attachments/assets/b6c1f9b7-fe85-4e9a-a9c7-c604d9300402)

  

Automated setup for Linux Dev:
`cd ~/smore_bot_ws`
`chmod +x install_ros2_jazzy.sh`
`./install_ros2_jazzy.sh`

Manual setup for Linux Dev:
1. Install ROS ([deb packages](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html))
    A. Add UTF-8 Locale
        ```echo "Checking current locale settings:" && locale | grep -i utf-8 && echo "UTF-8 locale detected" || (echo "Setting up UTF-8 locale..." && sudo apt update && sudo apt install -y locales && sudo locale-gen en_US en_US.UTF-8 && sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && export LANG=en_US.UTF-8 && echo "New locale settings:" && locale)```
        
    B. Ubuntu Universe
        ```sudo apt install software-properties-common
        sudo add-apt-repository universe```
        
    C. Add ROS 2 GPG key
        ```sudo apt update && sudo apt install curl
        sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg```

    D. Add ROS 2 apt repository
        ```echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null```
        
    E. Install Dev Tools
    ```sudo apt update && sudo apt install ros-dev-tools```
    
    F. Install ROS 2
	```sudo apt upgrade
	 sudo apt install ros-jazzy-desktop```

2. Add Sourcing to .bashrc
    ```echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
    echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc
    echo "source ~/smore_bot_ws/install/setup.bash" >> ~/.bashrc
    source ~/.bashrc```

3. Check setup:
    ```printenv | grep -i ROS | grep -q "ROS_VERSION=2" && printenv | grep -i ROS | grep -q "ROS_PYTHON_VERSION=3" && printenv | grep -i ROS | grep -q "ROS_DISTRO=jazzy" && printenv | grep -i ROS | grep -q "ROS_DOMAIN_ID=0" && echo "All ROS environment variables are correctly set!" || echo "Some ROS environment variables are missing or incorrect."```

4. Check ROS works:
    ```ros2 run demo_nodes_cpp talker```
    Then in another terminal:
    ```ros2 run demo_nodes_py listener```
    Wait for the listener to print "I heard: [Hello World: xx]"