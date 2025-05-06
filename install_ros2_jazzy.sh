#!/bin/bash

# ROS 2 Jazzy Installation Script for Ubuntu
# ==========================================

set -e  # Exit on error

# Text formatting
BOLD="\e[1m"
GREEN="\e[32m"
YELLOW="\e[33m"
RED="\e[31m"
RESET="\e[0m"

echo -e "${BOLD}=== ROS 2 Jazzy Installation Script ===${RESET}\n"

# Function for section headers
section() {
    echo -e "\n${BOLD}${GREEN}=== $1 ===${RESET}"
}

# Function for commands with output
run_cmd() {
    echo -e "${YELLOW}Running: $1${RESET}"
    eval $1
    echo -e "${GREEN}✓ Done${RESET}"
}

# 1. Setup UTF-8 Locale
section "Setting up UTF-8 Locale"
echo "Checking current locale settings:"
if locale | grep -i utf-8; then
    echo -e "${GREEN}UTF-8 locale detected${RESET}"
else
    echo "Setting up UTF-8 locale..."
    run_cmd "sudo apt update"
    run_cmd "sudo apt install -y locales"
    run_cmd "sudo locale-gen en_US en_US.UTF-8"
    run_cmd "sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8"
    export LANG=en_US.UTF-8
    echo "New locale settings:"
    locale
fi

# 2. Setup Ubuntu Universe Repository
section "Setting up Ubuntu Universe Repository"
run_cmd "sudo apt install -y software-properties-common"
run_cmd "sudo add-apt-repository -y universe"

# 3. Add ROS 2 GPG key
section "Adding ROS 2 GPG key"
run_cmd "sudo apt update"
run_cmd "sudo apt install -y curl"
run_cmd "sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg"

# 4. Add ROS 2 apt repository
section "Adding ROS 2 apt repository"
run_cmd "echo \"deb [arch=\$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu \$(. /etc/os-release && echo \$UBUNTU_CODENAME) main\" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null"

# 5. Install Dev Tools
section "Installing ROS 2 Development Tools"
run_cmd "sudo apt update"
run_cmd "sudo apt install -y ros-dev-tools"

# 6. Install ROS 2
section "Installing ROS 2 Jazzy Desktop"
run_cmd "sudo apt update"
run_cmd "sudo apt upgrade -y"
run_cmd "sudo apt install -y ros-jazzy-desktop"

# 6.1. Install additional packages
section "Additional ROS packages"
run_cmd sudo apt install ros-jazzy-rqt*
run_cmd sudo apt install ros-jazzy-rosbridge-suite
run_cmd sudo apt install ros-jazzy-urdf
run_cmd sudo apt install ros-jazzy-xacro

# 7. Add Sourcing to .bashrc
section "Configuring .bashrc"
if grep -q "source /opt/ros/jazzy/setup.bash" ~/.bashrc; then
    echo "ROS 2 sourcing already in .bashrc"
else
    run_cmd "echo \"source /opt/ros/jazzy/setup.bash\" >> ~/.bashrc"
fi

if grep -q "export ROS_DOMAIN_ID=0" ~/.bashrc; then
    echo "ROS_DOMAIN_ID already in .bashrc"
else
    run_cmd "echo \"export ROS_DOMAIN_ID=0\" >> ~/.bashrc"
fi

if grep -q "source install/setup.bash" ~/.bashrc: then
    echo "Install/setup.bash hack already in .bashrc"
else
    echo "source install/setup.bash' >> ~/.bashrc"
fi

# Source the changes for this session
source ~/.bashrc

# 8. Check environment setup
section "Verifying ROS 2 Environment"
if printenv | grep -i ROS | grep -q "ROS_VERSION=2" && \
   printenv | grep -i ROS | grep -q "ROS_PYTHON_VERSION=3" && \
   printenv | grep -i ROS | grep -q "ROS_DISTRO=jazzy" && \
   printenv | grep -i ROS | grep -q "ROS_DOMAIN_ID=0"; then
    echo -e "${GREEN}All ROS environment variables are correctly set!${RESET}"
else
    echo -e "${RED}Some ROS environment variables are missing or incorrect.${RESET}"
    echo "Current ROS environment variables:"
    printenv | grep -i ROS
fi

# 9. Instructions for testing ROS 2
section "Testing ROS 2 Installation"
echo -e "${BOLD}To verify that ROS 2 is working correctly, run these commands in separate terminals:${RESET}"
echo -e "Terminal 1: ${YELLOW}ros2 run demo_nodes_cpp talker${RESET}"
echo -e "Terminal 2: ${YELLOW}ros2 run demo_nodes_py listener${RESET}"
echo -e "\nYou should see messages like \"I heard: [Hello World: xx]\" in the listener terminal."
echo -e "\n${GREEN}ROS 2 Jazzy installation is complete!${RESET}"
