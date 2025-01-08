#!/bin/bash

# Function to check if ROS 2 Humble is already installed
check_ros2_installed() {
    if [ -d "/opt/ros/humble" ]; then
        echo "ROS 2 Humble is already installed."
        return 0
    else
        echo "ROS 2 Humble is not installed."
        return 1
    fi
}

# Function to install ROS 2 Humble
install_ros2() {
    echo "Installing ROS 2 Humble..."

    sudo apt update && sudo apt install -y locales
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8

    sudo apt install -y software-properties-common
    sudo add-apt-repository universe
    sudo apt update && sudo apt install -y curl

    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

    sudo apt update && sudo apt upgrade -y
    sudo apt install -y ros-humble-desktop ros-dev-tools
    source /opt/ros/humble/setup.bash
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

    # Additional Python dependencies
    pip install --user -U empy==3.3.4 pyros-genmsg setuptools==58.2

    echo "ROS 2 Humble installation completed."
}

# Main function for ROS 2
if ! check_ros2_installed; then
    install_ros2
else
    echo "ROS 2 Humble is already installed, skipping."
fi
