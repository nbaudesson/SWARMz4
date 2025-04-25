#!/bin/bash

# Source check_swarmz_path.sh to ensure SWARMZ4_PATH is set
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/check_swarmz_path.sh"

# Function to detect Ubuntu version
detect_ubuntu_version() {
    lsb_release -rs | cut -d. -f1
}

# Function to check if the appropriate ROS 2 is already installed
check_ros2_installed() {
    local ros_version=$1
    if [ -d "/opt/ros/$ros_version" ]; then
        return 0
    else
        return 1
    fi
}

# Function to install ROS 2 Humble (for Ubuntu 22)
install_ros2_humble() {
    echo "Installing ROS 2 Humble..."

    sudo apt update && sudo apt install -y locales
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8

    sudo apt install -y software-properties-common
    sudo add-apt-repository universe
    sudo apt update && sudo apt install -y curl gnupg2 lsb-release

    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

    sudo apt update && sudo apt upgrade -y
    sudo apt install -y ros-humble-desktop ros-dev-tools

    source /opt/ros/humble/setup.bash
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

    sudo apt install ros-humble-tf-transformations -y
    sudo apt install ros-humble-gazebo-ros -y
    sudo apt install ros-humble-ros-gz-bridge -y
    sudo apt install ros-humble-ros-gzharmonic -y

    sudo apt install libyaml-cpp-dev -y # Required for ROS 2 to parse yaml files in cpp
    sudo apt install python3-colcon-common-extensions -y
    sudo apt install python3-rosdep -y

    echo "ROS 2 Humble installation completed."
}

# Function to install ROS 2 Jazzy (for Ubuntu 24)
install_ros2_jazzy() {
    echo "Installing ROS 2 Jazzy..."

    sudo apt update && sudo apt install -y locales
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8

    sudo apt install -y software-properties-common
    sudo add-apt-repository universe
    sudo apt update && sudo apt install -y curl gnupg2 lsb-release

    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

    sudo apt update && sudo apt upgrade -y
    sudo apt install -y ros-jazzy-desktop ros-dev-tools

    source /opt/ros/jazzy/setup.bash
    echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

    sudo apt install ros-jazzy-tf-transformations -y
    sudo apt install ros-jazzy-gazebo-ros -y
    sudo apt install ros-jazzy-ros-gz-bridge -y
    sudo apt install ros-jazzy-ros-gzharmonic -y

    sudo apt install libyaml-cpp-dev -y # Required for ROS 2 to parse yaml files in cpp
    sudo apt install python3-colcon-common-extensions -y
    sudo apt install python3-rosdep -y

    echo "ROS 2 Jazzy installation completed."
}

# Build workspace
build_workspace() {
    local ros_version=$1
    source /opt/ros/$ros_version/setup.bash
    
    cd $SWARMZ4_PATH/ros2_ws || { echo "Failed to access ros2_ws directory"; exit 1; }
    
    # Only run rosdep init if it hasn't been initialized yet
    if ! rosdep version &>/dev/null; then
        sudo rosdep init
    fi
    
    sudo apt update
    rosdep update
    rosdep install --from-paths src --ignore-src -r -y
    colcon build --symlink-install
}

# Main function for ROS 2
ubuntu_version=$(detect_ubuntu_version)

if [ "$ubuntu_version" = "24" ]; then
    ROS_DISTRO="jazzy"
    if ! check_ros2_installed "$ROS_DISTRO"; then
        install_ros2_jazzy
        build_workspace "$ROS_DISTRO"
    else
        echo "ROS 2 Jazzy is already installed, skipping."
    fi
elif [ "$ubuntu_version" = "22" ]; then
    ROS_DISTRO="humble"
    if ! check_ros2_installed "$ROS_DISTRO"; then
        install_ros2_humble
        build_workspace "$ROS_DISTRO"
    else
        echo "ROS 2 Humble is already installed, skipping."
    fi
else
    echo "Unsupported Ubuntu version $ubuntu_version. This script supports Ubuntu 22 for ROS 2 Humble and Ubuntu 24 for ROS 2 Jazzy."
    exit 1
fi

# Make sure ROS setup is sourced even if already installed
if [ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
    source /opt/ros/$ROS_DISTRO/setup.bash
fi

echo "ROS 2 installation and setup complete."
