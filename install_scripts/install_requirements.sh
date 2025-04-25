sudo apt update
# build tools
sudo apt-get install build-essential -y
sudo apt-get install cmake -y
sudo apt-get install git -y
# QGC
sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y
sudo apt install libfuse2 -y
sudo apt install libxcb-xinerama0 libxkbcommon-x11-0 libxcb-cursor-dev -y
sudo apt-get install xvfb -y
# ROS2
check_ros2_repo_configured() {
    if [ -f "/etc/apt/sources.list.d/ros2-latest.list" ]; then
    sudo apt install libyaml-cpp-dev -y # Required for ROS 2 to parse yaml files in cpp
    sudo apt install python3-colcon-common-extensions -y
    sudo apt install python3-rosdep -y
    sudo apt install ros-humble-tf-transformations -y
    sudo apt install ros-humble-gazebo-ros -y
    sudo apt install ros-humble-ros-gz-bridge -y
    sudo apt install ros-humble-ros-gzharmonic -y
    else
        echo "Skipping ROS 2 package installation. Will be handled in complete ROS 2 install."
    fi
}
# terminal
sudo apt install tmux xterm expect switcheroo-control -y
# Gazebo messages
sudo apt install libgz-transport13 -y
# Pip
sudo apt install python3-pip -y
sudo ln -s /usr/bin/pip3 /usr/bin/pip
# pip for ros2
pip install --break-system-packages empy==3.3.4 pyros-genmsg setuptools==58.2 psutil numpy==1.26.4 transforms3d
