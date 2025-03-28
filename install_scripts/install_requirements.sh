sudo apt update
# build tools
sudo apt-get install build-essential -y
sudo apt-get install cmake -y
sudo apt-get install git -y
# QGC
sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y
sudo apt install libfuse2 -y
sudo apt install libxcb-xinerama0 libxkbcommon-x11-0 libxcb-cursor-dev -y
# ROS2
sudo apt install libyaml-cpp-dev -y # Required for ROS 2 to parse yaml files in cpp
sudo apt install python3-colcon-common-extensions -y
sudo apt install python3-rosdep -y
# terminal
sudo apt install tmux xterm switcheroo-control -y
# Gazebo messages
sudo apt install libgz-transport13 -y
# Pip
sudo apt install python3-pip -y
sudo ln -s /usr/bin/pip3 /usr/bin/pip
# pip for ros2
pip install --user empy==3.3.4 pyros-genmsg setuptools==58.2 psutil numpy==1.26.4 transforms3d
