# Use the official ROS image as the base image
FROM osrf/ros:humble-desktop-jammy

# Set shell for running commands
SHELL ["/bin/bash", "-c"]

# ENV HOME='/home/swarmzuser'

# install bootstrap tools
RUN apt-get update && apt-get upgrade -y && apt-get install --no-install-recommends -y \
    build-essential \
    git \
    kmod \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-vcstool \
    python3-pip \
    gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl \
    libxcb-xinerama0 libxkbcommon-x11-0 libxcb-cursor-dev \
    libxcb-cursor0 \
    libfuse2 \
    xterm \
    tmux \
    sudo \
    nano \
    libyaml-cpp-dev \
    xvfb \
    expect \
    pciutils \
    mesa-utils \
    wget \
    switcheroo-control


RUN apt-get remove modemmanager -y

RUN pip install --user -U empy==3.3.4 pyros-genmsg setuptools==58.2 psutil

RUN adduser --disabled-password --shell /bin/bash --gecos '' swarmzuser
RUN adduser swarmzuser sudo
RUN usermod -a -G dialout swarmzuser

RUN rosdep fix-permissions

USER swarmzuser

WORKDIR /home/swarmzuser
RUN git clone --recursive https://github.com/nbaudesson/SWARMz4.git

USER root
ENV HOME='/home/swarmzuser'
WORKDIR /home/swarmzuser/SWARMz4
RUN ./install_scripts/install_swarmz.sh

    
RUN chown -R swarmzuser:swarmzuser /home/swarmzuser

USER swarmzuser
RUN export SWAMRZ4_PATH="/home/swarmzuser/SWARMz4" && \
    echo "export SWARMZ4_PATH=\"$SWARMZ4_PATH\"" >> ~/.bashrc

RUN cd ros2_ws && \
    source /opt/ros/humble/setup.bash && \
#    /home/swarmzuser/SWARMz4/PX4-Autopilot/Tools/copy_to_ros_ws.sh . && \
    colcon build --symlink-install

# Clean up
USER root

RUN apt-get clean
RUN rm -rf /var/lib/apt/lists/*
RUN rm -rf /tmp/*

USER swarmzuser
## Finish QGroundControl install
WORKDIR /home/swarmzuser/SWARMz4/launch_scripts
RUN ./QGroundControl.AppImage --appimage-extract
RUN rm QGroundControl.AppImage
RUN ln -s /home/swarmzuser/SWARMz4/launch_scripts/squashfs-root/QGroundControl QGroundControl.AppImage
RUN chmod +x *.sh

WORKDIR /home/swarmzuser/
RUN mkdir -p .config/QGroundControl.org

COPY QGroundControl.ini /home/swarmzuser/.config/QGroundControl.org/

# Set the entrypoint to source ROS setup.bash and run a bash shell

WORKDIR /home/swarmzuser/SWARMz4/

CMD ["/bin/bash"]