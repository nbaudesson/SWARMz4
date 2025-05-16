# Use the official ROS image as the base image
FROM ubuntu:noble
ARG DEBIAN_FRONTEND=noninteractive
ENV TZ=Europe/Paris

# Set shell for running commands
SHELL ["/bin/bash", "-c"]

# install bootstrap tools
RUN apt-get update && apt-get upgrade -y && apt-get install --no-install-recommends -y \
    build-essential \
    git \
    kmod \
    lsb-release \
    ca-certificates \
    adduser \
    xterm \
    tmux \
    sudo \
    nano \
    xvfb \
    expect \
    pciutils \
    mesa-utils \
    wget \
    switcheroo-control


RUN apt-get remove modemmanager -y

#RUN adduser --disabled-password --shell /bin/bash --gecos '' swarmzuser
RUN useradd -m -s /bin/bash swarmzuser && echo 'swarmzuser:password' | chpasswd && adduser swarmzuser sudo
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
#RUN adduser swarmzuser sudo
RUN usermod -a -G dialout swarmzuser

USER swarmzuser

WORKDIR /home/swarmzuser

RUN mkdir SWARMz4

# First, only copy what we need to clean the build directories
COPY --chown=swarmzuser:swarmzuser ./install_scripts /home/swarmzuser/SWARMz4/install_scripts/
COPY --chown=swarmzuser:swarmzuser ./launch_scripts /home/swarmzuser/SWARMz4/launch_scripts/
COPY --chown=swarmzuser:swarmzuser ./Docker /home/swarmzuser/SWARMz4/Docker/

# Copy the ROS2 workspace but clean the build directories first
COPY --chown=swarmzuser:swarmzuser ./ros2_ws /home/swarmzuser/SWARMz4/ros2_ws/

RUN sudo rm -rf /home/swarmzuser/SWARMz4/ros2_ws/build /home/swarmzuser/SWARMz4/ros2_ws/install /home/swarmzuser/SWARMz4/ros2_ws/log

# Now copy the remaining files
COPY --chown=swarmzuser:swarmzuser . /home/swarmzuser/SWARMz4/

ENV HOME='/home/swarmzuser'
WORKDIR /home/swarmzuser/SWARMz4

RUN ./install_scripts/install_swarmz.sh
  
# RUN sudo chown -R swarmzuser:swarmzuser /home/swarmzuser

# USER swarmzuser
RUN export SWARMZ4_PATH="/home/swarmzuser/SWARMz4" && \
    echo "export SWARMZ4_PATH=\"$SWARMZ4_PATH\"" >> ~/.bashrc

# Clean up
RUN sudo apt-get clean
RUN sudo rm -rf /var/lib/apt/lists/*
RUN sudo rm -rf /tmp/*

#USER swarmzuser
## Finish QGroundControl install
WORKDIR /home/swarmzuser/SWARMz4/launch_scripts
RUN ./QGroundControl.AppImage --appimage-extract || true
RUN rm QGroundControl.AppImage || true
RUN ln -s /home/swarmzuser/SWARMz4/launch_scripts/squashfs-root/QGroundControl QGroundControl.AppImage || true
RUN chmod +x *.sh || true

WORKDIR /home/swarmzuser/
RUN mkdir -p .config/QGroundControl.org

RUN cp /home/swarmzuser/SWARMz4/Docker/QGroundControl.ini /home/swarmzuser/.config/QGroundControl.org/ || true

# Set the entrypoint to source ROS setup.bash and run a bash shell
WORKDIR /home/swarmzuser/SWARMz4/

CMD ["/bin/bash"]