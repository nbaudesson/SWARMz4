#! /bin/sh

# Check if SWARMZ4_PATH is already set in the environment
if [ -n "$SWARMZ4_PATH" ]; then
    echo "Using pre-existing SWARMZ4_PATH: $SWARMZ4_PATH"
else
    echo "SWARMZ4_PATH is not set. Searching for 'swarmz4' directory in $HOME..."

    # Locate the folder named 'swarmz4' starting from the current user's home directory
    SWARMZ4_PATH=$(find "$HOME" -maxdepth 4 -type d -name "swarmz4" 2>/dev/null)

    # Check if the folder was found
    if [ -z "$SWARMZ4_PATH" ]; then
        echo "Error: 'swarmz4' directory not found in $HOME!"
        exit 1
    else
        echo "Found 'swarmz4' directory at: $SWARMZ4_PATH"

        # Optionally export SWARMZ4_PATH so it persists for subsequent scripts
        export SWARMZ4_PATH
    fi
fi

# Set the GZ_SIM_RESOURCE_PATH environment variable
GZ_PATH="$SWARMZ4_PATH/PX4-Autopilot/Tools/simulation/gz"
export GZ_SIM_RESOURCE_PATH="$GZ_PATH"

# Append the export line to .bashrc only if it's not already there
if ! grep -q "export GZ_SIM_RESOURCE_PATH" "$HOME/.bashrc"; then
  echo "export GZ_SIM_RESOURCE_PATH=\"$GZ_PATH\"" >> "$HOME/.bashrc"
fi

### Kill old processes ###
pkill -f 'px4' || echo "No PX4 processes to kill."
pkill -f 'MicroXRCEAgent' || echo "No MicroXRCEAgent processes to kill."
pkill -f 'gz sim' || echo "No Gazebo simulations to kill."
pkill -f 'parameter_bridge'

### Start Micro-XRCE-DDS Agent ###
cd $SWARMZ4_PATH/Micro-XRCE-DDS-Agent || { echo "Micro-XRCE-DDS-Agent directory not found!"; exit 1; }
gnome-terminal --tab --title="MicroXRCEAgent" -- sh -c "MicroXRCEAgent udp4 -p 8888; bash"
echo "Started MicroXRCEAgent."

### PX4 Drone Configuration ###
cd $SWARMZ4_PATH/PX4-Autopilot || { echo "PX4-Autopilot directory not found!"; exit 1; }

# Parameters
# PX4_MODEL="x500_lidar_front" # Change to "x500" for classic x500 # this tries to connect to a x500, if none are found, it will try to spawn one
PX4_MODEL="gz_x500_lidar_front" # Change to "gz_x500" for classic x500
PX4_SYS_AUTOSTART=4017 # Use 4001 for classic x500

# Arguments
if [ -z "$1" ]; then
  NUM_DRONES=2
else
  NUM_DRONES=$1
fi
if [ -z "$2" ]; then
  HEADLESS=1
else
  HEADLESS=$2
fi

# Launch PX4 Instances
launch_px4_instance() {
  local instance_id=$1
  if [ "$HEADLESS" -ne 1 ]; then
    echo "HEADLESS mode is disabled. Launching Gazebo standalone."
    gnome-terminal --tab --title="px4_$instance_id" -- sh -c "
      TIMEOUT=10 \
      PX4_GZ_STANDALONE=1 \
      HEADLESS=$HEADLESS \
      PX4_SYS_AUTOSTART=$PX4_SYS_AUTOSTART \
      PX4_GZ_MODEL_POSE=\"0,$((instance_id - 1))\" \
      PX4_SIM_MODEL=$PX4_MODEL \
      ./build/px4_sitl_default/bin/px4 -i $instance_id; bash"
  else
    echo "HEADLESS mode is enabled. Skipping Gazebo standalone."
    gnome-terminal --tab --title="px4_$instance_id" -- sh -c "
      TIMEOUT=10 \
      HEADLESS=$HEADLESS \
      PX4_SYS_AUTOSTART=$PX4_SYS_AUTOSTART \
      PX4_GZ_MODEL_POSE=\"0,$((instance_id - 1))\" \
      PX4_SIM_MODEL=$PX4_MODEL \
      ./build/px4_sitl_default/bin/px4 -i $instance_id; bash"
  fi
}

### Launch PX4s ###
for i in $(seq 1 $NUM_DRONES); do
  echo "Launching PX4 instance $i..."
  launch_px4_instance $i
  sleep 3
done

### Launch Gazebo ###
# Conditionally Launch Gazebo
if [ "$HEADLESS" -ne 1 ]; then
  echo "HEADLESS mode is disabled. Launching Gazebo standalone."
  cd $SWARMZ4_PATH/PX4-Autopilot/Tools/simulation/gz || { echo "Gazebo tools directory not found!"; exit 1; }
  gnome-terminal --tab --title="gazebo" -- sh -c "python3 simulation-gazebo; bash"
else
  echo "HEADLESS mode is enabled. Skipping Gazebo standalone."
fi


### ROS 2 Setup ###
cd $SWARMZ4_PATH/ros2_ws || { echo "ROS 2 workspace directory not found!"; exit 1; }
source install/setup.bash

# ROS 2 Bridges and Launch Files
echo "Starting ROS 2 bridges and launch files..."
ros2 run ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock &
CLOCK_BRIDGE_PID=$!
ros2 launch px4_gz_bridge px4_laser_gz_bridge.launch.py nb_of_drones:=$NUM_DRONES &
LASER_BRIDGE_PID=$!

# Wait for user to exit
echo "Press Ctrl+C to terminate all processes."
trap "kill $CLOCK_BRIDGE_PID $LASER_BRIDGE_PID; pkill -f 'px4'; pkill -f 'MicroXRCEAgent'; pkill -f 'gz sim'; pkill -f 'parameter_bridge'; exit" INT

wait