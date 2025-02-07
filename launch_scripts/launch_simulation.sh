#! /bin/sh

# Check if SWARMZ4_PATH is already set in the environment
if [ -n "$SWARMZ4_PATH" ]; then
    echo "Using pre-existing SWARMZ4_PATH: $SWARMZ4_PATH"
else
    echo "SWARMZ4_PATH is not set. Searching for 'SWARMz4' directory in $HOME..."

    # Locate the folder named 'swarmz4' starting from the current user's home directory
    SWARMZ4_PATH=$(find "$HOME" -maxdepth 4 -type d -name "SWARMz4" 2>/dev/null)

    # Check if the folder was found
    if [ -z "$SWARMZ4_PATH" ]; then
        echo "Error: 'SWARMz4' directory not found in $HOME!"
        sleep 5
        exit 1
    else
        echo "Found 'SWARMz4' directory at: $SWARMZ4_PATH"

        # Optionally export SWARMZ4_PATH so it persists for subsequent scripts
        export SWARMZ4_PATH
    fi
fi

kill_processes() {
    echo "Killing old processes..."
    pkill -f 'px4' || echo "No PX4 processes to kill."
    pkill -f 'MicroXRCEAgent' || echo "No MicroXRCEAgent processes to kill."
    pkill -f 'gz sim' || echo "No Gazebo simulations to kill."
    pkill -f 'parameter_bridge' || echo "No parameter_bridge processes to kill."
    pkill -f 'QGroundControl' || echo "No QGroundControl processes to kill."
    # Second time's a charm
    pkill -f 'QGroundControl' || echo "No QGroundControl processes to kill."
}

# Call kill_processes function
kill_processes

### Start Micro-XRCE-DDS Agent ###
cd $SWARMZ4_PATH/Micro-XRCE-DDS-Agent || { echo "Micro-XRCE-DDS-Agent directory not found!"; exit 1; }
gnome-terminal --tab --title="MicroXRCEAgent" -- sh -c "MicroXRCEAgent udp4 -p 8888; bash"
echo "Started MicroXRCEAgent."

### PX4 Drone Configuration ###
cd $SWARMZ4_PATH/PX4-Autopilot || { echo "PX4-Autopilot directory not found!"; exit 1; }

# Parameters
PX4_MODEL="gz_x500_lidar_front" # Change to "gz_x500" for classic x500
PX4_SYS_AUTOSTART=4017 # Use 4001 for classic x500
FIELD_LENGTH=5
FIELD_WIDTH=2
# FIELD_LENGTH=500
# FIELD_WIDTH=250
NUM_DRONES_PER_TEAM=2
TOTAL_DRONES=$((NUM_DRONES_PER_TEAM * 2))
HEADLESS=1 # headless
# HEADLESS=0 # GUI
# WORLD="default"
WORLD="swarmz_world"

# Arguments
if [ -n "$1" ]; then
  HEADLESS=$1
fi
if [ -n "$2" ]; then
  NUM_DRONES_PER_TEAM=$2
  TOTAL_DRONES=$((NUM_DRONES_PER_TEAM * 2))
fi
if [ -n "$3" ]; then
  FIELD_LENGTH=$3
fi
if [ -n "$4" ]; then
  FIELD_WIDTH=$4
fi
if [ -n "$5" ]; then
  WORLD=$5
fi

# Launch PX4 Instances with improved pose handling
launch_px4_instance() {
    local instance_id=$1
    local x_pos=$2
    local y_pos=$3
    local yaw=${4:-0}  # Default yaw to 0 if not specified
    local pose="$x_pos,$y_pos,0,0,0,$yaw"
    
    echo "Launching PX4 instance $instance_id at position ($x_pos, $y_pos, 0) with yaw $yaw"

    local cmd="
        TIMEOUT=10 \
        PX4_SYS_AUTOSTART=$PX4_SYS_AUTOSTART \
        PX4_GZ_MODEL_POSE=\"$pose\" \
        PX4_SIM_MODEL=$PX4_MODEL \
        PX4_GZ_STANDALONE=1 \
        "

    if [ "$HEADLESS" -eq 1 ]; then
      cmd="$cmd HEADLESS=1"    
    fi
    echo "$cmd $SWARMZ4_PATH/PX4-Autopilot/build/px4_sitl_default/bin/px4 -i $instance_id; bash"
    gnome-terminal --tab --title="px4_$instance_id" -- sh -c "$cmd $SWARMZ4_PATH/PX4-Autopilot/build/px4_sitl_default/bin/px4 -i $instance_id; bash"
}

# Launch drones
launch_team() {
    local team_num=$1
    local start_pos_x=$2
    local start_pos_y=$3
    local yaw=$4

    for i in $(seq 1 $NUM_DRONES_PER_TEAM); do
        local instance_id=$(($([ "$team_num" -eq 1 ] && echo "$NUM_DRONES_PER_TEAM - $i + 1" || echo "$NUM_DRONES_PER_TEAM + $i")))
        local y_pos=$(($([ "$team_num" -eq 1 ] && echo "$start_pos_y + $i - 1" || echo "$start_pos_y + $i - 1")))
        
        echo "Launching PX4 instance $instance_id for Team $team_num..."
        launch_px4_instance "$instance_id" "$start_pos_x" "$y_pos" "$yaw"
    done
}

### Main Execution ###

# Cleanup function
cleanup() {
    echo "Cleaning up processes..."
    kill_processes
    kill $CLOCK_BRIDGE_PID $LASER_BRIDGE_PID 2>/dev/null
    # exit 0
}
trap cleanup INT TERM

# Launch teams
launch_team 1 0 0 0            # Team 1 at x=0, y=0, facing forward
launch_team 2 $FIELD_LENGTH $FIELD_WIDTH-$NUM_DRONES_PER_TEAM+1 3.14159  # Team 2 at x=FIELD_LENGTH, y=246, facing Team 1

### Launch QGroundControl ###
echo "Launching QGroundControl..."
cd $SWARMZ4_PATH/launch_scripts || { echo "launch_scripts directory not found!"; exit 1; }
./QGroundControl.AppImage &

### Launch Gazebo ###

# Conditionally Launch Gazebo
if [ "$HEADLESS" -ne 1 ]; then
  echo "HEADLESS mode is disabled. Launching Gazebo standalone."
  cd $SWARMZ4_PATH/PX4-Autopilot/Tools/simulation/gz || { echo "Gazebo tools directory not found!"; exit 1; }
  gnome-terminal --tab --title="gazebo" -- sh -c "python3 simulation-gazebo --world $WORLD; bash"
  sleep 10 # Wait for Gazebo to fully launch
else
  echo "HEADLESS mode is enabled. Launching Gazebo in headless mode."
  cd $SWARMZ4_PATH/PX4-Autopilot/Tools/simulation/gz || { echo "Gazebo tools directory not found!"; exit 1; }
  gnome-terminal --tab --title="gazebo" -- sh -c "python3 simulation-gazebo --world $WORLD --headless; bash"
  sleep 10 # Wait for Gazebo to fully launch
fi

### ROS 2 Setup ###
cd $SWARMZ4_PATH/ros2_ws || { echo "ROS 2 workspace directory not found!"; exit 1; }
source install/setup.bash

# ROS 2 Bridges and Launch Files
echo "Starting ROS 2 bridges and launch files..."
ros2 run ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock &
CLOCK_BRIDGE_PID=$!
ros2 launch px4_gz_bridge px4_laser_gz_bridge.launch.py nb_of_drones:=$TOTAL_DRONES &
LASER_BRIDGE_PID=$!

# Wait for user to exit
echo "Press Ctrl+C to terminate all processes."
trap "cleanup; exit 0" INT

wait
