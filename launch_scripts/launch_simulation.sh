#! /bin/bash

# Check if SWARMZ4_PATH is already set and validate it
if [ -n "$SWARMZ4_PATH" ]; then
    # Check if the path contains spaces or newlines (indicating multiple paths)
    if [[ "$SWARMZ4_PATH" == *" "* || "$SWARMZ4_PATH" == *$'\n'* ]]; then
        echo "Warning: SWARMZ4_PATH contains spaces or line returns. It may be invalid."
        echo "Resetting SWARMZ4_PATH..."
        unset SWARMZ4_PATH
    else
        echo "Using pre-existing SWARMZ4_PATH: $SWARMZ4_PATH"
    fi
fi

# If SWARMZ4_PATH is not set or was reset, determine it from script location
if [ -z "$SWARMZ4_PATH" ]; then
    # Find the directory where this script is located and go up 1 level to reach SWARMZ4 root
    SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    SWARMZ4_PATH="$(dirname "$SCRIPT_DIR")"
    
    echo "Set SWARMZ4_PATH to: $SWARMZ4_PATH"
    
    # Export SWARMZ4_PATH so it persists for subsequent scripts
    export SWARMZ4_PATH
    
    # Update or add SWARMZ4_PATH to ~/.bashrc for persistence
    if grep -q "export SWARMZ4_PATH=" ~/.bashrc; then
        sed -i "s|export SWARMZ4_PATH=.*|export SWARMZ4_PATH=\"$SWARMZ4_PATH\"|g" ~/.bashrc
        echo "Replacing ~/.bashrc SWARMZ4_PATH with new SWARMZ4_PATH"
    else
        echo "export SWARMZ4_PATH=\"$SWARMZ4_PATH\"" >> ~/.bashrc
        echo "Updated ~/.bashrc with SWARMZ4_PATH"
    fi
fi

# Create log directory structure for headless mode
create_log_dir() {
    local timestamp=$(date +"%Y-%m-%d_%H-%M-%S")
    LOG_DIR="$SWARMZ4_PATH/logs/simulation_$timestamp"
    
    mkdir -p "$LOG_DIR"
    echo "Created log directory: $LOG_DIR"
}

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

# Create log directory if in headless mode
if [ "$HEADLESS" -eq 1 ]; then
  create_log_dir
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
      $cmd $SWARMZ4_PATH/PX4-Autopilot/build/px4_sitl_default/bin/px4 -i $instance_id > "$LOG_DIR/px4_$instance_id.log" 2>&1 &
      echo "PX4 instance $instance_id started in background. Logs: $LOG_DIR/px4_$instance_id.log"
    else
      echo "$cmd $SWARMZ4_PATH/PX4-Autopilot/build/px4_sitl_default/bin/px4 -i $instance_id; bash"
      gnome-terminal --tab --title="px4_$instance_id" -- sh -c "$cmd $SWARMZ4_PATH/PX4-Autopilot/build/px4_sitl_default/bin/px4 -i $instance_id; bash"
    fi
}

# Launch drones
launch_team() {
    local team_num=$1
    local start_pos_x=$2
    local start_pos_y=$3
    local yaw=$4

    for i in $(seq 1 $NUM_DRONES_PER_TEAM); do
        # Calculate instance ID (1-5 for team 1, 6-10 for team 2)
        local instance_id
        if [ "$team_num" -eq 1 ]; then
            instance_id=$i
        else
            instance_id=$((NUM_DRONES_PER_TEAM + i))
        fi
        
        # 2m spacing between drones
        local y_pos=$(((i - 1) * 2))  # This will place drones at 0, 2, 4, 6, 8 etc.
        
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

### Start Micro-XRCE-DDS Agent ###
cd $SWARMZ4_PATH/Micro-XRCE-DDS-Agent || { echo "Micro-XRCE-DDS-Agent directory not found!"; exit 1; }
if [ "$HEADLESS" -eq 1 ]; then
    MicroXRCEAgent udp4 -p 8888 > "$LOG_DIR/microxrce_agent.log" 2>&1 &
    echo "Started MicroXRCEAgent in background. Logs: $LOG_DIR/microxrce_agent.log"
else
    gnome-terminal --tab --title="MicroXRCEAgent" -- sh -c "MicroXRCEAgent udp4 -p 8888; bash"
    echo "Started MicroXRCEAgent."
fi

# Launch teams
launch_team 1 0 0 0                  # Team 1 at x=0, starting from y=0
launch_team 2 $FIELD_LENGTH $FIELD_WIDTH 3.14159  # Team 2 at x=FIELD_LENGTH, starting from y=FIELD_WIDTH

### Launch QGroundControl ###
echo "Launching QGroundControl..."
cd $SWARMZ4_PATH/launch_scripts || { echo "launch_scripts directory not found!"; exit 1; }
if [ "$HEADLESS" -eq 1 ]; then
    ./QGroundControl.AppImage > "$LOG_DIR/qgc.log" 2>&1 &
    echo "Started QGroundControl in background. Logs: $LOG_DIR/qgc.log"
else
    # Full headless mode uses xvfb-run
    xvfb-run -a ./QGroundControl.AppImage &
fi

### Launch Gazebo ###
cd $SWARMZ4_PATH/PX4-Autopilot/Tools/simulation/gz || { echo "Gazebo tools directory not found!"; exit 1; }
if [ "$HEADLESS" -eq 1 ]; then
    echo "HEADLESS mode is enabled. Launching Gazebo in headless mode."
    python3 simulation-gazebo --world $WORLD --headless > "$LOG_DIR/gazebo.log" 2>&1 &
    echo "Started Gazebo in background. Logs: $LOG_DIR/gazebo.log"
else
    echo "HEADLESS mode is disabled. Launching Gazebo standalone."
    gnome-terminal --tab --title="gazebo" -- sh -c "python3 simulation-gazebo --world $WORLD; bash"
fi
sleep 10 # Wait for Gazebo to fully launch

### ROS 2 Setup ###
cd $SWARMZ4_PATH/ros2_ws || { echo "ROS 2 workspace directory not found!"; exit 1; }
source install/setup.bash

# ROS 2 Bridges and Launch Files
echo "Starting ROS 2 bridges and launch files..."
if [ "$HEADLESS" -eq 1 ]; then
    ros2 run ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock > "$LOG_DIR/clock_bridge.log" 2>&1 &
    CLOCK_BRIDGE_PID=$!
    echo "Started clock bridge in background. Logs: $LOG_DIR/clock_bridge.log"
    
    ros2 launch px4_gz_bridge px4_laser_gz_bridge.launch.py nb_of_drones:=$TOTAL_DRONES > "$LOG_DIR/laser_bridge.log" 2>&1 &
    LASER_BRIDGE_PID=$!
    echo "Started laser bridge in background. Logs: $LOG_DIR/laser_bridge.log"
else
    ros2 run ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock &
    CLOCK_BRIDGE_PID=$!
    ros2 launch px4_gz_bridge px4_laser_gz_bridge.launch.py nb_of_drones:=$TOTAL_DRONES &
    LASER_BRIDGE_PID=$!
fi

# Wait for user to exit
echo "Press Ctrl+C to terminate all processes."
trap "cleanup; exit 0" INT

if [ "$HEADLESS" -eq 1 ]; then
    echo "All processes started in background."
    echo "Log directory: $LOG_DIR"
    echo "Waiting... Press Ctrl+C to terminate all processes."
fi

wait