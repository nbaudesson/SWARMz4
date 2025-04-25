#!/bin/bash

#############################################################################
# SWARMz4 Game Launcher
# ====================
#
# Description:
#   This script launches a multi-drone simulation environment for drone combat games.
#   It handles spawning two teams of drones at random positions on opposite sides of
#   the field, configures their controllers, and sets up all required ROS2 and PX4
#   components.
#
# Features:
#   - Automatic process cleanup before launch
#   - Random team spawn positions
#   - Configurable number of drones per team
#   - Customizable field dimensions
#   - Multiple headless mode levels support
#   - Automatic spawn position recording
#   - Process monitoring and cleanup
#
# Requirements:
#   - ROS2 Humble
#   - PX4-Autopilot
#   - Gazebo Garden
#   - MicroXRCE-DDS Agent
#   - QGroundControl
#   - SWARMz4 ROS2 packages
#
# Usage:
#   ./launch_game.sh [HEADLESS_LEVEL] [SPAWN_FILE] [DRONES_PER_TEAM] [FIELD_LENGTH] [FIELD_WIDTH] [WORLD]
#
# Arguments:
#   HEADLESS_LEVEL    : Headless operation level (default: 0)
#                        0 = Full GUI mode (everything has GUI)
#                        1 = Partial headless (Gazebo headless, rest with GUI)
#                        2 = Full headless (all components headless, minimal logs)
#   SPAWN_FILE        : Path to save spawn positions (default: config/spawn_position.yaml)
#   DRONES_PER_TEAM   : Number of drones per team (default: 5)
#   FIELD_LENGTH      : Field length in meters (default: 500)
#   FIELD_WIDTH       : Field width in meters (default: 250)
#   WORLD             : Gazebo world file name (default: swarmz_world)
# 
# Examples and additional notes in original script header...
#############################################################################

#===========================================================================
# 1. SWARMZ PATH SETUP
#===========================================================================
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PARENT_DIR="$(dirname "$SCRIPT_DIR")"
# Make sure SWARMZ4_PATH exists
source "$PARENT_DIR/install_scripts/check_swarmz_path.sh"
export SWARMZ4_PATH
# Make sure Gazebo maritime plugin path is set
source "$PARENT_DIR/install_scripts/set_gazebo_path.sh"
export GZ_SIM_SYSTEM_PLUGIN_PATH=$GZ_SIM_SYSTEM_PLUGIN_PATH:$SWARMZ4_PATH/ros2_ws/install/gazebo_maritime/lib
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$SWARMZ4_PATH/ros2_ws/install/gazebo_maritime/lib

#===========================================================================
# 2. DEFAULT PARAMETERS
#===========================================================================
# Simulation Parameters
PX4_MODEL="gz_x500_lidar_front" # Change to "gz_x500" for classic x500
PX4_SYS_AUTOSTART=4017          # Use 4001 for classic x500, 4017 for x500 with lidar
FIELD_LENGTH=500
FIELD_WIDTH=250
NUM_DRONES_PER_TEAM=5
TOTAL_DRONES=$((NUM_DRONES_PER_TEAM * 2))
HEADLESS_LEVEL=0                # 0=GUI, 1=Gazebo headless, 2=Full headless
WORLD="swarmz_world_2"
SPAWN_POSITION_FILE="$SWARMZ4_PATH/ros2_ws/src/px4_pkgs/px4_controllers/offboard_control_py/config/spawn_position.yaml"

# Bridge and Process PIDs
CLOCK_BRIDGE_PID=""
LASER_BRIDGE_PIDS=()

#===========================================================================
# 3. ARGUMENT PARSING
#===========================================================================
if [ -n "$1" ]; then
  HEADLESS_LEVEL=$1
  if ! [[ "$HEADLESS_LEVEL" =~ ^[0-2]$ ]]; then
    echo "Error: HEADLESS_LEVEL must be 0, 1, or 2"
    exit 1
  fi
fi

if [ -n "$2" ]; then
  SPAWN_POSITION_FILE=$2
fi

if [ -n "$3" ]; then
  NUM_DRONES_PER_TEAM=$3
  TOTAL_DRONES=$((NUM_DRONES_PER_TEAM * 2))
fi

if [ -n "$4" ]; then
  FIELD_LENGTH=$4
fi

if [ -n "$5" ]; then
  FIELD_WIDTH=$5
fi

if [ -n "$6" ]; then
  WORLD=$6
fi

# Set output suppression for headless mode
if [ "$HEADLESS_LEVEL" -eq 2 ]; then
  SUPPRESS_OUTPUT=true
  echo "Full headless mode activated. Output suppressed."
fi

#===========================================================================
# 4. UTILITY FUNCTIONS
#===========================================================================
# Terminal handling with headless mode support
launch_terminal() {
    local title="$1"
    local command="$2"
    local headless_level="$3"
    
    # For level 2 (full headless), just run in background
    if [ "$headless_level" -eq 2 ]; then
        echo "Starting process: $title (silent background)"
        eval "$command > /dev/null 2>&1 &"
        return
    fi
    
    # For GUI modes, try to use gnome-terminal or fall back to xterm
    if command -v gnome-terminal >/dev/null 2>&1; then
        gnome-terminal --tab --title="$title" -- bash -c "$command; echo 'Press Enter to close'; read"
    else
        # Fallback to xterm if available, otherwise run in background
        if command -v xterm >/dev/null 2>&1; then
            echo "No gnome-terminal found. Using xterm for $title."
            xterm -T "$title" -e "$command; echo 'Press Enter to close'; read" &
        else
            # If neither terminal is available, run in background
            echo "No gnome-terminal or xterm found. Running $title in background."
            eval "$command &"
        fi
    fi
}

# Process cleanup function
kill_processes() {
    echo "Cleaning up running processes..."
    local processes=("px4" "MicroXRCEAgent" "gz sim" "parameter_bridge" "QGroundControl")
    
    for proc in "${processes[@]}"; do
        echo "Stopping $proc processes..."
        pkill -f "$proc" || echo "No $proc processes found."
        # Second attempt for stubborn processes
        sleep 1
        pkill -9 -f "$proc" 2>/dev/null
    done
}

# Cleanup function for graceful exit
cleanup() {
    echo "Initiating cleanup..."
    kill_processes
    
    if [ -n "$CLOCK_BRIDGE_PID" ]; then
        kill $CLOCK_BRIDGE_PID 2>/dev/null && echo "Stopped clock bridge"
    fi
    
    # Clean up laser bridges if they exist
    if [ ${#LASER_BRIDGE_PIDS[@]} -gt 0 ]; then
        echo "Stopping laser bridges..."
        for pid in "${LASER_BRIDGE_PIDS[@]}"; do
            kill $pid 2>/dev/null
        done
        echo "Laser bridges stopped"
    fi
    
    echo "Cleanup complete"
}

# GPU detection function
has_secondary_gpu() {
    # Check if lspci is available
    if ! command -v lspci &> /dev/null; then
        echo "lspci not found, can't detect GPUs properly"
        return 1
    fi
    
    # Get GPU info
    gpu_info=$(lspci | grep -E "VGA|3D")
    
    # Check if we have more than one GPU
    gpu_count=$(echo "$gpu_info" | wc -l)
    if [ "$gpu_count" -lt 2 ]; then
        return 1
    fi
    
    # Check if we have a non-Intel GPU (likely dedicated)
    if echo "$gpu_info" | grep -v "Intel" | grep -E "NVIDIA|AMD|ATI" &> /dev/null; then
        # Check if the primary GPU is Intel (meaning dedicated GPU is secondary)
        if echo "$gpu_info" | head -1 | grep "Intel" &> /dev/null; then
            echo "Detected secondary dedicated GPU"
            return 0
        fi
    fi
    
    return 1
}

#===========================================================================
# 5. SPAWN POSITION FUNCTIONS
#===========================================================================
# Handle spawn position file for tracking drone positions
handle_spawn_position() {
    local action=$1  # 'init' or 'update'
    local file=$2
    local team_id=$3
    local drone_id=$4
    local x=$5
    local y=$6

    case $action in
        init)
            # Initialize the YAML structure with empty positions
            cat > "$file" << EOL
# Drone spawn positions
# Format: team_id:{drone_id:{x, y, yaw}, ...}
{
  "1":{
EOL
            # Team 1 drones (starting from 0)
            for ((i=0; i<NUM_DRONES_PER_TEAM; i++)); do
                echo "    \"$i\":{
      x: 0,
      y: 0,
      yaw: 90}," >> "$file"
            done
            echo "  }," >> "$file"
            
            # Team 2 drones (starting from NUM_DRONES_PER_TEAM)
            echo "  \"2\":{" >> "$file"
            for ((i=0; i<NUM_DRONES_PER_TEAM; i++)); do
                local d_id=$((i + NUM_DRONES_PER_TEAM))
                echo "    \"$d_id\":{
      x: 0,
      y: 0,
      yaw: 90}," >> "$file"
            done
            echo "  }," >> "$file"
            echo "}" >> "$file"
            ;;
            
        update)
            # Swap x and y when writing to file to fit for NED coordinates in Gazebo
            sed -i "/\"$drone_id\":{/,/}/{s/x: [0-9.-]*/x: $y/}" "$file"
            sed -i "/\"$drone_id\":{/,/}/{s/y: [0-9.-]*/y: $x/}" "$file"
            # Always set yaw to 90 (facing East) regardless of team
            sed -i "/\"$drone_id\":{/,/}/{s/yaw: [0-9.-]*/yaw: 90/}" "$file"
            ;;
    esac
}

# Generate random team position within bounds
generate_team_position() {
    local min_x=$1
    local max_x=$2
    local min_y=$3
    local max_y=$4
    
    # Generate random x within bounds
    local x=$(awk -v min=$min_x -v max=$max_x 'BEGIN{srand(); print int(min+rand()*(max-min+1))}')
    # Generate random y, leaving space for the line of drones
    local max_y_adjusted=$((max_y - (NUM_DRONES_PER_TEAM * 2)))
    local y=$(awk -v min=$min_y -v max=$max_y_adjusted 'BEGIN{srand(); print int(min+rand()*(max-min+1))}')
    
    echo "$x $y"
}

#===========================================================================
# 6. DRONE SPAWN & MANAGEMENT FUNCTIONS
#===========================================================================
# Launch PX4 instance with position
launch_px4_instance() {
    local instance_id=$1
    local x_pos=$2  
    local y_pos=$3
    local pose="$x_pos,$y_pos,0,0,0,0"  # Fixed yaw to 0 in Gazebo (will be 90 in YAML)
    
    # Special case for first drone (ID 0) - launches Gazebo simulation
    if [ "$instance_id" -eq 0 ]; then
        echo "Launching first PX4 instance at position ($x_pos, $y_pos, 0)"
        cd $SWARMZ4_PATH/PX4-Autopilot || { echo "PX4-Autopilot directory not found!"; exit 1; }
        
        local headless_flag=""
        if [ "$HEADLESS_LEVEL" -ge 1 ]; then
            headless_flag="HEADLESS=1"
        fi
        
        # Determine whether to use switcherooctl based on GPU availability
        local cmd=""
        if has_secondary_gpu; then
            echo "Using switcherooctl to leverage dedicated GPU"
            cmd="switcherooctl launch env PX4_UXRCE_DDS_NS=px4_$instance_id VERBOSE_SIM=1 PX4_SYS_AUTOSTART=$PX4_SYS_AUTOSTART PX4_SIM_MODEL=$PX4_MODEL PX4_GZ_MODEL_POSE=\"$pose\" PX4_GZ_WORLD=\"$WORLD\" $headless_flag make px4_sitl $PX4_MODEL"
        else
            echo "Using standard GPU configuration"
            cmd="PX4_UXRCE_DDS_NS=px4_$instance_id VERBOSE_SIM=1 PX4_SYS_AUTOSTART=$PX4_SYS_AUTOSTART PX4_SIM_MODEL=$PX4_MODEL PX4_GZ_MODEL_POSE=\"$pose\" PX4_GZ_WORLD=\"$WORLD\" $headless_flag make px4_sitl $PX4_MODEL"
        fi

        # Launch with EKF reset script
        launch_terminal "px4_$instance_id" "$SWARMZ4_PATH/launch_scripts/px4_reset_sensors.sh \"$cmd\"" "$HEADLESS_LEVEL"
        
        echo "Waiting 12 seconds for first PX4 instance and Gazebo to initialize..."
        sleep 12  # Increased wait time for better initialization
    else
        echo "Launching PX4 instance $instance_id at position ($x_pos, $y_pos, 0)"
        local cmd="
            TIMEOUT=10 \
            VERBOSE_SIM=1 \
            PX4_UXRCE_DDS_NS=px4_$instance_id \
            PX4_SYS_AUTOSTART=$PX4_SYS_AUTOSTART \
            PX4_GZ_MODEL_POSE=\"$pose\" \
            PX4_SIM_MODEL=$PX4_MODEL \
            PX4_GZ_WORLD=\"$WORLD\" \
            "

        # Either Gazebo headless or full headless
        if [ "$HEADLESS_LEVEL" -ge 1 ]; then
          cmd="$cmd HEADLESS=1"
        fi

        cmd="$cmd $SWARMZ4_PATH/PX4-Autopilot/build/px4_sitl_default/bin/px4 -i $instance_id"
        
        # Launch with EKF reset script
        launch_terminal "px4_$instance_id" "$SWARMZ4_PATH/launch_scripts/px4_reset_sensors.sh \"$cmd\"" "$HEADLESS_LEVEL"
        
        # Add longer sleep between drone launches
        echo "Waiting for PX4 instance $instance_id to initialize..."
        sleep 1
    fi
}

# Launch a team of drones with positioning
launch_team() {
    local team_num=$1
    local min_x=$2
    local max_x=$3
    local min_y=$4
    local max_y=$5
    
    # Generate random base position
    local start_x=$(awk -v min=$min_x -v max=$max_x 'BEGIN{srand(); print int(min+rand()*(max-min+1))}')
    local start_y=$(awk -v min=$min_y -v max=$((max_y - (NUM_DRONES_PER_TEAM * 2))) 'BEGIN{srand(); print int(min+rand()*(max-min+1))}')
    
    echo "Spawning Team $team_num at base position ($start_x, $start_y)"
    
    # Launch drones in formation
    for ((i=0; i<NUM_DRONES_PER_TEAM; i++)); do
        local drone_id=$((team_num == 1 ? i : NUM_DRONES_PER_TEAM + i))
        local drone_y=$((start_y + i*2))  # 2-meter spacing
        
        echo "Launching drone $drone_id at ($start_x, $drone_y)"
        launch_px4_instance "$drone_id" "$start_x" "$drone_y"
        handle_spawn_position "update" "$SPAWN_POSITION_FILE" "$team_num" "$drone_id" "$start_x" "$drone_y"
        
        echo "Drone $drone_id launched and spawn position updated."
    done
}

#===========================================================================
# 7. BRIDGE FUNCTIONS
#===========================================================================
# Start laser bridges for all drones
start_laser_bridges() {
    # Only start laser bridges if using the lidar-equipped model
    if [[ "$PX4_MODEL" != "gz_x500_lidar_front" ]]; then
        echo "Skipping laser bridges as model $PX4_MODEL is not lidar-equipped"
        return
    fi
    echo "Starting laser bridges for $TOTAL_DRONES drones..."

    # Set log level to ERROR to suppress INFO and WARN messages
    local log_level="ERROR"
    
    for ((i=0; i<TOTAL_DRONES; i++)); do
        # Construct the GZ laser topic path
        local gz_topic="/world/$WORLD/model/${PX4_MODEL}_${i}/link/lidar_sensor_link/sensor/lidar/scan"
        local ros_topic="px4_${i}/laser/scan"
        
        # Start the bridge with log level control and unique node name
        echo "Starting laser bridge for drone $i: $gz_topic -> $ros_topic"
        ros2 run ros_gz_bridge parameter_bridge "$gz_topic@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan" \
            --ros-args -r "$gz_topic:=$ros_topic" --log-level ros_gz_bridge:=$log_level -r __node:=laser_bridge_drone_${i} > /dev/null 2>&1 &
        
        # Store the PID for cleanup
        LASER_BRIDGE_PIDS+=($!)        
        # Short delay to avoid overwhelming the system
        sleep 0.1
    done
    echo "All laser bridges started successfully"
}

# Start the clock bridge
start_clock_bridge() {
    echo "Starting ROS 2 bridge for clock synchronization..."
    
    # Set log level to ERROR to suppress INFO and WARN messages
    local log_level="ERROR"

    # Launch clock bridge with log level control and unique node name
    ros2 run ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock \
        --ros-args --log-level ros_gz_bridge:=$log_level -r __node:=clock_bridge > /dev/null 2>&1 &
    
    CLOCK_BRIDGE_PID=$!
    echo "Clock bridge started (PID: $CLOCK_BRIDGE_PID)."
}

#===========================================================================
# 8. MAIN EXECUTION
#===========================================================================
# Register cleanup handler
trap cleanup INT TERM

# Step 1: Clean up any existing processes
echo "Step 1: Cleaning up existing processes..."
kill_processes

# Step 2: Start MicroXRCE-DDS Agent
echo "Step 2: Starting MicroXRCE-DDS Agent..."
cd $SWARMZ4_PATH/Micro-XRCE-DDS-Agent || { echo "Micro-XRCE-DDS-Agent directory not found!"; exit 1; }
launch_terminal "MicroXRCEAgent" "MicroXRCEAgent udp4 -p 8888 -v 4" "$HEADLESS_LEVEL"
if [ "$HEADLESS_LEVEL" -eq 2 ]; then
    echo "Started MicroXRCEAgent with output suppressed"
else
    echo "Started MicroXRCEAgent with verbose logging."
fi

# Step 3: Initialize spawn positions file
echo "Step 3: Initializing spawn positions file..."
handle_spawn_position "init" "$SPAWN_POSITION_FILE"

# Step 4: Launch QGroundControl
echo "Step 4: Launching QGroundControl..."
if [ "$HEADLESS_LEVEL" -eq 2 ]; then
    # Full headless mode uses xvfb-run with minimal logging
    xvfb-run -a $SWARMZ4_PATH/launch_scripts/QGroundControl.AppImage > /dev/null 2>&1 &
    echo "Started QGroundControl with minimal logging"
else
    # GUI mode
    $SWARMZ4_PATH/launch_scripts/QGroundControl.AppImage > /dev/null 2>&1 &
fi

# Step 5: Launch PX4 teams
echo "Step 5: Launching drone teams..."
cd $SWARMZ4_PATH/PX4-Autopilot || { echo "PX4-Autopilot directory not found!"; exit 1; }
# Team 1: x=[0,20%], y=[0,WIDTH]
launch_team 1 0 $((FIELD_LENGTH/5)) 0 $FIELD_WIDTH
# Team 2: x=[80%,100%], y=[0,WIDTH]
launch_team 2 $((FIELD_LENGTH*4/5)) $FIELD_LENGTH 0 $FIELD_WIDTH

# Step 6: Start ROS 2 bridges
echo "Step 6: Starting ROS 2 bridges..."
cd $SWARMZ4_PATH/ros2_ws || { echo "ROS 2 workspace directory not found!"; exit 1; }
source install/setup.bash
 
# Start the clock bridge
start_clock_bridge

# Start the laser bridges
start_laser_bridges

# Step 7: Wait for user to exit
echo "Step 7: All processes started."
if [ "$HEADLESS_LEVEL" -eq 2 ]; then
    echo "Running in headless mode. Press Ctrl+C to terminate all processes."
else
    echo "Press Ctrl+C to terminate all processes."
fi

trap "cleanup; exit 0" INT
wait
