#!/bin/bash

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
# Examples:
#   # Launch with default settings (GUI mode)
#   ./launch_game.sh
#
#   # Launch with Gazebo headless
#   ./launch_game.sh 1
#
#   # Launch in full headless mode with no GUI terminals
#   ./launch_game.sh 2
#
#   # Launch with custom drone settings
#   ./launch_game.sh 0 default_spawn.yaml 5 500 250
#
# Features:
#   - Team 1 spawns randomly between x=[0,100], y=[0,FIELD_WIDTH]
#   - Team 2 spawns randomly between x=[400,500], y=[0,FIELD_WIDTH]
#   - Drones within each team spawn in a line with 2m spacing
#   - Spawn positions are recorded to YAML file for use by controllers
#   - All processes are properly terminated on exit
#
# Notes:
#   - Team 1 drones face forward (0 degrees)
#   - Team 2 drones face team 1 (180 degrees)
#   - The script requires SWARMZ4_PATH environment variable or finds it automatically
#   - QGroundControl provides visual feedback and manual control options
#   - Use Ctrl+C to properly terminate all processes
#

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

# Terminal handling function that supports gnome-terminal or background processes
launch_terminal() {
    local title="$1"
    local command="$2"
    local headless_level="$3"
    
    # For level 2 (full headless), just run in background with minimal output
    if [ "$headless_level" -eq 2 ]; then
        # Run in background with reduced output
        if [ -z "$SUPPRESS_OUTPUT" ]; then
            echo "Starting process: $title"
            eval "$command > /dev/null 2>&1 &"
        else
            eval "$command &"
        fi
        return
    fi
    
    # For GUI modes, try to use gnome-terminal or fall back to background
    if command -v gnome-terminal >/dev/null 2>&1; then
        gnome-terminal --tab --title="$title" -- bash -c "$command; echo 'Press Enter to close'; read"
    else
        # Fallback to just running in background
        echo "No gnome-terminal found. Running $title in background."
        eval "$command &"
    fi
}

# Consolidated process killing function with better feedback
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

# Call kill_processes function
kill_processes

### Start Micro-XRCE-DDS Agent ###
cd $SWARMZ4_PATH/Micro-XRCE-DDS-Agent || { echo "Micro-XRCE-DDS-Agent directory not found!"; exit 1; }

# Parameters
PX4_MODEL="gz_x500_lidar_front" # Change to "gz_x500" for classic x500
PX4_SYS_AUTOSTART=4017 # Use 4001 for classic x500
FIELD_LENGTH=500
FIELD_WIDTH=250
NUM_DRONES_PER_TEAM=5
TOTAL_DRONES=$((NUM_DRONES_PER_TEAM * 2))
HEADLESS_LEVEL=0 # 0=GUI, 1=Gazebo headless, 2=Full headless
WORLD="swarmz_world_2"
SPAWN_POSITION_FILE="$SWARMZ4_PATH/ros2_ws/src/px4_pkgs/px4_controllers/offboard_control_py/config/spawn_position.yaml"

# Arguments
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

# For full headless mode, set up logging reduction
if [ "$HEADLESS_LEVEL" -eq 2 ]; then
  SUPPRESS_OUTPUT=true
  echo "Full headless mode activated. Reducing log output."
fi

launch_terminal "MicroXRCEAgent" "MicroXRCEAgent udp4 -p 8888" "$HEADLESS_LEVEL"
echo "Started MicroXRCEAgent."

### PX4 Drone Configuration ###
cd $SWARMZ4_PATH/PX4-Autopilot || { echo "PX4-Autopilot directory not found!"; exit 1; }

# Store spawn positions in memory
declare -A spawn_positions

# Simplified spawn position handling using a single function
handle_spawn_position() {
    local action=$1  # 'init' or 'update'
    local file=$2
    local team_id=$3
    local drone_id=$4
    local x=$5
    local y=$6
    local yaw=$7

    case $action in
        init)
            # Initialize the YAML structure with empty positions
            cat > "$file" << EOL
# Drone spawn positions
# Format: team_id:{drone_id:{x, y, yaw}, ...}
{
  "1":{
EOL
            # Team 1 drones
            for ((i=1; i<=NUM_DRONES_PER_TEAM; i++)); do
                echo "    \"$i\":{
      x: 0,
      y: 0,
      yaw: 0}," >> "$file"
            done
            echo "  }," >> "$file"
            
            # Team 2 drones
            echo "  \"2\":{" >> "$file"
            for ((i=1; i<=NUM_DRONES_PER_TEAM; i++)); do
                local d_id=$((i + NUM_DRONES_PER_TEAM))
                echo "    \"$d_id\":{
      x: 0,
      y: 0,
      yaw: 180}," >> "$file"
            done
            echo "  }," >> "$file"
            echo "}" >> "$file"
            ;;
            
        update)
            # Transform Gazebo coordinates to NED coordinates in the YAML file:
            # Convert yaw from radians to degrees and add 90° offset to align with North
            # 1. $yaw * 180/3.14159   -> converts from radians to degrees
            # 2. + 90                 -> adds 90° offset for NED frame (Gazebo 0° = East, NED 0° = North)
            # 3. + 0.5                -> adds 0.5 for proper rounding with int()
            # 4. int()                -> truncates to integer
            local yaw_deg=$(awk "BEGIN {print int($yaw * 180/3.14159 + 90 + 0.5)}")
            # Swap x and y when writing to file to fit for NED coordinates in Gazebo
            sed -i "/\"$drone_id\":{/,/}/{s/x: [0-9.-]*/x: $y/}" "$file"
            sed -i "/\"$drone_id\":{/,/}/{s/y: [0-9.-]*/y: $x/}" "$file"
            sed -i "/\"$drone_id\":{/,/}/{s/yaw: [0-9.-]*/yaw: $yaw_deg/}" "$file"
            ;;
    esac
}

# Launch PX4 Instances with improved pose handling
launch_px4_instance() {
    local instance_id=$1
    local x_pos=$2  
    local y_pos=$3
    local yaw=${4:-0}  # Default yaw to 0 if not specified
    local pose="$x_pos,$y_pos,0,0,0,$yaw"
    echo "Launching PX4 instance $instance_id at position ($x_pos, $y_pos, 0) with yaw $yaw"
    # PX4_GZ_STANDALONE=1 \
    local cmd="
        TIMEOUT=10 \
        PX4_SYS_AUTOSTART=$PX4_SYS_AUTOSTART \
        PX4_GZ_MODEL_POSE=\"$pose\" \
        PX4_SIM_MODEL=$PX4_MODEL \
        PX4_SYS_AUTOSTART=$PX4_SYS_AUTOSTART \
        PX4_GZ_WORLD=$WORLD \
        "

    # Either Gazebo headless or full headless
    if [ "$HEADLESS_LEVEL" -ge 1 ]; then
      cmd="$cmd HEADLESS=1"
    fi

    cmd="$cmd $SWARMZ4_PATH/PX4-Autopilot/build/px4_sitl_default/bin/px4 -i $instance_id"
    launch_terminal "px4_$instance_id" "$cmd" "$HEADLESS_LEVEL"
    
    # Add sleep for first PX4 instance to ensure initialization
    if [ "$instance_id" -eq 1 ]; then
        echo "Waiting 10 seconds for PX4 instance 1 to initialize..."
        sleep 5
    else
        sleep 1
    fi
}

# Launch PX4 Instances with improved pose handling
launch_px4_instance_bis() {
    local instance_id=$1
    local x_pos=$2  
    local y_pos=$3
    local yaw=${4:-0}  # Default yaw to 0 if not specified
    local pose="$x_pos,$y_pos,0,0,0,$yaw" # Yaw doesn't work without the standalone
    echo "Launching PX4 instance $instance_id at position ($x_pos, $y_pos, 0) with yaw $yaw"
    # PX4_GZ_STANDALONE=1 \
    local cmd="
        cd $SWARMZ4_PATH/PX4-Autopilot && \
        TIMEOUT=10 \
        PX4_UXRCE_DDS_NS=px4_$instance_id \
        PX4_SYS_AUTOSTART=$PX4_SYS_AUTOSTART \
        PX4_GZ_MODEL_POSE=\"$pose\" \
        PX4_SIM_MODEL=$PX4_MODEL \
        "

    # Either Gazebo headless or full headless
    if [ "$HEADLESS_LEVEL" -ge 1]; then
      cmd="$cmd HEADLESS=1"
    fi

    cmd="$cmd $SWARMZ4_PATH/PX4-Autopilot/build/px4_sitl_default/bin/px4 -i $instance_id"
    launch_terminal "px4_$instance_id" "$cmd" "$HEADLESS_LEVEL"
}

# Add random position generator function
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

# Improved team spawning with better position calculation
launch_team() {
    local team_num=$1
    local min_x=$2
    local max_x=$3
    local min_y=$4
    local max_y=$5
    local yaw=$6
    
    # Generate random base position
    local start_x=$(awk -v min=$min_x -v max=$max_x 'BEGIN{srand(); print int(min+rand()*(max-min+1))}')
    local start_y=$(awk -v min=$min_y -v max=$((max_y - (NUM_DRONES_PER_TEAM * 2))) 'BEGIN{srand(); print int(min+rand()*(max-min+1))}')
    
    echo "Spawning Team $team_num at base position ($start_x, $start_y)"
    
    # Launch drones in formation
    for ((i=1; i<=NUM_DRONES_PER_TEAM; i++)); do
        local drone_id=$((team_num == 1 ? i : NUM_DRONES_PER_TEAM + i))
        local drone_y=$((start_y + (i-1)*2))  # 2-meter spacing
        
        echo "Launching drone $drone_id at ($start_x, $drone_y)"
        launch_px4_instance "$drone_id" "$start_x" "$drone_y" "$yaw"
        handle_spawn_position "update" "$SPAWN_POSITION_FILE" "$team_num" "$drone_id" "$start_x" "$drone_y" "$yaw"
    done
}

### Main Execution ###

# Initialize spawn positions
handle_spawn_position "init" "$SPAWN_POSITION_FILE"

# Cleanup function
cleanup() {
    echo "Initiating cleanup..."
    kill_processes
    if [ -n "$CLOCK_BRIDGE_PID" ]; then
        kill $CLOCK_BRIDGE_PID 2>/dev/null && echo "Stopped clock bridge"
    fi
    if [ -n "$LASER_BRIDGE_PID" ]; then
        kill $LASER_BRIDGE_PID 2>/dev/null && echo "Stopped laser bridge"
    fi
    echo "Cleanup complete"
}

trap cleanup INT TERM

# Launch teams with random positions scaled to field dimensions
launch_team 1 0 $((FIELD_LENGTH/5)) 0 $FIELD_WIDTH 0        # Team 1: x=[0,20%], y=[0,WIDTH], facing forward
launch_team 2 $((FIELD_LENGTH*4/5)) $FIELD_LENGTH 0 $FIELD_WIDTH 3.14159  # Team 2: x=[80%,100%], y=[0,WIDTH], facing Team 1

### Launch QGroundControl ###
echo "Launching QGroundControl..."
cd $SWARMZ4_PATH/launch_scripts || { echo "launch_scripts directory not found!"; exit 1; }

# Launch QGroundControl based on headless level
if [ "$HEADLESS_LEVEL" -eq 2 ]; then
    # Full headless mode uses xvfb-run
    xvfb-run -a ./QGroundControl.AppImage &
else
    # GUI mode
    ./QGroundControl.AppImage &
fi

### Can't manage to make standalone launch work anymore ###
# ### Launch Gazebo ###
# # Conditional Gazebo stanalone Launch based on headless level
# if [ "$HEADLESS_LEVEL" -eq 0 ]; then
#     echo "Full GUI mode. Launching Gazebo with GUI."
#     cd $SWARMZ4_PATH/PX4-Autopilot/Tools/simulation/gz || { echo "Gazebo tools directory not found!"; exit 1; }
#     launch_terminal "gazebo" "python3 simulation-gazebo --world $WORLD --model_store $SWARMZ4_PATH/PX4-Autopilot/Tools/simulation/gz
# else
#     echo "Headless mode enabled. Launching Gazebo in headless mode."
#     cd $SWARMZ4_PATH/PX4-Autopilot/Tools/simulation/gz || { echo "Gazebo tools directory not found!"; exit 1; }
#     launch_terminal "gazebo" "python3 simulation-gazebo --world $WORLD --headless true
# fi

# Wait a bit for Gazebo to initialize
if [ "$HEADLESS_LEVEL" -lt 2 ]; then
    sleep 10  # Normal wait time
else
    sleep 5   # Reduced wait time for headless mode
fi

### ROS 2 Setup ###
cd $SWARMZ4_PATH/ros2_ws || { echo "ROS 2 workspace directory not found!"; exit 1; }
source install/setup.bash

# ROS 2 Bridges and Launch Files
echo "Starting ROS 2 bridges and launch files..."
if [ "$HEADLESS_LEVEL" -eq 2 ]; then
    # Minimal output for full headless mode
    ros2 run ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock > /dev/null 2>&1 &
    CLOCK_BRIDGE_PID=$!
    ros2 launch px4_gz_bridge px4_laser_gz_bridge.launch.py nb_of_drones:=$TOTAL_DRONES > /dev/null 2>&1 &
    LASER_BRIDGE_PID=$!
else
    # Normal output for GUI modes
    ros2 run ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock &
    CLOCK_BRIDGE_PID=$!
    ros2 launch px4_gz_bridge px4_laser_gz_bridge.launch.py nb_of_drones:=$TOTAL_DRONES &
    LASER_BRIDGE_PID=$!
fi

# Wait for user to exit
echo "Press Ctrl+C to terminate all processes."
trap "cleanup; exit 0" INT
wait
