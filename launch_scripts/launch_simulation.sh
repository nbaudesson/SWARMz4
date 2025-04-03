#!/bin/bash

#############################################################################
# SWARMz4 Simulation Launcher
# ==========================
#
# Description:
#   This script launches a simplified multi-drone simulation environment
#   for testing and development. It spawns two small teams of drones at
#   fixed positions for easier debugging.
#
# Features:
#   - Automatic process cleanup before launch
#   - Fixed team spawn positions for predictable testing
#   - Configurable number of drones per team
#   - Customizable field dimensions
#   - Multiple headless mode levels support
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
#   ./launch_simulation.sh [HEADLESS_LEVEL] [DRONES_PER_TEAM] [FIELD_LENGTH] [FIELD_WIDTH] [WORLD]
#############################################################################

#===========================================================================
# 1. SWARMZ PATH SETUP
#===========================================================================
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PARENT_DIR="$(dirname "$SCRIPT_DIR")"
source "$PARENT_DIR/install_scripts/check_swarmz_path.sh"
export SWARMZ4_PATH

#===========================================================================
# 2. DEFAULT PARAMETERS
#===========================================================================
# Simulation Parameters
PX4_MODEL="gz_x500_lidar_front" # Change to "gz_x500" for classic x500
PX4_SYS_AUTOSTART=4017          # Use 4001 for classic x500, 4017 for x500 with lidar
FIELD_LENGTH=5
FIELD_WIDTH=2
NUM_DRONES_PER_TEAM=2
TOTAL_DRONES=$((NUM_DRONES_PER_TEAM * 2))
HEADLESS_LEVEL=0                # 0=GUI, 1=Gazebo headless, 2=Full headless
WORLD="swarmz_world"

# Bridge and Process PIDs
CLOCK_BRIDGE_PID=""
LASER_BRIDGE_PID=""

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

# Set output suppression for headless mode
if [ "$HEADLESS_LEVEL" -eq 2 ]; then
  SUPPRESS_OUTPUT=true
  echo "Full headless mode activated. Output suppressed."
fi

#===========================================================================
# 4. UTILITY FUNCTIONS
#===========================================================================
# Process cleanup function
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

# Terminal handling with headless mode support
launch_terminal() {
    local title="$1"
    local command="$2"
    local headless_level="$3"
    
    # For level 2 (full headless), just run in background with output suppressed
    if [ -n "$headless_level" ] && [ "$headless_level" -eq 2 ]; then
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

# Cleanup function for graceful exit
cleanup() {
    echo "Cleaning up processes..."
    kill_processes
    kill $CLOCK_BRIDGE_PID $LASER_BRIDGE_PID 2>/dev/null
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
        # Only one GPU found
        return 1
    fi
    
    # Check if we have a non-Intel GPU (likely dedicated)
    if echo "$gpu_info" | grep -v "Intel" | grep -E "NVIDIA|AMD|ATI" &> /dev/null; then
        # Check if the primary GPU is Intel (meaning dedicated GPU is secondary)
        if echo "$gpu_info" | head -1 | grep "Intel" &> /dev/null; then
            # We have a dedicated GPU that's not the primary one
            echo "Detected secondary dedicated GPU"
            return 0
        fi
    fi
    
    # No secondary dedicated GPU found
    return 1
}

#===========================================================================
# 5. DRONE SPAWN & MANAGEMENT FUNCTIONS
#===========================================================================
# Launch PX4 instance with position
launch_px4_instance() {
    local instance_id=$1
    local x_pos=$2  
    local y_pos=$3
    local yaw=${4:-0}  # Default yaw to 0 if not specified
    local pose="$x_pos,$y_pos,0,0,0,$yaw"
    
    local headless_flag=""
    if [ -n "$HEADLESS_LEVEL" ] && [ "$HEADLESS_LEVEL" -ge 1 ]; then
        headless_flag="HEADLESS=1"
    fi

    # Special case for first drone (now ID 0) - launches Gazebo simulation
    if [ "$instance_id" -eq 0 ]; then
        echo "Launching first PX4 instance at position ($x_pos, $y_pos, 0) with yaw $yaw (with Gazebo)"
        cd $SWARMZ4_PATH/PX4-Autopilot || { echo "PX4-Autopilot directory not found!"; exit 1; }
        
        # Determine whether to use switcherooctl based on GPU availability
        local cmd=""
        if has_secondary_gpu; then
            echo "Using switcherooctl to leverage dedicated GPU"
            cmd="switcherooctl launch env PX4_UXRCE_DDS_NS=px4_$instance_id VERBOSE_SIM=1 PX4_SYS_AUTOSTART=$PX4_SYS_AUTOSTART PX4_SIM_MODEL=$PX4_MODEL PX4_GZ_MODEL_POSE=\"$pose\" PX4_GZ_WORLD=\"$WORLD\" $headless_flag make px4_sitl $PX4_MODEL"
        else
            echo "Using standard GPU configuration"
            cmd="PX4_UXRCE_DDS_NS=px4_$instance_id VERBOSE_SIM=1 PX4_SYS_AUTOSTART=$PX4_SYS_AUTOSTART PX4_SIM_MODEL=$PX4_MODEL PX4_GZ_MODEL_POSE=\"$pose\" PX4_GZ_WORLD=\"$WORLD\" $headless_flag make px4_sitl $PX4_MODEL"
        fi
        
        # Use EKF reset script if available
        if [ -f "$SWARMZ4_PATH/launch_scripts/px4_reset_ekf.sh" ]; then
            launch_terminal "px4_$instance_id" "$SWARMZ4_PATH/launch_scripts/px4_reset_ekf.sh \"$cmd\"" "$HEADLESS_LEVEL"
        else
            launch_terminal "px4_$instance_id" "$cmd" "$HEADLESS_LEVEL"
        fi
        
        echo "Waiting 15 seconds for first PX4 instance and Gazebo to initialize..."
        sleep 15
    else
        echo "Launching PX4 instance $instance_id at position ($x_pos, $y_pos, 0) with yaw $yaw"
        
        # Build command with environment variables
        local cmd="TIMEOUT=10 VERBOSE_SIM=1 PX4_UXRCE_DDS_NS=px4_$instance_id PX4_SYS_AUTOSTART=$PX4_SYS_AUTOSTART PX4_GZ_MODEL_POSE=\"$pose\" PX4_SIM_MODEL=$PX4_MODEL PX4_GZ_WORLD=\"$WORLD\" $headless_flag $SWARMZ4_PATH/PX4-Autopilot/build/px4_sitl_default/bin/px4 -i $instance_id"
        
        # Use EKF reset script if available
        if [ -f "$SWARMZ4_PATH/launch_scripts/px4_reset_ekf.sh" ]; then
            launch_terminal "px4_$instance_id" "$SWARMZ4_PATH/launch_scripts/px4_reset_ekf.sh \"$cmd\"" "$HEADLESS_LEVEL"
        else
            launch_terminal "px4_$instance_id" "$cmd" "$HEADLESS_LEVEL"
        fi
        
        echo "Waiting for PX4 instance $instance_id to initialize..."
        sleep 1
    fi
}

# Launch a team of drones with fixed positions for debugging
launch_team() {
    local team_num=$1
    local min_x=$2
    local max_x=$3
    local min_y=$4
    local max_y=$5
    local yaw=$6
    
    # For debug mode, use fixed positions close to each other
    # Each team starts at its fixed position
    local start_x=$min_x
    local start_y=$min_y
    
    echo "DEBUGGING MODE: Spawning Team $team_num at fixed position ($start_x, $start_y)"
    
    # Launch drones with closer spacing (1m apart instead of 2m)
    for ((i=0; i<NUM_DRONES_PER_TEAM; i++)); do
        local drone_id=$((team_num == 1 ? i : NUM_DRONES_PER_TEAM + i))
        local drone_y=$((start_y + i))  # 1-meter spacing for easier debugging
        
        echo "Launching drone $drone_id at ($start_x, $drone_y)"
        launch_px4_instance "$drone_id" "$start_x" "$drone_y" "$yaw"
        
        echo "Drone $drone_id launched."
    done
}

#===========================================================================
# 6. MAIN EXECUTION
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
elif [ "$HEADLESS_LEVEL" -eq 1 ]; then
    echo "Started MicroXRCEAgent in background."
else
    echo "Started MicroXRCEAgent."
fi

# Step 3: Launch drone teams (with fixed positions for debugging)
echo "Step 3: Launching drone teams..."
cd $SWARMZ4_PATH/PX4-Autopilot || { echo "PX4-Autopilot directory not found!"; exit 1; }
# Team 1 at x=0, y=0 with 1m spacing
launch_team 1 0 0 0 0 0
# Team 2 at x=5, y=0 with 1m spacing, facing team 1
launch_team 2 5 0 0 0 3.14159

# Step 4: Launch QGroundControl
echo "Step 4: Launching QGroundControl..."
cd $SWARMZ4_PATH/launch_scripts || { echo "launch_scripts directory not found!"; exit 1; }
if [ "$HEADLESS_LEVEL" -ge 1 ]; then
    # Full headless mode uses xvfb-run with minimal logging
    xvfb-run -a $SWARMZ4_PATH/launch_scripts/QGroundControl.AppImage > /dev/null 2>&1 &
    echo "Started QGroundControl with minimal logging"
else
    # GUI mode
    $SWARMZ4_PATH/launch_scripts/QGroundControl.AppImage > /dev/null 2>&1 &
fi

# Step 5: Wait for user to exit
echo "Step 5: All processes started."
if [ "$HEADLESS_LEVEL" -eq 2 ]; then
    echo "Running in headless mode with output suppressed."
elif [ "$HEADLESS_LEVEL" -eq 1 ]; then
    echo "Running in Gazebo headless mode."
else
    echo "Running in GUI mode."
fi
echo "Press Ctrl+C to terminate all processes."

trap "cleanup; exit 0" INT
wait