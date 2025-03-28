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
HEADLESS_LEVEL=0 # 0=GUI, 1=Gazebo headless, 2=Full headless
# WORLD="default"
WORLD="swarmz_world"

# Arguments
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

# Remove log directory creation for headless mode
if [ "$HEADLESS_LEVEL" -eq 2 ]; then
  SUPPRESS_OUTPUT=true
  echo "Full headless mode activated. Output suppressed."
fi

# Terminal handling function that supports gnome-terminal or background processes
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

# Function to check if there's a dedicated GPU that's not being used as the main one
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
        
        # Use EKF reset script similar to launch_game.sh
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
        
        # Use EKF reset script similar to launch_game.sh
        if [ -f "$SWARMZ4_PATH/launch_scripts/px4_reset_ekf.sh" ]; then
            launch_terminal "px4_$instance_id" "$SWARMZ4_PATH/launch_scripts/px4_reset_ekf.sh \"$cmd\"" "$HEADLESS_LEVEL"
        else
            launch_terminal "px4_$instance_id" "$cmd" "$HEADLESS_LEVEL"
        fi
        
        echo "Waiting for PX4 instance $instance_id to initialize..."
        sleep 1
    fi
}

# Launch drones - modified for debugging with tighter spacing
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

# Use launch_terminal function for consistency with launch_game.sh
launch_terminal "MicroXRCEAgent" "MicroXRCEAgent udp4 -p 8888 -v 4" "$HEADLESS_LEVEL"
if [ "$HEADLESS_LEVEL" -eq 2 ]; then
    echo "Started MicroXRCEAgent with output suppressed"
elif [ "$HEADLESS_LEVEL" -eq 1 ]; then
    echo "Started MicroXRCEAgent in background."
else
    echo "Started MicroXRCEAgent."
fi

# Launch teams with closer fixed positions for debugging
launch_team 1 0 0 0 0 0                # Team 1 at x=0, y=0 with 1m spacing
launch_team 2 5 0 0 0 3.14159          # Team 2 at x=5, y=0 with 1m spacing, facing team 1

### Launch QGroundControl ###
echo "Launching QGroundControl..."
cd $SWARMZ4_PATH/launch_scripts || { echo "launch_scripts directory not found!"; exit 1; }
if [ "$HEADLESS_LEVEL" -ge 1 ]; then
    # Full headless mode uses xvfb-run with minimal logging
    xvfb-run -a $SWARMZ4_PATH/launch_scripts/QGroundControl.AppImage > /dev/null 2>&1 &
    echo "Started QGroundControl with minimal logging"
else
    # GUI mode
    $SWARMZ4_PATH/launch_scripts/QGroundControl.AppImage > /dev/null 2>&1 &
fi

# Wait for user to exit
echo "Press Ctrl+C to terminate all processes."
trap "cleanup; exit 0" INT

if [ "$HEADLESS_LEVEL" -eq 2 ]; then
    echo "All processes started in headless mode with output suppressed."
elif [ "$HEADLESS_LEVEL" -eq 1 ]; then
    echo "All processes started in Gazebo headless mode."
else
    echo "All processes started in GUI mode."
    
fi
echo "Press Ctrl+C to terminate all processes."
wait