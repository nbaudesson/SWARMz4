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

SPAWN_POSITION_FILE="$SWARMZ4_PATH/ros2_ws/src/px4_pkgs/px4_controllers/offboard_control_py/config/spawn_position.yaml"
# Getting the name of the terminal we are using to run the script (gnome-terminal, terminator, xterm)
TERMINAL_NAME=$(ps -p $PPID -o comm=) # get the name of the actual terminal (sourcing)

# Bridge and Process PIDs
CLOCK_BRIDGE_PID=""
LASER_BRIDGE_PIDS=()
BOAT_BRIDGE_PIDS=()
OFFBOARD_SERVER_PIDS=()
CANNON_SERVER_PIDS=()
 
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
 
# Set output suppression for headless mode
if [ "$HEADLESS_LEVEL" -eq 2 ]; then
  SUPPRESS_OUTPUT=true
  echo "Full headless mode activated. Output suppressed."
fi
 
#===========================================================================
# 4. UTILITY FUNCTIONS
#===========================================================================
 
# Random spawn of boats and creation of bridges for boat control # TODO: UPDATE world.py name and add sizeable field
warship_spawn() {
 
    # Generate world sdl file with random boat x,y position and 1.1 meters over the ground
    python3 $SWARMZ4_PATH/launch_scripts/create_world_water.py $start_x1 $start_y1 1.1 $start_x2 $start_y2 1.1 "game_world_water" "wam-v2" $FIELD_LENGTH $FIELD_WIDTH
}

# Terminal handling with headless mode support
launch_terminal() {
    local title="$1"
    local command="$2"
    local headless_level="$3"
    
    # Vérifie si la session tmux existe déjà
    if tmux has-session -t "$title" 2>/dev/null; then
        echo "Session tmux '$title' déjà existante. Suppression et relancement."
        tmux kill-session -t "$title"
    fi
 
    # Crée une nouvelle session tmux en mode détaché
    echo "Lancement de la session tmux '$title'"
    tmux new-session -d -s "$title" "$command"
 
    # Si headless_level est égal à 2, exécute en arrière-plan sans terminal
    if [ "$headless_level" -eq 2 ]; then
        echo "Starting process: $title (silent background)"
        eval "$command > /dev/null 2>&1 &"
        return
    fi
 
    # Vérification si le script bash est lancé dans un terminal graphique
    if [[ "$TERMINAL_NAME" != "gnome-terminal-" && "$TERMINAL_NAME" != "terminator" && "$TERMINAL_NAME" != "x-terminal-emul" && "$TERMINAL_NAME" != "xterm" ]]; then
        TERMINAL_NAME=$(ps -o comm= $(ps -o ppid= $(ps -o ppid= $$))) # Obtenir le nom du terminal parent
    fi
 
    # Si le terminal graphique est gnome-terminal
    if [[ "$TERMINAL_NAME" == "gnome-terminal-" ]]; then
        echo "Launching with gnome-terminal"
        gnome-terminal --tab --title="$title" -- bash -c "tmux attach -t $title; exec bash"
    # Si le terminal graphique est terminator
    # Launch command with terminator
    elif [[ "$TERMINAL_NAME" == "terminator" || "$TERMINAL_NAME" == "x-terminal-emul" ]]; then
        echo "No gnome-terminal found. Using terminator for $title."
 
        id="$ROS_DOMAIN_ID"
        ros2="/opt/ros/$ROS_DISTRO/setup.bash"
 
        # Lancement du terminal Terminator avec attachement à tmux
        terminator --new-tab -e "bash -c 'echo -ne \"\033]0;$title\007\"; \
            export ROS_DOMAIN_ID=$id; \
            source ~/.bashrc; \
            source $ros2; \
            source $SWARMZ4_PATH/ros2_ws/install/setup.bash; \
            export SWARMZ4_PATH=\"$SWARMZ4_PATH\"; \
            export GZ_SIM_SYSTEM_PLUGIN_PATH=\$GZ_SIM_SYSTEM_PLUGIN_PATH:\$SWARMZ4_PATH/ros2_ws/install/gazebo_maritime/lib; \
            export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:\$SWARMZ4_PATH/ros2_ws/install/gazebo_maritime/lib; \
            tmux attach -t $title || tmux new -s $title; \
            echo \"Press Enter to close\"; read'" &
    else
        # Si aucun terminal graphique trouvé, utiliser xterm ou exécuter en arrière-plan
        if [[ "$TERMINAL_NAME" == "xterm" ]]; then
            echo "No gnome-terminal found. Using xterm for $title."
            xterm -T "$title" -e "tmux attach -t $title; exec bash" &
        else
            # Si aucun terminal graphique n'est trouvé, lancer en arrière-plan
            echo "No gnome-terminal or xterm found. Running $title in background."
            eval "$command &"
        fi
    fi
}
 
# Process cleanup function
kill_processes() {
    echo "Cleaning up running processes..."
    local processes=("px4" "MicroXRCEAgent" "gz sim" "parameter_bridge" "QGroundControl" "offboard_control_px4" "cannon_server")
    
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

    # Clean up boat bridges if they exist
    if [ -n "$BOAT_BRIDGE_PIDS" ]; then
        echo "Stopping boat bridges..."
        for pid in "${BOAT_BRIDGE_PIDS[@]}"; do
            kill $pid 2>/dev/null
        done
        echo "Boat bridges stopped"
    fi
    
    # Clean up laser bridges if they exist
    if [ ${#LASER_BRIDGE_PIDS[@]} -gt 0 ]; then
        echo "Stopping laser bridges..."
        for pid in "${LASER_BRIDGE_PIDS[@]}"; do
            kill $pid 2>/dev/null
        done
        echo "Laser bridges stopped"
    fi
    
    # Clean up offboard servers if they exist
    if [ ${#OFFBOARD_SERVER_PIDS[@]} -gt 0 ]; then
        echo "Stopping offboard servers..."
        for pid_or_session in "${OFFBOARD_SERVER_PIDS[@]}"; do
            # Check if numeric (direct PID) or string (tmux session name)
            if [[ "$pid_or_session" =~ ^[0-9]+$ ]]; then
                kill $pid_or_session 2>/dev/null
            else
                # Kill tmux session if it exists
                tmux has-session -t "$pid_or_session" 2>/dev/null && tmux kill-session -t "$pid_or_session"
            fi
        done
        echo "Offboard servers stopped"
    fi
    
    # Clean up cannon servers if they exist
    if [ ${#CANNON_SERVER_PIDS[@]} -gt 0 ]; then
        echo "Stopping cannon servers..."
        for pid_or_session in "${CANNON_SERVER_PIDS[@]}"; do
            # Check if numeric (direct PID) or string (tmux session name)
            if [[ "$pid_or_session" =~ ^[0-9]+$ ]]; then
                kill $pid_or_session 2>/dev/null
            else
                # Kill tmux session if it exists
                tmux has-session -t "$pid_or_session" 2>/dev/null && tmux kill-session -t "$pid_or_session"
            fi
        done
        echo "Cannon servers stopped"
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
            sed -i "/    \"$drone_id\":{/,/^      yaw:/ {
                /      x: [0-9.-]*/ s/: .*/: $y,/
                /      y: [0-9.-]*/ s/: .*/: $x,/
                /      yaw: [0-9.-]*/ s/: .*/: 90},/
            }" "$file"
            ;;
    esac
}
 
# Generate random team position within bounds
generate_team_position() {
    #Generate random spawn position for boats following the following specifications:
    #Spawn position Team 1: x=[0,20%], y=[0,WIDTH]
    #Spawn position Team 2: x=[80%,100%], y=[0,WIDTH]
    #The drones will use the spawn position of the boat to spawn 10 meters in front of it and centered creating a T formation
    seed=$(date +%s%N) # nanosecond precision
    start_x1=$(( RANDOM % ((FIELD_LENGTH / 5 - 10) + 1) ))
    start_y1=$(( NUM_DRONES_PER_TEAM + RANDOM % (FIELD_WIDTH - 2 * NUM_DRONES_PER_TEAM + 1) ))
    start_x2=$(( (FIELD_LENGTH * 4 / 5 + 10) + RANDOM % (FIELD_LENGTH - (FIELD_LENGTH * 4 / 5 + 10) + 1) ))
    start_y2=$(( NUM_DRONES_PER_TEAM + RANDOM % (FIELD_WIDTH - 2 * NUM_DRONES_PER_TEAM + 1) ))

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
            cmd="switcherooctl launch env PX4_UXRCE_DDS_NS=px4_$instance_id VERBOSE_SIM=1 PX4_SYS_AUTOSTART=$PX4_SYS_AUTOSTART PX4_SIM_MODEL=$PX4_MODEL PX4_GZ_MODEL_POSE=\"$pose\" PX4_GZ_WORLD=game_world_water $headless_flag make px4_sitl $PX4_MODEL"
        else
            echo "Using standard GPU configuration"
            cmd="PX4_UXRCE_DDS_NS=px4_$instance_id VERBOSE_SIM=1 PX4_SYS_AUTOSTART=$PX4_SYS_AUTOSTART PX4_SIM_MODEL=$PX4_MODEL PX4_GZ_MODEL_POSE=\"$pose\" PX4_GZ_WORLD=game_world_water $headless_flag make px4_sitl $PX4_MODEL"
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
            PX4_GZ_WORLD=game_world_water \
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
    local start_x=$2
    local start_y=$3

    echo "Spawning Team $team_num at base position ($start_x, $start_y)"

    #Take the spawn position of the boat and place the drones 10 meters in front of it and centered in y
    if [ "$team_num" -eq 1 ]; then
        start_x=$((start_x + 10))
        start_y=$((start_y - 5))
    elif [ "$team_num" -eq 2 ]; then
        start_x=$((start_x - 10))
        start_y=$((start_y - 3)) # 5-2 meter spacing for team 2
    fi
    
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
launch_team_servers() {
    local team_num=$1

    echo "Starting offboard controllers for drones of Team $team_num"
    cd $SWARMZ4_PATH/ros2_ws || { echo "ros2_ws directory not found!"; exit 1; }

    local cmd="ros2 launch offboard_control_py offboard_team.launch.py team_id:=$team_num"

    # For headless level 2, we can capture PID directly
    if [ "$HEADLESS_LEVEL" -eq 2 ]; then
        echo "Starting offboard servers for team $team_num in background"
        eval "source install/setup.bash && $cmd > /dev/null 2>&1 &"
        OFFBOARD_SERVER_PIDS+=($!)
        echo "Started offboard controllers for team $team_num (PID: ${OFFBOARD_SERVER_PIDS[-1]})"
    else
        launch_terminal "offboard_servers_$team_num" "source install/setup.bash && $cmd" "1"
        # For tmux sessions, store the session name for potential cleanup
        OFFBOARD_SERVER_PIDS+=("offboard_servers_$team_num")
    fi
    
    echo "Waiting 5 seconds for offboard controllers to initialize..."
    sleep 5  # Increased wait time for better initialization
    
    cmd="ros2 run boat_driver cannon_server --ros-args -r __ns:=/flag_ship_$team_num"

    # For headless level 2, we can capture PID directly
    if [ "$HEADLESS_LEVEL" -eq 2 ]; then
        echo "Starting cannon server for team $team_num in background"
        eval "source install/setup.bash && $cmd > /dev/null 2>&1 &"
        CANNON_SERVER_PIDS+=($!)
        echo "Started cannon server for team $team_num (PID: ${CANNON_SERVER_PIDS[-1]})"
    else
        launch_terminal "cannon_server_$team_num" "source install/setup.bash && $cmd" "1"
        # For tmux sessions, store the session name for potential cleanup
        CANNON_SERVER_PIDS+=("cannon_server_$team_num")
    fi
    
    echo "Waiting 2 seconds for cannon to initialize..."
    sleep 2  # Increased wait time for better initialization
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
        local gz_topic="/world/game_world_water/model/${PX4_MODEL}_${i}/link/lidar_sensor_link/sensor/lidar/scan"
        local ros_topic="laser/scan"
        
        # Start the bridge with log level control and unique node name
        echo "Starting laser bridge for drone $i: $gz_topic -> $ros_topic"
        ros2 run ros_gz_bridge parameter_bridge "$gz_topic@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan" \
            --ros-args --log-level ros_gz_bridge:=$log_level -r "$gz_topic:=$ros_topic" -r __ns:=/px4_${i} -r __node:=drone_laser_bridge > /dev/null 2>&1 &
        
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

start_boat_bridge() {
    # bridges for thruster and cannon control

    # Set log level to ERROR to suppress INFO and WARN messages
    local log_level="ERROR"
    
    for i in $(seq 1 2); do
        echo "Starting left propeller bridge for flag ship $i"
        local gz_topic="/model/flag_ship_${i}/joint/left_engine_propeller_joint/cmd_thrust"
        local ros2_topic="left_propeller_cmd"
        ros2 run ros_gz_bridge parameter_bridge $gz_topic@std_msgs/msg/Float64]gz.msgs.Double \
            --ros-args --log-level ros_gz_bridge:=$log_level -r "$gz_topic:=$ros2_topic" -r __ns:=/flag_ship_${i} -r __node:=left_propeller > /dev/null 2>&1 &
        # Store the PID for cleanup
        BOAT_BRIDGE_PIDS+=($!)
        sleep 0.1

        echo "Starting right propeller bridge for flag ship $i"
        local gz_topic="/model/flag_ship_${i}/joint/right_engine_propeller_joint/cmd_thrust"
        local ros2_topic="right_propeller_cmd"
        ros2 run ros_gz_bridge parameter_bridge /model/flag_ship_${i}/joint/right_engine_propeller_joint/cmd_thrust@std_msgs/msg/Float64]gz.msgs.Double \
            --ros-args --log-level ros_gz_bridge:=$log_level -r "$gz_topic:=$ros2_topic" -r __ns:=/flag_ship_${i} -r __node:=right_propeller > /dev/null 2>&1 &
        BOAT_BRIDGE_PIDS+=($!)
        sleep 0.1

        echo "Starting cannon pitch bridge for flag ship $i"
        local gz_topic="/model/flag_ship_${i}/joint/j1/cmd_vel"
        local ros2_topic="cannon_pitch_cmd"
        ros2 run ros_gz_bridge parameter_bridge /model/flag_ship_${i}/joint/j1/cmd_vel@std_msgs/msg/Float64]gz.msgs.Double \
            --ros-args --log-level ros_gz_bridge:=$log_level -r "$gz_topic:=$ros2_topic" -r __ns:=/flag_ship_${i} -r __node:=cannon_pitch > /dev/null 2>&1 &
        BOAT_BRIDGE_PIDS+=($!)
        sleep 0.1

        echo "Starting cannon yaw bridge for flag ship $i"
        local gz_topic="/model/flag_ship_${i}/joint/j2/cmd_vel"
        local ros2_topic="cannon_yaw_cmd"
        ros2 run ros_gz_bridge parameter_bridge /model/flag_ship_${i}/joint/j2/cmd_vel@std_msgs/msg/Float64]gz.msgs.Double \
            --ros-args --log-level ros_gz_bridge:=$log_level -r "$gz_topic:=$ros2_topic" -r __ns:=/flag_ship_${i} -r __node:=cannon_yaw > /dev/null 2>&1 &
        BOAT_BRIDGE_PIDS+=($!)
        sleep 0.1

    done
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
echo "$SWARMZ4_PATH/Micro-XRCE-DDS-Agent"
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
#Generate random spawn positions
generate_team_position
#Spawn boats ans start thruster bridges
warship_spawn
 
cd $SWARMZ4_PATH/PX4-Autopilot || { echo "PX4-Autopilot directory not found!"; exit 1; }
 
launch_team 1 $start_x1 $start_y1
launch_team 2 $start_x2 $start_y2
cp $SPAWN_POSITION_FILE "$SWARMZ4_PATH/ros2_ws/install/offboard_control_py/share/offboard_control_py/config/spawn_position.yaml"
# Step 6: Start ROS 2 bridges
echo "Step 6: Starting ROS 2 bridges..."
 
# Start the clock bridge
start_clock_bridge
 
# Start the laser bridges
start_laser_bridges

# Start the boat bridges
start_boat_bridge

sleep 5

launch_team_servers 1
launch_team_servers 2

# Step 7: Wait for user to exit
echo "Step 7: All processes started."
if [ "$HEADLESS_LEVEL" -eq 2 ]; then
    echo "Running in headless mode. Press Ctrl+C to terminate all processes."
else
    echo "Press Ctrl+C to terminate all processes."
fi
 
trap "cleanup; exit 0" INT
wait