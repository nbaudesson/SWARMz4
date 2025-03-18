#!/bin/bash

# SWARMz4 Game Launcher using tmux
# ===============================
#
# Description:
#   This script launches a multi-drone simulation environment for drone combat games
#   using tmux to organize all processes in a session with multiple windows.
#   It handles spawning two teams of 5 drones each at fixed positions on opposite sides
#   of the field.
#
# Usage:
#   ./launch_game_tmux.sh [HEADLESS_LEVEL]
#
# Arguments:
#   HEADLESS_LEVEL    : Headless operation level (default: 0)
#                        0 = Full GUI mode (everything has GUI)
#                        1 = Partial headless (Gazebo headless, rest with GUI)
#                        2 = Full headless (all components headless, minimal logs)
#

# Check for tmux installation
if ! command -v tmux &> /dev/null; then
    echo "Error: tmux is not installed. Please install it first."
    echo "       You can install it with: sudo apt-get install tmux"
    exit 1
fi

# Default headless level
HEADLESS_LEVEL=0

# Parse arguments
if [ -n "$1" ]; then
  HEADLESS_LEVEL=$1
  if ! [[ "$HEADLESS_LEVEL" =~ ^[0-2]$ ]]; then
    echo "Error: HEADLESS_LEVEL must be 0, 1, or 2"
    echo "Usage: ./launch_game_tmux.sh [HEADLESS_LEVEL]"
    exit 1
  fi
fi

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

# Check for required files
if [ ! -f "$SWARMZ4_PATH/launch_scripts/QGroundControl.AppImage" ]; then
    echo "Warning: QGroundControl.AppImage not found in launch_scripts directory."
    echo "You may need to download it from: https://docs.qgroundcontrol.com/master/en/getting_started/download_and_install.html"
    echo "and place it in $SWARMZ4_PATH/launch_scripts/"
    echo "The script will continue but QGroundControl might not work."
fi

# Kill any existing tmux session named 'swarmz'
if tmux has-session -t swarmz 2>/dev/null; then
    echo "Killing existing tmux session 'swarmz'..."
    tmux kill-session -t swarmz
fi

# Kill any running processes that might conflict
echo "Cleaning up running processes..."
pkill -f "px4|MicroXRCEAgent|gz sim|parameter_bridge|QGroundControl" || true

# Generate spawn position file
SPAWN_POSITION_FILE="$SWARMZ4_PATH/ros2_ws/src/px4_pkgs/px4_controllers/offboard_control_py/config/spawn_position.yaml"
echo "Creating spawn position file at: $SPAWN_POSITION_FILE"

# Initialize the spawn position YAML
cat > "$SPAWN_POSITION_FILE" << EOL
# Drone spawn positions
# Format: team_id:{drone_id:{x, y, yaw}, ...}
{
  "1":{
    "1":{
      x: 30,
      y: 50,
      yaw: 90},
    "2":{
      x: 32,
      y: 50,
      yaw: 90},
    "3":{
      x: 34,
      y: 50,
      yaw: 90},
    "4":{
      x: 36,
      y: 50,
      yaw: 90},
    "5":{
      x: 38,
      y: 50,
      yaw: 90},
  },
  "2":{
    "6":{
      x: 30,
      y: 450,
      yaw: 270},
    "7":{
      x: 32,
      y: 450,
      yaw: 270},
    "8":{
      x: 34,
      y: 450,
      yaw: 270},
    "9":{
      x: 36,
      y: 450,
      yaw: 270},
    "10":{
      x: 38,
      y: 450,
      yaw: 270},
  },
}
EOL

# Create a temporary tmux config file that includes the headless settings
TEMP_TMUX_CONFIG="/tmp/swarmz_tmux_${HEADLESS_LEVEL}.conf"
cat "$SWARMZ4_PATH/launch_scripts/swarmz_tmux.conf" > $TEMP_TMUX_CONFIG

# Make sure to export the HEADLESS_LEVEL to the tmux environment
export HEADLESS_LEVEL="$HEADLESS_LEVEL"

# Start tmux session
echo "Starting tmux session with SWARMz4 environment..."
echo "Headless level: $HEADLESS_LEVEL"
tmux -f "$TEMP_TMUX_CONFIG" start-server

# Create a simple script to set the environment variable within tmux
TMP_ENV_SCRIPT="/tmp/tmux_env_setter_${HEADLESS_LEVEL}.sh"
cat > "$TMP_ENV_SCRIPT" << EOF
#!/bin/bash
tmux set-environment -g HEADLESS_LEVEL "$HEADLESS_LEVEL"
EOF
chmod +x "$TMP_ENV_SCRIPT"

# Execute the environment setter before loading the config
tmux new-session -d -s tmp_env_setter "$TMP_ENV_SCRIPT"
sleep 1
tmux kill-session -t tmp_env_setter

# Now source the config file
tmux -f "$TEMP_TMUX_CONFIG" source-file "$TEMP_TMUX_CONFIG"

# Attach to the session
echo "Attaching to tmux session..."
tmux attach-session -t swarmz

# If we get here, the tmux session was detached
echo "Tmux session detached. Use 'tmux attach-session -t swarmz' to reconnect."
echo "To terminate the session, use 'tmux kill-session -t swarmz'."

# Clean up temporary files
rm -f $TEMP_TMUX_CONFIG
rm -f $TMP_ENV_SCRIPT
