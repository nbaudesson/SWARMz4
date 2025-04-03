#! /bin/bash
# Source check_swarmz_path.sh to ensure SWARMZ4_PATH is set correctly
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/check_swarmz_path.sh"
export SWARMZ4_PATH

# Function to copy all custom worlds and models from custom_files directory
copy_custom_files() {
    echo "Copying custom worlds and models from custom_files directory..."
    
    # Check if source directories exist
    if [ ! -d "$SWARMZ4_PATH/custom_files/worlds" ]; then
        echo "Custom worlds directory not found at $SWARMZ4_PATH/custom_files/worlds"
        mkdir -p "$SWARMZ4_PATH/custom_files/worlds"
    fi
    
    if [ ! -d "$SWARMZ4_PATH/custom_files/models" ]; then
        echo "Custom models directory not found at $SWARMZ4_PATH/custom_files/models"
        mkdir -p "$SWARMZ4_PATH/custom_files/models"
    fi
    
    # Copy custom worlds to PX4-Autopilot directory
    if [ -d "$SWARMZ4_PATH/PX4-Autopilot/Tools/simulation/gz/worlds" ]; then
        echo "Copying custom worlds to PX4-Autopilot..."
        cp -r "$SWARMZ4_PATH/custom_files/worlds/"* "$SWARMZ4_PATH/PX4-Autopilot/Tools/simulation/gz/worlds/" 2>/dev/null || echo "No custom worlds to copy to PX4-Autopilot"
    else
        echo "PX4-Autopilot worlds directory not found"
    fi
    
    # Copy custom models to PX4-Autopilot directory
    if [ -d "$SWARMZ4_PATH/PX4-Autopilot/Tools/simulation/gz/models" ]; then
        echo "Copying custom models to PX4-Autopilot..."
        cp -r "$SWARMZ4_PATH/custom_files/models/"* "$SWARMZ4_PATH/PX4-Autopilot/Tools/simulation/gz/models/" 2>/dev/null || echo "No custom models to copy to PX4-Autopilot"
    else
        echo "PX4-Autopilot models directory not found"
    fi
    
    # Copy custom worlds to ~/.simulation-gazebo directory if it exists
    if [ -d "$HOME/.simulation-gazebo/worlds" ]; then
        echo "Copying custom worlds to ~/.simulation-gazebo..."
        cp -r "$SWARMZ4_PATH/custom_files/worlds/"* "$HOME/.simulation-gazebo/worlds/" 2>/dev/null || echo "No custom worlds to copy to ~/.simulation-gazebo"
    fi
    
    # Copy custom models to ~/.simulation-gazebo directory if it exists
    if [ -d "$HOME/.simulation-gazebo/models" ]; then
        echo "Copying custom models to ~/.simulation-gazebo..."
        cp -r "$SWARMZ4_PATH/custom_files/models/"* "$HOME/.simulation-gazebo/models/" 2>/dev/null || echo "No custom models to copy to ~/.simulation-gazebo"
    fi
    
    echo "Custom files copied successfully."
}

# Execute the copy function
copy_custom_files