#!/bin/bash

# Check if SWARMZ4_PATH is already set in the environment
if [ -n "$SWARMZ4_PATH" ]; then
    echo "Using pre-existing SWARMZ4_PATH: $SWARMZ4_PATH"
else
    echo "SWARMZ4_PATH is not set. Searching for 'swarmz4' directory in $HOME..."

    # Locate the folder named 'swarmz4' starting from the current user's home directory
    SWARMZ4_PATH=$(find "$HOME" -maxdepth 4 -type d -name "SWARMz4" 2>/dev/null)

    # Check if the folder was found
    if [ -z "$SWARMZ4_PATH" ]; then
        echo "Error: 'swarmz4' directory not found in $HOME!"
        sleep 5
        exit 1
    else
        echo "Found 'swarmz4' directory at: $SWARMZ4_PATH"

        # Optionally export SWARMZ4_PATH so it persists for subsequent scripts
        export SWARMZ4_PATH
    fi
fi

# Function to check if PX4-Autopilot is already installed
check_px4_installed() {
    if [ -d "$SWARMZ4_PATH/PX4-Autopilot" ]; then
        echo "PX4-Autopilot is already installed at $SWARMZ4_PATH."
        return 0
    else
        echo "PX4-Autopilot is not installed at $SWARMZ4_PATH."
        return 1
    fi
}

# Function to install PX4-Autopilot
install_px4() {
    echo "Installing PX4-Autopilot in $SWARMZ4_PATH" 

    cd $SWARMZ4_PATH
    git clone --recursive https://github.com/PX4/PX4-Autopilot.git
    cd $SWARMZ4_PATH/PX4-Autopilot || { echo "Failed to access PX4 directory"; exit 1; }
    bash ./Tools/setup/ubuntu.sh
    make px4_sitl

    echo "PX4-Autopilot installation completed."
}

# Function to check if standard PX4 worlds are installed
check_px4_worlds_installed() {
    if [ -f "$HOME/.simulation-gazebo/worlds/empty.sdf" ] && \
       [ -f "$HOME/.simulation-gazebo/worlds/default.sdf" ]; then
        echo "PX4 standard worlds are already installed."
        return 0
    else
        echo "PX4 standard worlds not found, need to copy from PX4."
        return 1
    fi
}

# Function to check if standard PX4 models are installed
check_px4_models_installed() {
    if [ -d "$HOME/.simulation-gazebo/models/x500" ] && \
       [ -d "$HOME/.simulation-gazebo/models/x500_lidar_front" ]; then
        echo "PX4 standard models are already installed."
        return 0
    else
        echo "PX4 standard models not found, need to copy from PX4."
        return 1
    fi
}

# Function to check if custom world is already installed
check_custom_world_installed() {
    if [ -f "$HOME/.simulation-gazebo/worlds/swarmz_world.sdf" ]; then
        echo "Custom world is already installed."
        return 0
    else
        echo "Custom world not found, need to install."
        return 1
    fi
}

standalone_copy_world_and_model() {
    echo "Setting up simulation directories..."
    
    # Create directories if they don't exist
    mkdir -p "$HOME/.simulation-gazebo/worlds"
    mkdir -p "$HOME/.simulation-gazebo/models"
    
    # Copy PX4 worlds if needed
    if ! check_px4_worlds_installed; then
        echo "Copying PX4 standard worlds..."
        cp -r "$SWARMZ4_PATH/PX4-Autopilot/Tools/simulation/gz/worlds/"* "$HOME/.simulation-gazebo/worlds/"
    fi
    
    # Copy PX4 models if needed
    if ! check_px4_models_installed; then
        echo "Copying PX4 standard models..."
        cp -r "$SWARMZ4_PATH/PX4-Autopilot/Tools/simulation/gz/models/"* "$HOME/.simulation-gazebo/models/"
    fi
    
    # Copy custom world if needed
    if ! check_custom_world_installed; then
        echo "Copying custom world..."
        cp "$SWARMZ4_PATH/launch_scripts/swarmz_world.sdf" "$HOME/.simulation-gazebo/worlds/swarmz_world.sdf"
    fi
    
    echo "Simulation files setup complete."
}

copy_world_and_model() {
    echo "Setting up simulation directories..."
    
    # Copy custom world if needed
    echo "Copying custom world..."
    cp "$SWARMZ4_PATH/launch_scripts/swarmz_world.sdf" "$SWARMZ4_PATH/PX4-Autopilot/Tools/simulation/gz/worlds/swarmz_world.sdf"
    cp "$SWARMZ4_PATH/launch_scripts/swarmz_world_2.sdf" "$SWARMZ4_PATH/PX4-Autopilot/Tools/simulation/gz/worlds/swarmz_world_2.sdf"
    
    echo "Simulation files setup complete."
}

# Main function for PX4
if ! check_px4_installed; then
    install_px4
else
    echo "PX4-Autopilot is already installed, skipping."
fi

# Setup simulation files
copy_world_and_model
