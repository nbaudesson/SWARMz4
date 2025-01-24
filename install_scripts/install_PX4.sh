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

# Function to check if custom_world is already installed
check_custom_world_installed() {
    if [ -f "$HOME/.simulation-gazebo/worlds/swarmz_world.sdf" ]; then
        echo "custom_world is already installed at $HOME/.simulation-gazebo/worlds/swarmz_world.sdf."
        return 0
    else
        echo "custom_world is not installed at $HOME/.simulation-gazebo/worlds/swarmz_world.sdf."
        return 1
    fi
}

copy_world() {
    echo "Copying custom world file to gazebo worlds directory."
    if [ ! -d "$HOME/.simulation-gazebo/worlds" ]; then
        mkdir -p "$HOME/.simulation-gazebo/worlds"
    fi
    cp $SWARMZ4_PATH/launch_scripts/swarmz_world.sdf "$HOME/.simulation-gazebo/worlds/swarmz_world.sdf"
}

# Main function for PX4
if ! check_px4_installed; then
    install_px4
else
    echo "PX4-Autopilot is already installed, skipping."
fi

# Copying the custom world file named swarmz_world.sdf
if ! check_custom_world_installed; then
    copy_world
else
    echo "custom world is already installed, skipping."
fi
