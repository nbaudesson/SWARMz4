#! /bin/bash

# Source check_swarmz_path.sh to ensure SWARMZ4_PATH is set correctly
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/check_swarmz_path.sh"
export SWARMZ4_PATH

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

# Main function for PX4
if ! check_px4_installed; then
    install_px4
else
    echo "PX4-Autopilot is already installed, skipping."
fi


