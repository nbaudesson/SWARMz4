#!/bin/bash

# Source check_swarmz_path.sh to ensure SWARMZ4_PATH is set correctly
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/check_swarmz_path.sh"
export SWARMZ4_PATH

# Function to check if QGroundControl is already installed
check_qgroundcontrol_installed() {
    if [ -f "$SWARMZ4_PATH/launch_scripts/QGroundControl.AppImage" ]; then
        echo "QGroundControl is already installed at $SWARMZ4_PATH."
        return 0
    else
        echo "QGroundControl is not installed at $SWARMZ4_PATH."
        return 1
    fi
}

# Function to install QGroundControl
install_qgroundcontrol() {
    echo "Installing QGroundControl..."

    sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y
    sudo apt install libfuse2 -y
    sudo apt install libxcb-xinerama0 libxkbcommon-x11-0 libxcb-cursor-dev -y
    # to run QGroundControl in a headless environment
    sudo apt-get install xvfb -y
    
    cd $SWARMZ4_PATH/launch_scripts || { echo "launch_scripts directory not found!"; exit 1; }
    wget https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl.AppImage -O QGroundControl.AppImage
    chmod +x QGroundControl.AppImage

    echo "QGroundControl installation completed."
}

# Main function for QGroundControl
if ! check_qgroundcontrol_installed; then
    install_qgroundcontrol
else
    echo "QGroundControl is already installed, skipping."
fi