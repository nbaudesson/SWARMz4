#!/bin/bash

# Locate the folder named 'swarmz4' starting from the current user's home directory.
SWARMZ4_PATH=$(find "$HOME" -type d -name "SWARMz4" 2>/dev/null)

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