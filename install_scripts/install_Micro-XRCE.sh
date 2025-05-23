#!/bin/bash

# Source check_swarmz_path.sh to ensure SWARMZ4_PATH is set correctly
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/check_swarmz_path.sh"
export SWARMZ4_PATH

# Function to check if Micro-XRCE-DDS-Agent is already installed
check_microxrce_installed() {
    if [ -d "$SWARMZ4_PATH/Micro-XRCE-DDS-Agent" ]; then
        echo "Micro-XRCE-DDS-Agent is already installed at $SWARMZ4_PATH."
        return 0
    else
        echo "Micro-XRCE-DDS-Agent is not installed at $SWARMZ4_PATH."
        return 1
    fi
}

# Function to install Micro-XRCE-DDS-Agent
install_microxrce() {
    echo "Installing Micro-XRCE-DDS-Agent..."
    
    sudo apt-get install build-essential -y

    cd $SWARMZ4_PATH
    git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
    cd Micro-XRCE-DDS-Agent || { echo "Failed to access Micro-XRCE-DDS-Agent directory"; exit 1; }
    mkdir build
    cd build || { echo "Failed to access build directory"; exit 1; }
    cmake ..
    make -j 4
    sudo make install
    sudo ldconfig /usr/local/lib/

    echo "Micro-XRCE-DDS-Agent installation completed."
}

# Main function for Micro-XRCE
if ! check_microxrce_installed; then
    install_microxrce
else
    echo "Micro-XRCE-DDS-Agent is already installed, skipping."
fi
