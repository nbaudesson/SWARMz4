#! /bin/sh

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

# Function to check if a process completed successfully
check_completion() {
    local logfile=$1
    if [ -f "$logfile" ]; then
        if grep -q "Installation completed successfully" "$logfile"; then
            return 0
        fi
    fi
    return 1
}

# Function to wait for required installations
wait_for_prerequisites() {
    local ros2_log="/tmp/ros2_install.log"
    local qgc_log="/tmp/qgc_install.log"
    local timeout=3600  # 1 hour timeout
    local start_time=$(date +%s)
    
    echo "Waiting for ROS 2 and QGroundControl installations to complete..."
    
    while true; do
        if check_completion "$ros2_log" && check_completion "$qgc_log"; then
            echo "Prerequisites installed successfully"
            return 0
        fi
        
        # Check timeout
        local current_time=$(date +%s)
        local elapsed=$((current_time - start_time))
        if [ $elapsed -gt $timeout ]; then
            echo "Timeout waiting for prerequisites"
            return 1
        fi
        
        sleep 10
    done
}

# Function to run installations in the correct sequence
install_all() {
    echo "Starting installation of components..."
    
    # Make all scripts executable
    chmod +x "$SWARMZ4_PATH/install_scripts/"*.sh
    
    # Start ROS 2 installation
    echo "Installing ROS 2 Humble..."
    gnome-terminal --tab --title="ros2" -- sh -c "bash $SWARMZ4_PATH/install_scripts/install_ros2.sh > /tmp/ros2_install.log 2>&1; echo 'Installation completed successfully' >> /tmp/ros2_install.log"
    
    # Start QGroundControl installation
    echo "Installing QGroundControl..."
    gnome-terminal --tab --title="QGroundControl" -- sh -c "bash $SWARMZ4_PATH/install_scripts/install_QGround_Control.sh > /tmp/qgc_install.log 2>&1; echo 'Installation completed successfully' >> /tmp/qgc_install.log"
    
    # Start Micro-XRCE in parallel (doesn't need to wait)
    echo "Installing Micro-XRCE-DDS-Agent..."
    gnome-terminal --tab --title="Micro-XRC" -- sh -c "bash $SWARMZ4_PATH/install_scripts/install_Micro-XRCE.sh"
    
    # Wait for ROS 2 and QGC to complete
    if wait_for_prerequisites; then
        # Start PX4 installation only after prerequisites are done
        echo "Installing PX4-Autopilot..."
        gnome-terminal --tab --title="PX4" -- sh -c "bash $SWARMZ4_PATH/install_scripts/install_PX4.sh"
    else
        echo "Error: Prerequisites installation failed or timed out"
        exit 1
    fi
}

# Execute the main function
install_all