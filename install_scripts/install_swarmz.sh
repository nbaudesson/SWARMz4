#! /bin/sh

# Check if SWARMZ4_PATH is already set in the environment
if [ -n "$SWARMZ4_PATH" ]; then
    echo "Using pre-existing SWARMZ4_PATH: $SWARMZ4_PATH"
else
    echo "SWARMZ4_PATH is not set. Searching for 'swarmz4' directory in $HOME..."

    # Locate the folder named 'swarmz4' starting from the current user's home directory
    SWARMZ4_PATH=$(find "$HOME" -maxdepth 4 -type d -name "SWARMz4" 2>/dev/null | head -n 1)

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

# Function to run each individual installation script
install_all() {
    echo "Starting installation of all components..."

    chmod +x "$SWARMZ4_PATH/install_scripts/install_ros2.sh" "$SWARMZ4_PATH/install_scripts/install_PX4.sh" "$SWARMZ4_PATH/install_scripts/install_Micro-XRCE.sh" "$SWARMZ4_PATH/install_scripts/install_QGround_Control.sh"

    # Create a temporary directory to store completion status files
    TEMP_DIR=$(mktemp -d)
    
    # Start first three installations in background and have them create marker files when done
    echo "Installing ROS 2 Humble..."
    gnome-terminal --tab --title="ros2" -- sh -c "bash $SWARMZ4_PATH/install_scripts/install_ros2.sh; touch $TEMP_DIR/ros2_done; bash" &
    
    echo "Installing Micro-XRCE-DDS-Agent..."
    gnome-terminal --tab --title="Micro-XRC" -- sh -c "bash $SWARMZ4_PATH/install_scripts/install_Micro-XRCE.sh; touch $TEMP_DIR/micro_xrce_done; bash" &
    
    echo "Installing QGroundControl..."
    gnome-terminal --tab --title="QGroundControl" -- sh -c "bash $SWARMZ4_PATH/install_scripts/install_QGround_Control.sh; touch $TEMP_DIR/qgc_done; bash" &
    
    echo "Waiting for other installations to complete before starting PX4 installation..."
    
    # Wait for all components to finish installation by checking for marker files
    while [ ! -f "$TEMP_DIR/ros2_done" ] || [ ! -f "$TEMP_DIR/micro_xrce_done" ] || [ ! -f "$TEMP_DIR/qgc_done" ]; do
        echo "Waiting for other installations to complete..."
        sleep 10
    done
    
    echo "All prerequisite installations have completed. Starting PX4 installation..."
    gnome-terminal --tab --title="PX4" -- sh -c "bash $SWARMZ4_PATH/install_scripts/install_PX4.sh; bash"
    
    # Clean up temp directory
    rm -rf "$TEMP_DIR"

    echo "All components launched successfully."
}

# Execute the main function
install_all