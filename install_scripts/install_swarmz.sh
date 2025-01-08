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

# Function to run each individual installation script
install_all() {
    echo "Starting installation of all components..."

    chmod +x "$SWARMZ4_PATH/install_scripts/install_ros2.sh" "$SWARMZ4_PATH/install_scripts/install_PX4.sh" "$SWARMZ4_PATH/install_scripts/install_Micro-XRCE.sh" "$SWARMZ4_PATH/install_scripts/install_QGround_Control.sh"

    echo "Installing ROS 2 Humble..."
    gnome-terminal --tab --title="ros2" -- sh -c "bash $SWARMZ4_PATH/install_scripts/install_ros2.sh; bash"

    echo "Installing PX4-Autopilot..."
    gnome-terminal --tab --title="PX4" -- sh -c "bash $SWARMZ4_PATH/install_scripts/install_PX4.sh; bash"

    echo "Installing Micro-XRCE-DDS-Agent..."
    gnome-terminal --tab --title="Micro-XRC" -- sh -c "bash $SWARMZ4_PATH/install_scripts/install_Micro-XRCE.sh; bash"

    echo "Installing QGroundControl..."
    gnome-terminal --tab --title="QGroundControl" -- sh -c "bash $SWARMZ4_PATH/install_scripts/install_QGround_Control.sh; bash"

    echo "All components launched successfully."
}

# Execute the main function
install_all