#! /bin/bash

# Source check_swarmz_path.sh to ensure SWARMZ4_PATH is set correctly
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/check_swarmz_path.sh"
export SWARMZ4_PATH

# Make all shell scripts executable
echo "Making all shell scripts executable..."
find "$SWARMZ4_PATH/install_scripts" -name "*.sh" -exec chmod +x {} \;
find "$SWARMZ4_PATH/launch_scripts" -name "*.sh" -exec chmod +x {} \;
echo "✓ Made all shell scripts executable"

# Function to run a script interactively with password support
run_installation() {
    local script_name=$1
    local script_path="$SWARMZ4_PATH/install_scripts/$script_name"
    
    echo "======================================================="
    echo "Starting installation: $script_name"
    echo "======================================================="
    
    # Make script executable
    chmod +x "$script_path"
    
    # Run the script interactively (allowing password prompts)
    bash "$script_path"
    local status=$?
    
    if [ $status -eq 0 ]; then
        echo "✓ $script_name completed successfully"
        return 0
    else
        echo "✗ $script_name failed with status $status"
        return 1
    fi
}

echo "Starting sequential installation of components..."
echo "You may be prompted for your password during installation."

# Install Requirements
if ! run_installation "install_requirements.sh"; then
    echo "Error: Requirements installation failed"
    exit 1
fi

# Install ROS 2
if ! run_installation "install_ros2.sh"; then
    echo "Error: ROS 2 installation failed"
    exit 1
fi

# Install QGroundControl
if ! run_installation "install_QGround_Control.sh"; then
    echo "Error: QGroundControl installation failed"
    exit 1
fi

# Install Micro-XRCE
if ! run_installation "install_Micro-XRCE.sh"; then
    echo "Error: Micro-XRCE installation failed"
    exit 1
fi

# Install PX4
if ! run_installation "install_PX4.sh"; then
    echo "Error: PX4 installation failed"
    exit 1
fi

# Install PX4
if ! run_installation "install_custom_files.sh"; then
    echo "Error: custom files installation failed"
    exit 1
fi

echo "All installations completed successfully!"
echo "SWARMz4 setup is complete."
exit 0