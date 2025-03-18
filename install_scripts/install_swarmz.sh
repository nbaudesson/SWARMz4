#! /bin/bash

# Check if SWARMZ4_PATH is already set and validate it
if [ -n "$SWARMZ4_PATH" ]; then
    # Check if the path contains spaces or newlines (indicating multiple paths)
    if [[ "$SWARMZ4_PATH" == *" "* || "$SWARMZ4_PATH" == *$'\n'* ]]; then
        echo "Warning: SWARMZ4_PATH contains spaces or line returns. It may be invalid."
        echo "Resetting SWARMZ4_PATH..."
        unset SWARMZ4_PATH
    else
        echo "Using pre-existing SWARMZ4_PATH: $SWARMZ4_PATH"
    fi
fi

# If SWARMZ4_PATH is not set or was reset, determine it from script location
if [ -z "$SWARMZ4_PATH" ]; then
    # Find the directory where this script is located and go up 1 level to reach SWARMZ4 root
    SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    SWARMZ4_PATH="$(dirname "$SCRIPT_DIR")"
    
    echo "Set SWARMZ4_PATH to: $SWARMZ4_PATH"
    
    # Export SWARMZ4_PATH so it persists for subsequent scripts
    export SWARMZ4_PATH
    
    # Update or add SWARMZ4_PATH to ~/.bashrc for persistence
    if grep -q "export SWARMZ4_PATH=" ~/.bashrc; then
        sed -i "s|export SWARMZ4_PATH=.*|export SWARMZ4_PATH=\"$SWARMZ4_PATH\"|g" ~/.bashrc
        echo "Replacing ~/.bashrc SWARMZ4_PATH with new SWARMZ4_PATH"
    else
        echo "export SWARMZ4_PATH=\"$SWARMZ4_PATH\"" >> ~/.bashrc
        echo "Updated ~/.bashrc with SWARMZ4_PATH"
    fi
fi

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

echo "All installations completed successfully!"
echo "SWARMz4 setup is complete."
exit 0