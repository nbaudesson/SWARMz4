#! /bin/bash
# Source check_swarmz_path.sh to ensure SWARMZ4_PATH is set correctly
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/check_swarmz_path.sh"
export SWARMZ4_PATH

# Define the path to check and add
gz_maritime_path=$SWARMZ4_PATH/ros2_ws/install/gazebo_maritime/lib
# Check if the path is already in LD_LIBRARY_PATH
if [[ ":$LD_LIBRARY_PATH:" != *":$gz_maritime_path:"* ]]; then
    # Add the path if it's not already in the variable
    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$gz_maritime_path
    if grep -q "export LD_LIBRARY_PATH=" ~/.bashrc; then
        sed -i "s|export LD_LIBRARY_PATH=.*|export LD_LIBRARY_PATH=\"$LD_LIBRARY_PATH\"|g" ~/.bashrc
        echo "Replacing ~/.bashrc LD_LIBRARY_PATH with new LD_LIBRARY_PATH"
    else
        echo "export LD_LIBRARY_PATH=\"$LD_LIBRARY_PATH\"" >> ~/.bashrc
        echo "Updated ~/.bashrc with LD_LIBRARY_PATH"
    fi
else 
    echo "Path is already in LD_LIBRARY_PATH."
fi


# Check if GZ_SIM_SYSTEM_PLUGIN_PATH is already set and validate it
if [ -n "$GZ_SIM_SYSTEM_PLUGIN_PATH" ]; then
    if [[ "$GZ_SIM_SYSTEM_PLUGIN_PATH" == *" "* || "$GZ_SIM_SYSTEM_PLUGIN_PATH" == *$'\n'* || "$GZ_SIM_SYSTEM_PLUGIN_PATH" == *\"* ]]; then
        echo "Warning: GZ_SIM_SYSTEM_PLUGIN_PATH contains spaces, line returns, or quotes. It may be invalid."
        echo "Resetting GZ_SIM_SYSTEM_PLUGIN_PATH..."
        unset GZ_SIM_SYSTEM_PLUGIN_PATH
    else
        echo "Using pre-existing GZ_SIM_SYSTEM_PLUGIN_PATH: $GZ_SIM_SYSTEM_PLUGIN_PATH"
    fi
fi

if [ -z "$GZ_SIM_SYSTEM_PLUGIN_PATH" ]; then
    export GZ_SIM_SYSTEM_PLUGIN_PATH=":$gz_maritime_path"

    if grep -q "export GZ_SIM_SYSTEM_PLUGIN_PATH=" ~/.bashrc; then
        sed -i "s|export GZ_SIM_SYSTEM_PLUGIN_PATH=.*|export GZ_SIM_SYSTEM_PLUGIN_PATH=\":$gz_maritime_path\"|g" ~/.bashrc
        echo "Replacing ~/.bashrc GZ_SIM_SYSTEM_PLUGIN_PATH with new value"
    else
        echo "export GZ_SIM_SYSTEM_PLUGIN_PATH=\":$gz_maritime_path\"" >> ~/.bashrc
        echo "Updated ~/.bashrc with GZ_SIM_SYSTEM_PLUGIN_PATH"
    fi
fi

source ~/.bashrc