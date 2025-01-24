#! /bin/sh

# Locate the folder named 'swarmz4' starting from the /home directory.
SWARMZ4_PATH=$(find "$HOME" -type d -name "swarmz4" 2>/dev/null)
export GZ_SIM_RESOURCE_PATH=$SWARMZ4_PATH/PX4-Autopilot/Tools/simulation/gz