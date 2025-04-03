#!/bin/bash

# Test script for GPU detection and switcherooctl usage
# This script demonstrates the GPU detection functionality and
# shows what commands would be used in different scenarios

# Function to check if there's a dedicated GPU that's not being used as the main one
has_secondary_gpu() {
    # Check if lspci is available
    if ! command -v lspci &> /dev/null; then
        echo "lspci not found, can't detect GPUs properly"
        return 1
    fi
    
    # Get GPU info
    gpu_info=$(lspci | grep -E "VGA|3D")
    
    # Check if we have more than one GPU
    gpu_count=$(echo "$gpu_info" | wc -l)
    if [ "$gpu_count" -lt 2 ]; then
        # Only one GPU found
        return 1
    fi
    
    # Check if we have a non-Intel GPU (likely dedicated)
    if echo "$gpu_info" | grep -v "Intel" | grep -E "NVIDIA|AMD|ATI" &> /dev/null; then
        # Check if the primary GPU is Intel (meaning dedicated GPU is secondary)
        if echo "$gpu_info" | head -1 | grep "Intel" &> /dev/null; then
            # We have a dedicated GPU that's not the primary one
            echo "Detected secondary dedicated GPU"
            return 0
        fi
    fi
    
    # No secondary dedicated GPU found
    return 1
}

# Output system GPU information
echo "====== GPU DETECTION TEST ======"
echo ""
echo "System GPU information:"
echo "------------------------"
if command -v lspci &> /dev/null; then
    lspci | grep -E "VGA|3D" | sed 's/^/  /'
else
    echo "  lspci not available - cannot list GPU devices"
    echo "  Please install pciutils package with: sudo apt-get install pciutils"
fi
echo ""

# Test GPU detection function
echo "Testing GPU detection:"
echo "---------------------"
if has_secondary_gpu; then
    USE_SWITCHEROO=true
    echo "✓ Secondary dedicated GPU detected!"
    echo "  → Will use switcherooctl for Gazebo"
else
    USE_SWITCHEROO=false
    echo "✗ No secondary dedicated GPU detected"
    echo "  → Will use standard GPU configuration"
fi
echo ""

# Show what command would be used
echo "Command that would be used:"
echo "-------------------------"
INSTANCE_ID=0
POSE="10,20,0,0,0,0"
PX4_MODEL="gz_x500_lidar_front"
PX4_SYS_AUTOSTART=4017
WORLD="swarmz_world_2"
HEADLESS_FLAG=""

if [ "$USE_SWITCHEROO" = true ]; then
    echo "Using switcherooctl command:"
    echo "  switcherooctl launch env PX4_UXRCE_DDS_NS=px4_$INSTANCE_ID VERBOSE_SIM=1 \\"
    echo "    PX4_SYS_AUTOSTART=$PX4_SYS_AUTOSTART PX4_SIM_MODEL=$PX4_MODEL \\"
    echo "    PX4_GZ_MODEL_POSE=\"$POSE\" PX4_GZ_WORLD=\"$WORLD\" $HEADLESS_FLAG \\"
    echo "    make px4_sitl $PX4_MODEL"
else
    echo "Using standard command:"
    echo "  PX4_UXRCE_DDS_NS=px4_$INSTANCE_ID VERBOSE_SIM=1 \\"
    echo "    PX4_SYS_AUTOSTART=$PX4_SYS_AUTOSTART PX4_SIM_MODEL=$PX4_MODEL \\"
    echo "    PX4_GZ_MODEL_POSE=\"$POSE\" PX4_GZ_WORLD=\"$WORLD\" $HEADLESS_FLAG \\"
    echo "    make px4_sitl $PX4_MODEL"
fi
echo ""

# Test switcherooctl availability if needed
if [ "$USE_SWITCHEROO" = true ]; then
    echo "Testing switcherooctl availability:"
    echo "--------------------------------"
    if command -v switcherooctl &> /dev/null; then
        echo "✓ switcherooctl is available"
        
        # Check if we can get info about GPUs with switcherooctl
        if switcherooctl list 2>/dev/null | grep -q "GPU"; then
            echo "✓ switcherooctl can access GPU information"
            echo "  GPU information from switcherooctl:"
            switcherooctl list | grep -A3 "GPU" | sed 's/^/  /'
        else
            echo "✗ switcherooctl cannot access GPU information"
            echo "  This may indicate a configuration issue with switcherooctl"
        fi
    else
        echo "✗ switcherooctl is NOT available!"
        echo "  You need to install the switcheroo-control package:"
        echo "  sudo apt-get install switcheroo-control"
    fi
fi

echo ""
echo "====== TEST COMPLETE ======"
