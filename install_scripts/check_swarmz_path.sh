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