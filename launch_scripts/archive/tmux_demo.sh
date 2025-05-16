#!/bin/bash

# ========================================================================
# TMUX DEMO SCRIPT
# ========================================================================
#
# DESCRIPTION:
#   This script creates a demonstration tmux session with a 2x2 grid layout
#   to showcase tmux's capabilities for multi-pane terminal management.
#   It also configures xclip integration for clipboard support and demonstrates
#   various tmux features like window management and pane navigation.
#
# FEATURES:
#   - Creates a tmux session with a 2x2 pane grid layout
#   - Configures proper titles and color coding for each pane
#   - Sets up xclip integration for clipboard operations
#   - Demonstrates scrollback buffer and mouse support
#   - Provides visual instructions for tmux navigation
#
# USAGE:
#   ./tmux_demo.sh
#
# PREREQUISITES:
#   - tmux must be installed: sudo apt install tmux
#   - xclip must be installed for clipboard support: sudo apt install xclip
#
# NAVIGATION:
#   - Ctrl+b arrow : Move between panes using arrow keys
#   - Ctrl+b n/p   : Move to next/previous window
#   - Ctrl+b [     : Enter scroll mode (use Page Up/Down, q to exit)
#   - Ctrl+b z     : Zoom in/out of current pane
#   - Ctrl+b d     : Detach from session (use tmux attach -t tmux-demo to return)
#
# COPY/PASTE:
#   - Ctrl+b [     : Enter copy mode
#   - v           : Start selection (in vi mode)
#   - y           : Copy selection to clipboard
#   - Ctrl+b ]     : Paste from clipboard
#
# NOTES:
#   - This script automatically kills any existing tmux-demo session
#   - Color coding is used to distinguish between different panes
#   - The script demonstrates proper handling of terminal colors in tmux
#   - It uses separate scripts for each pane to avoid common pitfalls
#
# LEARNING CONCEPTS:
#   - How to create and configure tmux sessions
#   - How to manage multiple panes and windows
#   - How to ensure proper color support in tmux
#   - How to integrate system clipboard with tmux
#   - How to organize multiple terminal processes
#
# ========================================================================

# Turn off exit on error - makes the script more robust
# TIP: This makes the script continue even if some commands fail
set +e

# Kill any existing tmux-demo session
# TIP: Always clean up existing sessions with the same name to avoid conflicts
if tmux has-session -t tmux-demo 2>/dev/null; then
    echo "Killing existing tmux-demo session..."
    tmux kill-session -t tmux-demo
fi

echo "Creating new tmux demo session..."

# Create initial session - much simpler approach
# TIP: Always start with a detached session (-d) so you can configure it before attaching
# REUSE: Start your own sessions this way and give them a meaningful name
echo "Creating tmux session..."
tmux new-session -d -s tmux-demo -n "Main Window"
sleep 1  # Add a small delay to ensure session is created before modifying it

# Configure basic options only
# TIP: These are common settings you'll want in most tmux sessions
# REUSE: Copy these settings directly into your tmux scripts
tmux set-option -t tmux-demo mouse on                      # Enable mouse support
tmux set-option -t tmux-demo pane-border-status top        # Show pane titles at top
tmux set-option -t tmux-demo pane-border-format "[#{pane_index}] #{pane_title}"  # Format for pane titles

# Set up copy-paste with xclip for better system clipboard integration
# TIP: This is a critical usability feature for productivity
# REUSE: Copy this entire block to add clipboard support to your tmux sessions
echo "Enabling copy-paste functionality with xclip..."

if command -v xclip &> /dev/null; then
  # Set up tmux to use xclip for system clipboard interaction
  tmux set-window-option -t tmux-demo mode-keys vi  # Use vi keys in copy mode
  
  # Configure copy-paste with xclip
  # TIP: These bindings make tmux behave more like modern editors
  tmux bind-key -T copy-mode-vi v send-keys -X begin-selection              # Start selection with 'v'
  tmux bind-key -T copy-mode-vi y send-keys -X copy-pipe-and-cancel "xclip -in -selection clipboard"  # Copy with 'y'
  tmux bind-key -T copy-mode-vi MouseDragEnd1Pane send-keys -X copy-pipe-and-cancel "xclip -in -selection clipboard"  # Mouse copy
  
  # Make the right-click paste from clipboard
  tmux bind-key -T root MouseDown3Pane run -b "xclip -out -selection clipboard | tmux load-buffer - && tmux paste-buffer"
  
  # Also add Ctrl+b p as a paste shortcut
  tmux bind-key p run "xclip -out -selection clipboard | tmux load-buffer - ; tmux paste-buffer"
  
  echo "xclip integration configured successfully!"
else
  echo "ERROR: xclip not found but should be installed. Please check your system!"
  # Fallback to basic vi copy mode
  tmux set-window-option -t tmux-demo mode-keys vi
fi

# Create the 2x2 grid layout
# TIP: Creating precise layouts is one of tmux's strengths
# REUSE: This pattern creates a reliable 2x2 grid you can adapt for your scripts
echo "Creating panes..."

# Start by setting the initial layout
# TIP: Choosing the right starting layout makes further splits more predictable
tmux select-layout -t tmux-demo:0 even-vertical

# Split the panes in a specific order to get a 2x2 grid
# TIP: The order of splits is crucial to get the layout right
# REUSE: This pattern can be adapted for different layouts (e.g., 2x3, 3x3)
tmux split-window -h -t tmux-demo:0.0                      # Split 1st pane horizontally
tmux select-layout -t tmux-demo:0 main-vertical            # Ensure proper vertical layout
tmux split-window -v -t tmux-demo:0.0                      # Split left pane vertically
tmux split-window -v -t tmux-demo:0.1                      # Split right pane vertically

# Force a tiled layout to ensure equal distribution
# TIP: This ensures all panes get equal space
tmux select-layout -t tmux-demo:0 tiled

# Create a function to display the pane content
# This function takes the pane ID and title as arguments
show_pane_content() {
    local pane_id=$1
    local pane_title=$2
    
    # Set the pane title
    tmux select-pane -t "tmux-demo:0.$pane_id" -T "$pane_title"
    
    # Create a temporary script file for this pane
    local script_file="/tmp/tmux_pane_${pane_id}_content.sh"
    cat > "$script_file" << EOF
#!/bin/bash
# Disable command echoing
stty -echo
# Disable PS1 prompt
export PS1=""
# Clear the screen
clear
# Display the content
printf "\033[1;36mThis is the $pane_title pane ($pane_id)\033[0m\n"
printf "You are in: $pane_title\n"
printf "\n"
printf "\033[1;32mGreen Text\033[0m - Colors are working if you see this in green\n"
printf "\033[1;31mRed Text\033[0m - Colors are working if you see this in red\n"
printf "\033[1;34mBlue Text\033[0m - Colors are working if you see this in blue\n"
printf "\033[1;33mYellow Text\033[0m - Colors are working if you see this in yellow\n"
printf "\n"
printf "Try these commands:\n"
printf "%s\n" "- Ctrl+b [ to enter scroll mode (q to exit)"
printf "%s\n" "- Ctrl+b z to zoom this pane"
printf "%s\n" "- Ctrl+b q to show pane numbers"

# Keep the script running to maintain output
while true; do sleep 1000; done
EOF
    
    # Make the script executable
    chmod +x "$script_file"
    
    # Execute the script directly - no bootstrap needed
    tmux send-keys -t "tmux-demo:0.$pane_id" "exec $script_file" C-m
}

# Now set up each pane with the function
echo "Setting up pane content..."
show_pane_content 0 "Top Left"
show_pane_content 1 "Top Right"
show_pane_content 2 "Bottom Left"
show_pane_content 3 "Bottom Right"

# Apply the tiled layout one more time to ensure proper distribution
# TIP: This ensures all panes get equal space
tmux select-layout -t tmux-demo:0 tiled

# Create an instructions window
# TIP: Adding help screens makes complex tmux setups more user-friendly
# REUSE: Always include an instructions window in complex tmux configurations
echo "Creating instructions window..."
tmux new-window -t tmux-demo -n "Instructions"

# Create a temporary script file for instructions
cat > "/tmp/tmux_instructions_content.sh" << 'EOF'
#!/bin/bash
# Disable command echoing
stty -echo
# Disable PS1 prompt
export PS1=""
# Clear the screen
clear
# Display instructions
printf "\033[1;36mTMUX DEMO INSTRUCTIONS\033[0m\n"
echo "===================="
echo
printf "\033[1;33mNavigation:\033[0m\n"
echo
printf "%s\n" "- Ctrl+b arrow : Move between panes"
printf "%s\n" "- Ctrl+b n/p   : Next/previous window" 
printf "%s\n" "- Ctrl+b d     : Detach from session"
echo
printf "\033[1;33mCopy & Paste with xclip:\033[0m\n"
echo
printf "%s\n" "- Ctrl+b [     : Enter copy mode"
printf "%s\n" "- Space or v   : Start selection (in vi mode)"
printf "%s\n" "- y           : Copy selection to system clipboard"
printf "%s\n" "- Ctrl+b ]     : Paste from system clipboard"
echo
printf "%s\n" "Mouse copy-paste is also enabled - just select text with mouse"
printf "%s\n" "to copy to the system clipboard automatically."
echo
printf "%s\n" "For xclip to work, make sure the DISPLAY environment variable is set."
printf "%s\n" "This works in GUI environments but may not work in remote SSH sessions."
echo
printf "%s\n" "To reattach: tmux attach -t tmux-demo"

# Keep the script running
while true; do sleep 1000; done
EOF

# Make the instructions script executable
chmod +x "/tmp/tmux_instructions_content.sh"

# Create an instructions bootstrap script
cat > "/tmp/tmux_instructions_bootstrap.sh" << EOF
#!/bin/bash
# This bootstrap script immediately clears the screen and executes our content script
clear
exec /tmp/tmux_instructions_content.sh
EOF

chmod +x "/tmp/tmux_instructions_bootstrap.sh"

# Execute the instructions bootstrap script
tmux select-pane -t tmux-demo:1.0 -T "Instructions"
tmux send-keys -t tmux-demo:1.0 "clear" C-m
tmux send-keys -t tmux-demo:1.0 "exec /tmp/tmux_instructions_bootstrap.sh" C-m

# Return to the first window
# TIP: Always end setup by going to the most useful window
tmux select-window -t tmux-demo:0

# Attach to the session if not already in tmux
# TIP: This handles both direct execution and execution from within tmux
# REUSE: Always include this logic for automatic session attachment
if [ -z "$TMUX" ]; then
    echo "Starting tmux demo session..."
    tmux attach-session -t tmux-demo
else
    echo "You're already in a tmux session."
    echo "To access the demo, run: tmux switch-client -t tmux-demo"
    echo "Or from another terminal: tmux attach -t tmux-demo"
fi