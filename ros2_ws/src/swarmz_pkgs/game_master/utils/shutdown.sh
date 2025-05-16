#!/usr/bin/expect -f
log_user 1

# Fonction pour afficher l'usage
proc show_usage {} {
    puts "Usage: ./shutdown.sh <drone_id>"
    puts "       ./shutdown.sh list    ;# pour voir les drones disponibles"
    exit 1
}

# Si "list" est passé, on affiche les sessions tmux
if {[llength $argv] == 1 && [string match "list" [lindex $argv 0]]} {
    puts " Drones (sessions tmux) disponibles :"
    set sessions [exec tmux list-sessions]
    puts "$sessions"
    exit 0
}

# Vérifie qu’un ID de drone a été donné
if {$argc != 1} {
    show_usage
}


# Get the drone ID from the argument
set drone_id [lindex $argv 0]

# Check if a tmux session with the given drone ID exists
set session_check [catch {exec tmux has-session -t $drone_id} result]
if {$session_check != 0} {
    puts "Tmux session '$drone_id' not found."
    exit 1
}

# Send shutdown commands to the tmux session
puts "Sending shutdown commands to '$drone_id'..."

# Set the parameter for the offboard loss failsafe and reduce the timeout for triggering the disarm procedure.
# First switch to "Land mode" to brake the drone speed and stabilize it 
spawn tmux send-keys -t $drone_id "param set COM_OBL_RC_ACT 4" C-m 
spawn tmux send-keys -t $drone_id "param set COM_OF_LOSS_T 0.001" C-m
sleep 5

# Then change the mode to disarm to let the drone fall 
spawn tmux send-keys -t $drone_id "param set COM_OBL_RC_ACT 7" C-m
sleep 1

# Restore the parameter to its default configuration/value
spawn tmux send-keys -t $drone_id "param set COM_OBL_RC_ACT 0" C-m
spawn tmux send-keys -t $drone_id "param set COM_OF_LOSS_T 1.0" C-m
sleep 1 

# Stop all MAVLink interfaces
spawn tmux send-keys -t $drone_id "mavlink stop-all" C-m
sleep 1

# Confirm that the shutdown process was completed
puts "Drone '$drone_id' commanded to land and MAVLink stopped."
