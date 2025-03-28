#!/usr/bin/expect -f
# Set log_user to 1 if you want to see the commands and output
# Set to 0 to hide everything
log_user 1

# Get the command to execute from the first argument
set cmd [lindex $argv 0]
cd "$env(SWARMZ4_PATH)/PX4-Autopilot"

# Use bash to execute the command with environment variables
spawn bash -c "$cmd"

# Wait for the shell prompt to appear
expect "pxh>"

# Execute the sequence of commands with 1 second pause between each
# send "sensor_gps_sim stop\r"
# expect "pxh>"
# sleep 1

# send "sensor_gps_sim start\r"
# expect "pxh>"
# sleep 1

# send "sensor_baro_sim stop\r"
# expect "pxh>"
# sleep 1

# send "sensor_baro_sim start\r"
# expect "pxh>"
# sleep 1

# send "sensor_mag_sim stop\r"
# expect "pxh>"
# sleep 1

# send "sensor_mag_sim start\r"
# expect "pxh>"
# sleep 1

# send "fake_imu stop\r"
# expect "pxh>"
# sleep 1

# send "fake_imu start\r"
# expect "pxh>"
# sleep 1

# send "gyro_calibration start\r"
# expect "pxh>"
# sleep 1

send "ekf2 stop\r"
expect "pxh>"
sleep 3

send "ekf2 start\r"

interact