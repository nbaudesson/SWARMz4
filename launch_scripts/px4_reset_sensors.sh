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

# On the PX4 side, you are only required to stop the uXRCE-DDS time synchronization,
# setting the parameter UXRCE_DDS_SYNCT to false.
# By doing so, Gazebo will act as main and only time source for both ROS2 and PX4.
send "param set UXRCE_DDS_SYNCT 0\r"
expect "pxh>"
sleep 1

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

send "sensor_baro_sim start\r"
expect "pxh>"
sleep 1

send "sensor_mag_sim stop\r"
expect "pxh>"
sleep 1

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