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
sleep 1

send "param set MC_SLOW_DEF_HVEL 3.0\r"
send "param set MPC_VEL_MANUAL 12.0\r"
send "param set MPC_XY_VEL_MAX 12.0\r"
send "param set MPC_XY_CRUISE 12.0\r"
send "param set MPC_XY_VEL_ALL -12.0\r"
send "param set MC_SLOW_DEF_VVEL 3.0\r"
send "param set MPC_Z_VEL_ALL -3.0\r"
send "param set MPC_Z_VEL_MAX_DN 3.0\r"
send "param set MPC_Z_VEL_MAX_UP 3.0\r"
send "param set MIS_TAKEOFF_ALT 5.0\r"

interact

# Default QGC parameter
# MC_SLOW_DEF_HVEL : id= 1019  default=3.0(m/s) Default horizontal velocity limit
# MPC_VEL_MANUAL   : id= 1101  default=10.0(m/s) Manual velocity limit
# MPC_XY_VEL_MAX   : id= 1114  default=12.0(m/s) Maximum horizontal velocity
# MPC_XY_CRUISE    : id= 1106  default=5.0(m/s) Cruise horizontal velocity
# MPC_XY_VEL_ALL   : id= 1111  default=-10.0(m/s) Overall horizontal velocity
# MC_SLOW_DEF_VVEL : id= 1020  default=1.0(m/s) Default vertical velocity limit
# MPC_Z_VEL_ALL    : id= 1122  default=-3.0(m/s) Overall vertical velocity limit
# MPC_Z_VEL_MAX_DN : id= 1125  default=1.5(m/s) Maximum downward vertical velocity
# MPC_Z_VEL_MAX_UP : id= 1126  default=3.0(m/s) Maximum upward vertical velocity
# MIS_TAKEOFF_ALT : id= 1043  default=2.5(m) Default takeoff altitude