from pymavlink import mavutil
import time

# List of connection strings for each drone instance
drone_connections = [
    'udp:127.0.0.1:14550'
]

# Function to send MAVLink command
def send_command(master, command, param1=0.0, param2=0.0):
    # master: The MAVLink connection object for communicating with the drone
    # command: The MAVLink command code to be executed by the drone
    # param1, param2: Command-specific parameters with default values of 0.0
    
    master.mav.command_long_send(
        master.target_system,  # System ID of the target drone
        master.target_component,  # Component ID within the target system
        command,  # The command code being sent
        0,  # Confirmation parameter (0 for first transmission)
        param1, param2,  # The first two command parameters
        0, 0, 0, 0, 0)  # Remaining five parameters (param3-param7) set to 0

# Function to set parameter value
def set_parameter(master, param_id, value, param_type=mavutil.mavlink.MAV_PARAM_TYPE_REAL32):
    """
    Set a parameter on the drone.
    
    Args:
        master: MAVLink connection object
        param_id: Parameter name (string)
        value: Parameter value (numeric)
        param_type: Parameter type (default: REAL32)
    
    Returns:
        True if successful, False otherwise
    """
    # Convert param_id to bytes if it's a string
    if isinstance(param_id, str):
        param_id_bytes = param_id.encode('utf-8')
    else:
        param_id_bytes = param_id
    
    # Keep a string version for printing
    param_id_str = param_id if isinstance(param_id, str) else param_id.decode('utf-8')
    print(f"Setting parameter {param_id_str} to {value}")
    
    # Send parameter set message
    master.mav.param_set_send(
        master.target_system,       # Target system
        master.target_component,    # Target component
        param_id_bytes,             # Parameter name as bytes
        float(value),               # Parameter value as float
        param_type                  # Parameter type
    )
    
    # Wait for parameter value message
    start_time = time.time()
    while time.time() - start_time < 5:  # 5 second timeout
        msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=1)
        if msg is not None:
            # Handle parameter ID which might be string or bytes
            msg_param_id = msg.param_id
            if isinstance(msg_param_id, bytes):
                msg_param_id = msg_param_id.decode('utf-8')
            
            if msg_param_id == param_id_str:
                if abs(msg.param_value - float(value)) < 0.001:  # Compare with small tolerance
                    print(f"Parameter {param_id_str} successfully set to {msg.param_value}")
                    return True
                else:
                    print(f"Parameter set failed. Current value: {msg.param_value}")
                    return False
    
    print(f"Timeout waiting for parameter confirmation")
    return False

# Parameters in MAVLink commands (param1-param7) explanation:
# - The meaning of each parameter depends on the specific command being sent
# - For MAV_CMD_PREFLIGHT_STORAGE (used in this example):
#   - param1: 1.0 = Save parameters, 0.0 = Load parameters
#   - param2: If loading (param1=0), set to 1.0 to load all parameters including default values
# 
# Common parameter patterns for other commands:
# - Navigation commands: param1-4 often represent coordinates, altitude, and speed
# - Arming commands: param1 typically indicates arm/disarm (1.0/0.0)
# - Mode change commands: param1-2 often represent mode ID and custom settings
# - Mission commands: parameters may indicate waypoint attributes
#
# All seven parameters must be provided in command_long_send(), even if unused

# Iterate over each drone connection
for i, connection_string in enumerate(drone_connections):
    print(f"Connecting to drone instance {i}...")
    master = mavutil.mavlink_connection(connection_string)

    # Wait for the heartbeat to find the system ID
    master.wait_heartbeat()
    print(f"Heartbeat from system (system {master.target_system} component {master.target_component})")

    # Set MPC_XY_VEL_MAX to 5 m/s
    set_parameter(master, "MPC_XY_VEL_MAX", 5.0)
    
    # This command does NOT stop or reset EKF2
    # MAV_CMD_PREFLIGHT_STORAGE actually saves/loads parameters to/from storage
    # With params (1.0, 0.0) it's saving parameters to storage
    send_command(master, mavutil.mavlink.MAV_CMD_PREFLIGHT_STORAGE, 1.0, 0.0)  # Saves parameters
    print(f"Parameters saved for drone instance {i}")  # Updated comment

    # This also doesn't start EKF2
    # With params (1.0, 1.0) it's still saving parameters with different options
    send_command(master, mavutil.mavlink.MAV_CMD_PREFLIGHT_STORAGE, 1.0, 1.0)  # Saves parameters
    print(f"Parameters saved with additional options for drone instance {i}")  # Updated comment
    
    # To actually reset EKF2, you would need a different command like:
    # send_command(master, mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 1.0, 0.0, 1.0)
    # Where the third parameter (1.0) specifies EKF2 reset