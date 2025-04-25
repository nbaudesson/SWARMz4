
from pymavlink import mavutil
import time


def create_mavlink_connection(port):
    #Create a MAVLink connection to the specified drone instance

    try:
        conn_str = f"udpin:0.0.0.0:{port}"
        print(f"Connecting to drones listening on port {port}")

        master = mavutil.mavlink_connection(
            conn_str,
            source_system=255,
            source_component=190
        )

        print(f"Waiting for heartbeat from drone...")

        # Wait for the first heartbeat to get system/component IDs
        msg = master.wait_heartbeat(blocking=True, timeout=15)

        if msg:
            # Manually extract and set target_system/component
            master.target_system = msg.get_srcSystem()
            master.target_component = msg.get_srcComponent()
            print(f"Received heartbeat from system {master.target_system}, component {master.target_component}")
            return master
        else:
            raise Exception("No heartbeat received, connection failed")

    except Exception as e:
        raise Exception(f"Failed to connect to drone : {str(e)}")





def send_command(master, command, param1=0.0, param2=0.0):
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        command,
        0,
        param1, param2,
        0, 0, 0, 0, 0)

def set_parameter(master, param_id, value, param_type=mavutil.mavlink.MAV_PARAM_TYPE_REAL32):
    if isinstance(param_id, str):
        param_id_bytes = param_id.encode('utf-8')
    else:
        param_id_bytes = param_id

    if len(param_id_bytes) > 16:
        raise ValueError(f"param_id '{param_id}' is too long (max 16 bytes)")
    
    param_id_str = param_id if isinstance(param_id, str) else param_id.decode('utf-8')
    print(f"Setting parameter {param_id_str} to {value}")

    print(f"target system : {master.target_system} and type : {type(master.target_system)}")
    print(f"target component : {master.target_component} and type : {type(master.target_component)}")

    # Send the parameter set command
    master.mav.param_set_send(
        master.target_system,
        master.target_component,
        param_id_bytes,
        value,
        param_type
    )
    
    # Wait for initial parameter response
    initial_set_success = False
    start_time = time.time()
    while time.time() - start_time < 5:
        msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=1)
        if msg is not None:
            msg_param_id = msg.param_id
            if isinstance(msg_param_id, bytes):
                msg_param_id = msg_param_id.decode('utf-8')
            
            if msg_param_id == param_id_str:
                if abs(msg.param_value - float(value)) < 0.001:
                    print(f"Parameter {param_id_str} set to {msg.param_value}")
                    initial_set_success = True
                    break
                else:
                    print(f"Parameter set failed: {msg.param_value}")
                    return False
    
    if not initial_set_success:
        print(f"Timeout waiting for parameter confirmation")
        return False
    
    # Verify the parameter was actually set
    print(f"Parameter set response received, verifying current value...")
    
    # Explicitly request the parameter to verify it was set correctly
    master.mav.param_request_read_send(
        master.target_system, master.target_component,
        param_id_bytes, -1
    )
    
    # Wait for the parameter value response
    verify_start_time = time.time()
    while time.time() - verify_start_time < 3:
        msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=1)
        if msg is not None:
            msg_param_id = msg.param_id
            if isinstance(msg_param_id, bytes):
                msg_param_id = msg_param_id.decode('utf-8')
            
            if msg_param_id == param_id_str:
                if abs(msg.param_value - float(value)) < 0.001:
                    print(f"✓ Verified: {param_id_str} = {msg.param_value}")
                    return True
                else:
                    print(f"✗ Verification failed: {param_id_str} = {msg.param_value}, expected {value}")
                    return False
    
    print(f"Timeout waiting for parameter verification")
    return False
    
    
def get_stable_drones_namespaces(node, max_attempts=10, wait_time=1.0):
    """
    Gets a stable list of namespaces by checking if two consecutive calls return the same result.
    
    Args:
    node: The ROS2 node to use for namespace detection
    max_attempts: Maximum number of attempts to get stable namespaces
    wait_time: Time to wait between attempts in seconds
        
    Returns:
        List of namespaces that remained stable between two consecutive checks
    """
    prev_namespaces = None
    attempts = 0
    
    while attempts < max_attempts:
        current_namespaces = get_all_drones(node)
        
        if not current_namespaces:
            node.get_logger().warn("No namespaces detected. Waiting...")
            time.sleep(wait_time)
            attempts += 1
            continue
            
        # Sort the lists to ensure consistent comparison
        current_namespaces.sort()
        
        if prev_namespaces is not None and current_namespaces == prev_namespaces:
            node.get_logger().info(f"Stable list of namespaces detected: {current_namespaces}")
            return current_namespaces
            
        # node.get_logger().info(f"Detected namespaces (attempt {attempts+1}/{max_attempts}): {current_namespaces}")
        prev_namespaces = current_namespaces
        attempts += 1
        time.sleep(wait_time)
    
    # If we couldn't get a stable list, return the last one we got
    node.get_logger().warn(f"Could not get stable namespaces after {max_attempts} attempts. Using last detected list: {prev_namespaces}")
    return prev_namespaces or []


def get_all_drones(node):
    """
    Get all PX4 drones running in the ROS 2 system through topics.
    :param node: The rclpy node
    :return: List of PX4 drone names
    """
    topic_names_and_types = node.get_topic_names_and_types()
    drones = set()
    for topic_name, _ in topic_names_and_types:
        if 'px4' in topic_name:
            namespace = topic_name.split('/')[1]
            if namespace:
                drones.add(f'/{namespace}')
    return list(drones)


def set_max_speed(drone_instance, master, speed_value_horizontal, speed_value_vertical):
    """Set the maximum horizontal speed for a specific drone by updating all relevant speed parameters"""  
    print(f"\nSetting speed parameters for drone {drone_instance} to : in XY {speed_value_horizontal} m/s and in Z {speed_value_vertical} m/s")

    try:
        
        # Update all speed-related parameters to avoid constraints
        success = True
        
        # Only the essential speed parameters
        speed_params = {
            "MPC_XY_VEL_MAX": speed_value_horizontal,      # Maximum XY velocity
            "MPC_XY_CRUISE": speed_value_horizontal,       # Cruise XY velocity
            "MPC_VEL_MANUAL": speed_value_horizontal,      # Manual velocity limit
            "MPC_Z_VEL_MAX_UP": speed_value_vertical,      # Maximum ascent velocity
            "MPC_Z_VEL_MAX_DN": speed_value_vertical,      # Maximum descent velocity
        }
        
        # Set all speed parameters
        for param, value in speed_params.items():
            print(f"\nConfiguring {param}...")
            print(f"param : {param}, type : {type(param)} ; value : {value}, type : {type(value)}") 
            if not set_parameter(master, param, value):
                success = False
                print(f"Failed to set {param}")
        
        # Save parameters to storage if all settings were successful
        if success:
            print("\nSaving parameters to persistent storage...")
            send_command(master, mavutil.mavlink.MAV_CMD_PREFLIGHT_STORAGE, 1.0, 0.0)
            time.sleep(2)  # Wait for storage operation to complete
            
            # Verify the parameters after saving
            print("\nVerifying parameters after save operation...")
            verification_success = True
            for param, value in speed_params.items():
                # Request parameter again to verify it was saved
                master.mav.param_request_read_send(
                    master.target_system, master.target_component, 
                    param.encode('utf-8'), -1
                )
                
                msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=10)
                if msg:
                    if abs(msg.param_value - float(value)) < 0.001:
                        print(f"✓ {param} correctly saved as {msg.param_value}")
                    else:
                        print(f"✗ {param} verification failed: {msg.param_value} != {value}")
                        verification_success = False
                else:
                    print(f"? {param} could not be verified (timeout)")
                    verification_success = False
            
            if verification_success:
                print(f"\nAll speed parameters successfully set, saved, and verified for drone {drone_instance}")
                return True
            else:
                print(f"\nSome parameter verifications failed for drone {drone_instance}")
                return False
        else:
            print(f"\nFailed to set one or more speed parameters for drone {drone_instance}")
            return False
        
    except Exception as e:
        print(f"Error: {e}")
        return False
    finally:
        if 'master' in locals() and master:
            master.close()
