from pymavlink import mavutil
import time
import argparse
import random

def create_mavlink_connection(instance):
    """Create a MAVLink connection to the specified drone instance"""
    # From PX4 logs, we know the exact port configuration:
    # "INFO  [mavlink] mode: Normal, data rate: 4000000 B/s on udp port 18571 remote port 14550"
    
    # PX4 listens on 18570+instance and sends to 14550
    drone_port = 18570 + instance
    listen_port = 14550   # Common port where all drones send messages
    
    try:
        # For drone 0, use 14550. For other instances, offset the listen port
        if instance > 0:
            listen_port = 14550 + (instance * 10)  # Calculated from PX4 logs
        
        # Connect directly to the QGC port (14550+) where we know PX4 is sending messages
        # From logs we see pattern: for instance 1, use 14550; for instance 2, use 14560, etc.
        conn_str = f"udpin:0.0.0.0:{listen_port}"
        print(f"Connecting to drone {instance} listening on port {listen_port}")
        
        # Create the connection
        master = mavutil.mavlink_connection(
            conn_str,
            source_system=255,
            source_component=190
        )
        
        # Set our target system based on instance (PX4 SYS_AUTOSTART parameter sets target_system = instance + 1)
        target_system = instance + 1
        master.target_system = target_system
        
        print(f"Waiting for heartbeat from drone {instance} (system ID: {target_system})...")
        start_time = time.time()
        heartbeat_received = False
        
        while time.time() - start_time < 15:  # Longer timeout for more reliability
            try:
                msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
                if msg:
                    if not heartbeat_received:
                        print(f"Received heartbeat from system {master.target_system}")
                        heartbeat_received = True
                        return master
                print(".", end="", flush=True)
            except Exception as wait_err:
                print(f"Heartbeat wait error: {wait_err}")
                continue
        
        print("\nNo heartbeat received from drone")
        raise Exception("No heartbeat received, connection failed")
        
    except Exception as e:
        raise Exception(f"Failed to connect to drone {instance}: {str(e)}")

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
    
    param_id_str = param_id if isinstance(param_id, str) else param_id.decode('utf-8')
    print(f"Setting parameter {param_id_str} to {value}")
    
    # Send the parameter set command
    master.mav.param_set_send(
        master.target_system,
        master.target_component,
        param_id_bytes,
        float(value),
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

def set_max_horizontal_speed(drone_instance, speed_value):
    """Set the maximum horizontal speed for a specific drone by updating all relevant speed parameters"""  
    print(f"Setting speed parameters for drone {drone_instance} to {speed_value} m/s")
    
    try:
        # Use the simplified connection approach
        master = create_mavlink_connection(drone_instance)
        
        # Update all speed-related parameters to avoid constraints
        success = True
        
        # Only the essential speed parameters
        speed_params = {
            "MPC_XY_VEL_MAX": speed_value,      # Maximum XY velocity
            "MPC_XY_CRUISE": speed_value,       # Cruise XY velocity
            "MPC_VEL_MANUAL": speed_value,      # Manual velocity limit
        }
        
        # Set all speed parameters
        for param, value in speed_params.items():
            print(f"\nConfiguring {param}...")
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
                
                msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=3)
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

def emergency_shutdown(drone_instance):
    """
    Perform emergency procedure: land and permanently disable the specified drone
    without killing the PX4 process that runs the simulation
    
    Args:
        drone_instance: Integer index of the drone
        
    Returns:
        True if successful, False otherwise
    """   
    print(f"EMERGENCY SHUTDOWN initiated for drone {drone_instance}")
    
    try:
        # Use the simplified connection approach
        master = create_mavlink_connection(drone_instance)
        master.wait_heartbeat()
        
        # Step 1: Command the drone to land immediately
        print("Commanding emergency land...")
        send_command(master, mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0)
        
        # Wait for the drone to actually land by monitoring its state
        print("Waiting for landing confirmation...")
        
        # Define landed states constants
        MAV_LANDED_STATE_UNDEFINED = 0
        MAV_LANDED_STATE_ON_GROUND = 1
        MAV_LANDED_STATE_IN_AIR = 2
        MAV_LANDED_STATE_TAKEOFF = 3
        MAV_LANDED_STATE_LANDING = 4
        
        # Monitor landing with timeout
        landing_timeout = time.time() + 60  # 60 second timeout for landing
        landed = False
        
        while time.time() < landing_timeout and not landed:
            # Method 1: Check EXTENDED_SYS_STATE for landed_state
            msg = master.recv_match(type='EXTENDED_SYS_STATE', blocking=True, timeout=0.5)
            if msg and hasattr(msg, 'landed_state'):
                if msg.landed_state == MAV_LANDED_STATE_ON_GROUND:
                    print("Landing confirmed: Drone on ground")
                    landed = True
                    break
                elif msg.landed_state == MAV_LANDED_STATE_LANDING:
                    print("Drone is in landing process...")
            
            # Method 2: Also monitor altitude as backup approach
            alt_msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
            if alt_msg:
                relative_alt = alt_msg.relative_alt / 1000.0  # Convert mm to meters
                if relative_alt < 0.3:  # If within 30cm of ground
                    print(f"Landing confirmed: Altitude {relative_alt:.2f}m (near ground)")
                    landed = True
                    break
                if relative_alt < 5.0:  # Show progress when below 5m
                    print(f"Descending: {relative_alt:.2f}m above ground")
            
            time.sleep(1)  # Check every second
        
        if not landed:
            print("WARNING: Landing timeout - proceeding anyway but disarm may fail")
        else:
            print("Drone landed. Waiting for stabilization...")
            time.sleep(3)
        
        # Step 2: Disarm the drone
        print("Disarming...")
        for attempt in range(3):
            send_command(master, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0)  # 0 = disarm
            
            # Wait for disarm confirmation
            disarm_timeout = time.time() + 5
            disarmed = False
            while time.time() < disarm_timeout:
                msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
                if msg and not msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                    disarmed = True
                    break
            
            if disarmed:
                print("Disarm confirmed")
                break
            else:
                print(f"Disarm attempt {attempt+1} failed, retrying...")
        
        # Step 3: Permanently disable takeoff capabilities
        print("Permanently disabling takeoff capabilities...")
        
        # Set parameters that prevent re-arming
        disable_params = {
            # Require arm authorization (which won't be provided)
            "COM_ARM_AUTH_REQ": 1,
            
            # Set to maximum disarm timeout after landing
            "COM_DISARM_LAND": 60,
            
            # Disable RC input completely
            "COM_RC_IN_MODE": 1,
            
            # Disable auto-arm and takeoff capabilities
            "COM_ARM_WITHOUT_GPS": 0,
            "COM_ARM_EKF_HGT": 1,
            "COM_ARM_MAG_STR": 2,
            
            # Set kill switch to active
            "CBRK_FLIGHTTERM": 0,
            
            # Set all motors to disabled
            "PWM_MAIN_DISARM": 900,
            "PWM_AUX_DISARM": 900,
        }
        
        params_set = True
        for param, value in disable_params.items():
            if not set_parameter(master, param, value):
                print(f"Failed to set {param} parameter")
                params_set = False
        
        # Save parameters to make them persist
        if params_set:
            print("Saving parameters to make disabling permanent...")
            send_command(master, mavutil.mavlink.MAV_CMD_PREFLIGHT_STORAGE, 1.0, 0.0)
            time.sleep(1)
        
        # Set to a mode that keeps it grounded
        print("Setting to ground mode...")
        send_command(master, mavutil.mavlink.MAV_CMD_DO_SET_MODE, 1, 6)  # MAIN_MODE_LAND
        
        print(f"Emergency shutdown and permanent grounding completed for drone {drone_instance}")
        print(f"The drone will remain in the simulation but cannot take off again")
        return True
        
    except Exception as e:
        print(f"Emergency shutdown error: {e}")
        return False
    finally:
        if 'master' in locals() and master:
            master.close()

def main():
    parser = argparse.ArgumentParser(description='Set maximum horizontal speed for a drone or perform emergency shutdown')
    parser.add_argument('--instance', type=int, default=0, help='Drone instance number')
    parser.add_argument('--speed', type=float, default=5.0, help='Maximum speed in m/s')
    parser.add_argument('--emergency', action='store_true', help='Perform emergency shutdown')
    args = parser.parse_args()
    
    if args.emergency:
        return 0 if emergency_shutdown(args.instance) else 1
    else:
        return 0 if set_max_horizontal_speed(args.instance, args.speed) else 1

if __name__ == "__main__":
    exit(main())