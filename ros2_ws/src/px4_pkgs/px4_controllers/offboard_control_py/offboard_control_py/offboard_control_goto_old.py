"""
PX4 Offboard Control Node for Waypoint Navigation

This node allows controlling a PX4 drone through predefined waypoints.
Supports both NED (North-East-Down) and FRD (Front-Right-Down) coordinate frames.

Usage:
    1. Launch with default waypoints:
       $ ros2 run offboard_control_py offboard_control_goto --ros-args -p frame:=ned --remap __ns:=/px4_1

    2. Launch with custom waypoint file:
       ros2 run offboard_control_py offboard_control_goto --ros-args -p frame:=ned -p mission_file:=mission.yaml --remap __ns:=/px4_1

Configuration:
    - frame: 'ned' or 'frd' (default: 'frd')
    - mission_file: path to YAML waypoint file (optional)

Waypoint File Format (mission.yaml):
    {
      "1": [  # Waypoints for drone with instance 0 (namespace /px4_1)
        {"x": 5.0, "y": 0.0, "z": -2.0, "yaw": 0.0, "time": 5},     # Face North, hover 5s
        {"x": 5.0, "y": 5.0, "z": -2.0, "yaw": 90.0, "time": 10}    # Face East, hover 10s
      ],
      "2": [  # Waypoints for drone with instance 1 (namespace /px4_2)
        {"x": -5.0, "y": 0.0, "z": -2.0, "yaw": 180.0, "time": 5}   # Face South, hover 5s
      ]
    }

Waypoint Parameters:
    - x, y, z: Position coordinates (in meters)
    - yaw: Heading angle in degrees (0=North, 90=East, 180=South, 270=West)
    - time: Hover time at waypoint in seconds (optional, default=5.0)
    - z: Height in meters, negative is up (optional, default=-2.0)

File Search Paths:
    1. Direct path provided
    2. <package_share>/missions/
    3. <package_share>/config/
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand
from px4_msgs.msg import VehicleLocalPosition, VehicleStatus
import numpy as np
import math
import time
import yaml
import os
from ament_index_python.packages import get_package_share_directory

class OffboardControlGoto(Node):
    """Node for controlling a vehicle through waypoints in either NED or FRD frame."""

    # Update default waypoints to use degrees
    DEFAULT_WAYPOINTS = [
        [5.0, 0.0, -2.0, 0.0, 5.0],     # [x, y, z, yaw_degrees, hover_time] - facing North
        [5.0, 5.0, -2.0, 90.0, 5.0],    # facing East
        [0.0, 5.0, -2.0, 180.0, 5.0],   # facing South
        [0.0, 0.0, -2.0, 270.0, 5.0],   # facing West
    ]

    def __init__(self) -> None:
        super().__init__('offboard_control_goto')

        # Get namespace and extract instance first
        self.node_namespace = self.get_namespace()
        self.instance = self.extract_instance_from_namespace()

        # Then add parameters
        self.declare_parameter('mission_file', '')  # Empty string means use default waypoints
        self.declare_parameter('frame', 'frd')      # 'frd' or 'ned'

        # Get parameters
        self.frame = self.get_parameter('frame').value
        mission_file = self.get_parameter('mission_file').value

        # Now load waypoints after instance is set
        self.WAYPOINTS = self.load_waypoints(mission_file)

        if self.frame not in ['frd', 'ned']:
            self.get_logger().error(f'Invalid frame: {self.frame}. Using FRD.')
            self.frame = 'frd'

        # Initialize control parameters
        self.POSITION_THRESHOLD = 0.3     # meters
        self.TAKEOFF_HEIGHT = 2.0        # meters
        self.TAKEOFF_THRESHOLD = 0.1     # meters
        self.current_waypoint_index = 0
        self.objective_complete = False
        
        # Get namespace and extract instance
        self.node_namespace = self.get_namespace()
        self.instance = self.extract_instance_from_namespace()
        
        # Initialize state variables
        self.initial_position = None
        self.ready = False
        self.armed = False
        self.in_offboard_mode = False
        self.has_taken_off = False
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()

        # Setup QoS and create publishers/subscribers
        self.setup_communications()

        # Add initialization flags
        self.fcu_params_ready = False
        self.command_retry_counter = 0
        self.initialization_retry_counter = 0
        self.preflight_checks_ok = False
        self.debug_counter = 0

        # Add hover time tracking
        self.hover_start_time = None
        self.current_hover_time = 0.0

        # Create timers
        self.init_timer = self.create_timer(1.0, self.initialization_check)
        self.control_timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info(
            f'Goto Controller started in {self.frame.upper()} frame mode')

    def extract_instance_from_namespace(self):
        """Extract instance number from namespace."""
        if self.node_namespace == '/':
            self.node_namespace = ''
            return 0

        last_slash = self.node_namespace.rfind('_')
        if (last_slash != -1 and last_slash + 1 < len(self.node_namespace)):
            try:
                return int(self.node_namespace[last_slash + 1:])
            except (ValueError, OverflowError) as e:
                self.get_logger().error(f'Error extracting instance: {str(e)}')
        return 0

    def setup_communications(self):
        """Setup publishers and subscribers with appropriate QoS."""
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, f'{self.node_namespace}/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, f'{self.node_namespace}/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, f'{self.node_namespace}/fmu/in/vehicle_command', qos_profile)

        # Subscribers
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, f'{self.node_namespace}/fmu/out/vehicle_local_position',
            self.vehicle_local_position_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, f'{self.node_namespace}/fmu/out/vehicle_status',
            self.vehicle_status_callback, qos_profile)

    def load_waypoints(self, mission_file):
        """Load waypoints from YAML file or use defaults."""
        if not mission_file:
            self.get_logger().info('No mission file specified, using default waypoints')
            return self.DEFAULT_WAYPOINTS

        # Try to find the file in different locations
        file_locations = [
            mission_file,  # Direct path
            os.path.join(get_package_share_directory('offboard_control_py'), 'missions', mission_file),
            os.path.join(get_package_share_directory('offboard_control_py'), 'config', mission_file)
        ]

        for file_path in file_locations:
            try:
                with open(file_path, 'r') as f:
                    mission_data = yaml.safe_load(f)
                    
                    # Extract waypoints based on node's instance number
                    instance_waypoints = mission_data.get(str(self.instance + 1))
                    if instance_waypoints:
                        waypoints = []
                        for wp in instance_waypoints:
                            # Convert mission format to waypoint format
                            # [x, y, z, yaw, hover_time]
                            waypoint = [
                                float(wp['x']),
                                float(wp['y']),
                                float(wp.get('z', -2.0)),  # Default height if not specified
                                float(wp.get('yaw', 0.0)),  # yaw in degrees, 0=North, 180=South
                                float(wp.get('time', 1.0))  # Default hover time if not specified
                            ]
                            # Normalize yaw angle to 0-360 degrees
                            waypoint[3] = waypoint[3] % 360.0
                            waypoints.append(waypoint)
                        
                        self.get_logger().info(
                            f'Loaded {len(waypoints)} waypoints from {file_path} '
                            f'for instance {self.instance + 1}')
                        return waypoints
                    else:
                        self.get_logger().warning(
                            f'No waypoints found for instance {self.instance + 1} '
                            f'in {file_path}, using defaults')
                        return self.DEFAULT_WAYPOINTS

            except FileNotFoundError:
                continue
            except yaml.YAMLError as e:
                self.get_logger().error(f'Error parsing YAML file {file_path}: {e}')
                return self.DEFAULT_WAYPOINTS
            except Exception as e:
                self.get_logger().error(f'Error loading waypoints from {file_path}: {e}')
                return self.DEFAULT_WAYPOINTS

        self.get_logger().warning(
            f'Mission file {mission_file} not found in any location, using defaults')
        return self.DEFAULT_WAYPOINTS

    def transform_waypoint(self, waypoint):
        """Transform waypoint based on frame type."""
        if self.frame == 'ned':
            pos = np.array(waypoint[:3])
            yaw_deg = waypoint[3]  # yaw in degrees
            hover_time = waypoint[4]
            
            # Convert yaw from degrees to radians for PX4
            yaw_rad = math.radians(yaw_deg)
            
            local_pos = pos + self.initial_position
            return local_pos, yaw_rad, hover_time
        else:  # FRD frame
            pos = np.array(waypoint[:3])
            yaw_deg = waypoint[3]  # yaw in degrees
            
            # Convert yaw from degrees to radians for PX4
            yaw_rad = math.radians(yaw_deg)
            
            local_pos = pos + self.initial_position
            return local_pos, yaw_rad, waypoint[4]

    def check_hover_complete(self, hover_time):
        """Check if we've hovered for the required time."""
        current_time = time.time()
        
        if self.hover_start_time is None:
            self.hover_start_time = current_time
            return False
            
        hover_duration = current_time - self.hover_start_time
        return hover_duration >= hover_time

    def check_position_reached(self, target_pos):
        """Check if current position is within threshold of target."""
        current_pos = np.array([
            self.vehicle_local_position.x,
            self.vehicle_local_position.y,
            self.vehicle_local_position.z
        ])
        distance = np.linalg.norm(target_pos - current_pos)
        return distance < self.POSITION_THRESHOLD

    def get_next_setpoint(self):
        """Get next waypoint if current one is reached and hover time completed."""
        if self.current_waypoint_index >= len(self.WAYPOINTS):
            return None, None, None

        waypoint = self.WAYPOINTS[self.current_waypoint_index]
        local_pos, local_yaw, hover_time = self.transform_waypoint(waypoint)
        
        if self.check_position_reached(local_pos):
            if not self.check_hover_complete(hover_time):
                # Keep current waypoint while hovering
                return local_pos, local_yaw, hover_time
                
            # Move to next waypoint after hover time is complete
            self.get_logger().info(
                f'Waypoint {self.current_waypoint_index} completed '
                f'(hovered for {hover_time:.1f} seconds)')
            self.current_waypoint_index += 1
            self.hover_start_time = None  # Reset hover timer
            
            if self.current_waypoint_index >= len(self.WAYPOINTS):
                self.objective_complete = True
                self.get_logger().info('All waypoints completed!')
                return None, None, None
            
            # Get next waypoint
            waypoint = self.WAYPOINTS[self.current_waypoint_index]
            local_pos, local_yaw, hover_time = self.transform_waypoint(waypoint)
            self.get_logger().info(
                f'Moving to waypoint {self.current_waypoint_index}: '
                f'[{local_pos[0]:.1f}, {local_pos[1]:.1f}, {local_pos[2]:.1f}], '
                f'heading: {math.degrees(local_yaw):.1f}°, '
                f'hover time: {hover_time:.1f}s')
        
        return local_pos, local_yaw, hover_time

    def vehicle_local_position_callback(self, msg):
        """Store position and set initial reference"""
        self.vehicle_local_position = msg
        
        if not self.ready and msg.xy_valid:
            self.initial_position = np.array([msg.x, msg.y, msg.z])
            self.ready = True
            self.get_logger().info(
                f'Initial position set: [North={msg.x:.2f}, East={msg.y:.2f}, Down={msg.z:.2f}]' 
                if self.frame == 'ned' else
                f'Initial position set: [Front={msg.x:.2f}, Right={msg.y:.2f}, Down={msg.z:.2f}]'
            )

    def vehicle_status_callback(self, msg):
        """Store vehicle status and track mode changes"""
        prev_armed = self.armed
        prev_offboard = self.in_offboard_mode
        
        self.vehicle_status = msg
        self.armed = msg.arming_state == VehicleStatus.ARMING_STATE_ARMED
        self.in_offboard_mode = msg.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
        
        # Log state changes
        if prev_armed != self.armed:
            self.get_logger().info(f'Armed state changed to: {self.armed}')
        if prev_offboard != self.in_offboard_mode:
            self.get_logger().info(f'Offboard mode changed to: {self.in_offboard_mode}')

    def arm(self):
        """Send an arm command to the vehicle."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    def disarm(self):
        """Send a disarm command to the vehicle."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')

    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")

    def land(self):
        """Switch to land mode."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Switching to land mode")

    def publish_offboard_control_heartbeat_signal(self):
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_position_setpoint(self, x: float, y: float, z: float, yaw: float = 0.0):
        """Publish the trajectory setpoint."""
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = yaw
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    def publish_vehicle_command(self, command, **params) -> None:
        """Publish a vehicle command."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1 + self.instance
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def initialization_check(self):
        """Check if system is ready for offboard control."""
        if not self.fcu_params_ready:
            self.initialization_retry_counter += 1
            self.get_logger().info('Waiting for FCU connection...')
            
            if (hasattr(self.vehicle_local_position, 'timestamp') and 
                self.vehicle_local_position.timestamp > 0 and
                hasattr(self.vehicle_status, 'timestamp') and
                self.vehicle_status.timestamp > 0):
                
                self.fcu_params_ready = True
                self.get_logger().info('FCU connection established!')
                self.preflight_checks()
            elif self.initialization_retry_counter > 10:
                self.get_logger().error('Failed to establish FCU connection!')
                rclpy.shutdown()

    def preflight_checks(self):
        """Perform pre-flight checks."""
        checks_passed = True
        
        if not self.vehicle_local_position.xy_valid:
            self.get_logger().warning('Position data not valid')
            checks_passed = False
            
        if self.vehicle_status.arming_state not in [
            VehicleStatus.ARMING_STATE_DISARMED,
            VehicleStatus.ARMING_STATE_ARMED
        ]:
            self.get_logger().warning(
                f'Vehicle in invalid arming state: {self.vehicle_status.arming_state}')
            checks_passed = False
            
        if checks_passed:
            self.preflight_checks_ok = True
            self.get_logger().info('Pre-flight checks passed')
            
            # Log waypoint information after checks pass
            waypoint_info = "\nLoaded waypoints:"
            for i, wp in enumerate(self.WAYPOINTS):
                waypoint_info += f"\n  {i}: pos=[{wp[0]:.1f}, {wp[1]:.1f}, {wp[2]:.1f}], " \
                               f"heading={wp[3]:.1f}°, hover_time={wp[4]:.1f}s"
            self.get_logger().info(waypoint_info)
        else:
            self.get_logger().warning('Pre-flight checks failed, retrying...')

    def control_loop(self) -> None:
        """Main control loop running at 10Hz."""
        self.publish_offboard_control_heartbeat_signal()

        # Debug info every 50 cycles (5 seconds)
        self.debug_counter += 1
        if self.debug_counter >= 50:
            self.get_logger().info(
                f'Status - Armed: {self.armed}, Offboard: {self.in_offboard_mode}, '
                f'FCU ready: {self.fcu_params_ready}, Checks OK: {self.preflight_checks_ok}, '
                f'Waypoint: {self.current_waypoint_index}/{len(self.WAYPOINTS)}'
            )
            self.debug_counter = 0

        # Wait for initialization
        if not self.fcu_params_ready or not self.preflight_checks_ok or not self.ready:
            return

        # Handle arming and offboard transition
        if not self.armed or not self.in_offboard_mode:
            self.command_retry_counter += 1
            if self.command_retry_counter >= 10:
                self.command_retry_counter = 0
                if not self.armed:
                    self.arm()
                if not self.in_offboard_mode:
                    self.engage_offboard_mode()
            return

        # Navigation logic
        if not self.has_taken_off:
            # Takeoff sequence
            takeoff_pos = self.initial_position.copy()
            takeoff_pos[2] -= self.TAKEOFF_HEIGHT
            self.publish_position_setpoint(
                takeoff_pos[0],
                takeoff_pos[1],
                takeoff_pos[2]
            )
            if abs(self.vehicle_local_position.z - takeoff_pos[2]) < self.TAKEOFF_THRESHOLD:
                self.has_taken_off = True
                self.get_logger().info('Takeoff complete, starting waypoint navigation')
        else:
            if not self.objective_complete:
                # Get and navigate to next waypoint
                local_pos, local_yaw, _ = self.get_next_setpoint()
                if local_pos is not None:
                    self.publish_position_setpoint(
                        local_pos[0],
                        local_pos[1],
                        local_pos[2],
                        local_yaw
                    )
            else:
                # Mission complete, hold position
                self.publish_position_setpoint(
                    self.vehicle_local_position.x,
                    self.vehicle_local_position.y,
                    self.vehicle_local_position.z
                )

def main(args=None) -> None:
    print('Starting offboard control goto node...')
    rclpy.init(args=args)
    offboard_control = OffboardControlGoto()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
