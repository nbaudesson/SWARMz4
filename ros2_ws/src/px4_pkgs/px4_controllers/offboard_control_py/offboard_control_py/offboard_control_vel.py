#!/usr/bin/env python3
"""
PX4 Velocity Control Node for ROS 2

This node provides velocity-based control for PX4-powered drones using ROS 2. It enables:
- Velocity control in FLU (Forward-Left-Up), FRD (Forward-Right-Down), or NED (North-East-Down) frames
- Automatic takeoff and landing sequences
- Position holding when zero velocity commanded
- Safety features including timeout handling and auto-landing

Requirements:
------------
- ROS 2 (tested with Foxy/Humble)
- PX4 Autopilot
- px4_msgs package
- Valid PX4 RTPS/ROS2 bridge configuration

Usage:
------
1. Launch the PX4 RTPS bridge
2. Start this node:
   $ ros2 run px4_controllers offboard_control_vel

3. Publish velocity commands using geometry_msgs/Twist messages:
   $ ros2 topic pub -1 ~/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {z: 0.0}}"

   Command structure:
   - linear.x: Forward velocity (m/s)
   - linear.y: Lateral velocity (m/s) - left(+)/right(-) in FLU, right(+)/left(-) in FRD/NED
   - linear.z: Vertical velocity (m/s) - up(+)/down(-) in FLU, down(+)/up(-) in FRD/NED
   - angular.z: Yaw rate (rad/s) - counter-clockwise(+)/clockwise(-)

   Common commands:
   - Takeoff: First non-zero velocity command initiates automatic takeoff
   - Hover: Send zero velocity {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {z: 0.0}}
   - Land: The drone will auto-land when hovering below land_height_threshold

Parameters:
-----------
- max_horizontal_speed: Maximum horizontal velocity (m/s)
- max_vertical_speed: Maximum vertical velocity (m/s)
- max_yaw_rate: Maximum yaw rotation rate (rad/s)
- velocity_timeout: Time before zero velocity is commanded after no input (s)
- frame_type: Reference frame for velocity commands ('FLU', 'FRD', or 'NED')
- takeoff_height: Height for automatic takeoff (m)
- hover_timeout: Time to hover before auto-landing (s)
- land_height_threshold: Height threshold for landing detection (m)

Publishers:
----------
- ~/fmu/in/offboard_control_mode: PX4 offboard control mode
- ~/fmu/in/trajectory_setpoint: Position/velocity setpoints
- ~/fmu/in/vehicle_command: Vehicle commands (arm, takeoff, etc.)

Subscribers:
-----------
- ~/fmu/out/vehicle_local_position: Current vehicle position and velocity
- ~/fmu/out/vehicle_status: Vehicle status (armed, mode, etc.)
- ~/cmd_vel: Velocity commands (geometry_msgs/Twist)

Author: [Your Name]
License: [License Type]
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import (
    OffboardControlMode, 
    TrajectorySetpoint, 
    VehicleCommand, 
    VehicleLocalPosition, 
    VehicleStatus
)
from geometry_msgs.msg import Twist
import time
import math

class OffboardControlVel(Node):
    """
    Node for controlling PX4 drone using pure velocity commands.
    
    This class implements a state machine with the following states:
    - IDLE: Waiting for commands
    - TAKEOFF: Executing automatic takeoff sequence
    - VELOCITY: Processing velocity commands
    - LANDING: Executing landing sequence
    
    The controller handles frame transformations, command rate limiting,
    and safety features like timeout handling and auto-landing.
    """
    
    # Control state definitions
    IDLE = 0      # Waiting for initial commands
    TAKEOFF = 1   # Executing takeoff sequence
    VELOCITY = 2  # Processing velocity commands
    LANDING = 3   # Executing landing sequence

    def __init__(self) -> None:
        """Initialize the velocity control node."""
        super().__init__('offboard_control_vel')
        
        # Define and declare all operational parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('max_horizontal_speed', 12.0),     # Maximum horizontal velocity (m/s)
                ('max_vertical_speed', 12.0),       # Maximum vertical velocity (m/s)
                ('max_yaw_rate', 10.0),            # Maximum yaw rate (rad/s)
                ('velocity_timeout', 2.0),          # Time before zeroing velocity (s)
                ('frame_type', 'FLU'),             # Reference frame for commands
                ('takeoff_height', 2.0),           # Target takeoff height (m)
                ('hover_timeout', 10.0),           # Hover time before auto-land (s)
                ('land_height_threshold', 1.0),    # Height to trigger landing (m)
            ]
        )
        
        # Store parameters
        self._max_horizontal_speed = self.get_parameter('max_horizontal_speed').value
        self._max_vertical_speed = self.get_parameter('max_vertical_speed').value
        self._max_yaw_rate = self.get_parameter('max_yaw_rate').value
        self._velocity_timeout = self.get_parameter('velocity_timeout').value
        self._frame_type = self.get_parameter('frame_type').value.upper()
        self._takeoff_height = self.get_parameter('takeoff_height').value
        self._hover_timeout = self.get_parameter('hover_timeout').value
        self._land_height_threshold = self.get_parameter('land_height_threshold').value

        # Validation
        if self._frame_type not in ['NED', 'FRD', 'FLU']:
            self.get_logger().error(f'Invalid frame type: {self._frame_type}. Using FLU.')
            self._frame_type = 'FLU'
        
        # Essential state variables
        self._state = self.IDLE
        self._armed = False
        self._offboard_mode = False
        self._current_pos = [0.0, 0.0, 0.0]
        self._current_yaw = 0.0
        self._current_vel = [0.0, 0.0, 0.0]
        self._takeoff_complete = False
        self._position_valid = False
        self._last_velocity_command = None
        self._last_command_time = 0.0
        self._command_start_heading = None  # Used for FRD frame
        self._hovering_start_time = None    # Used for auto-land timing
        self._land_start_time = None  # Add this for landing timeout tracking
        self._disarm_sent = False  # Add this to track disarm command
        
        # Multi-vehicle support
        self.node_namespace = self.get_namespace()
        self.instance = int(self.node_namespace.split('_')[-1]) if '_' in self.node_namespace else 0

        # Add command rate limiting
        self._command_rate = 10.0  # Hz
        
        # Add zero velocity command for timeout
        self._zero_cmd = Twist()
        self._zero_cmd.linear.x = 0.0
        self._zero_cmd.linear.y = 0.0
        self._zero_cmd.linear.z = 0.0
        self._zero_cmd.angular.z = 0.0

        # Add hold position state
        self._hold_position = None  # Will store position when zero velocity commanded

        # Setup publishers, subscribers and timer
        self.setup_communication(
            QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST,
                depth=1
            )
        )
        
        self.log_configuration()

    def setup_communication(self, qos_profile):
        """
        Setup all ROS publishers, subscribers and timers.
        
        Args:
            qos_profile: QoS profile for PX4 communication
        """
        # Publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode,
            f'{self.node_namespace}/fmu/in/offboard_control_mode',
            qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint,
            f'{self.node_namespace}/fmu/in/trajectory_setpoint',
            qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand,
            f'{self.node_namespace}/fmu/in/vehicle_command',
            qos_profile)
            
        # Subscribers
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition,
            f'{self.node_namespace}/fmu/out/vehicle_local_position',
            self.vehicle_local_position_callback,
            qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus,
            f'{self.node_namespace}/fmu/out/vehicle_status',
            self.vehicle_status_callback,
            qos_profile)
        self.velocity_command_subscriber = self.create_subscription(
            Twist,
            f'{self.node_namespace}/cmd_vel',
            self.velocity_command_callback,
            10)
            
        # Control loop timer - use command rate
        self.timer = self.create_timer(1.0/self._command_rate, self.control_loop)

    def log_configuration(self):
        """Log current configuration settings."""
        self.get_logger().info(
            f'Velocity Controller initialized:\n'
            f'  Frame: {self._frame_type}\n'
            f'  Max speeds: H={self._max_horizontal_speed}m/s, V={self._max_vertical_speed}m/s\n'
            f'  Max yaw rate: {self._max_yaw_rate}rad/s\n'
            f'  Command rate: {self._command_rate}Hz\n'
            f'  Timeout: {self._velocity_timeout}s\n'
            f'  Takeoff height: {self._takeoff_height}m\n'
            f'  Hover timeout: {self._hover_timeout}s\n'
            f'  Land height threshold: {self._land_height_threshold}m'
        )

    def vehicle_local_position_callback(self, msg):
        """Update current position and velocity."""
        self._current_pos = [msg.x, msg.y, msg.z]
        self._current_yaw = msg.heading
        self._current_vel = [msg.vx, msg.vy, msg.vz]
        
        # Updated validation check with correct field names
        if msg.xy_valid and msg.z_valid and msg.heading != float('nan'):
            self._position_valid = True

    def vehicle_status_callback(self, msg):
        """Handle vehicle status updates."""
        self._armed = msg.arming_state == VehicleStatus.ARMING_STATE_ARMED
        self._offboard_mode = msg.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD

    def velocity_command_callback(self, msg):
        """Process incoming velocity commands."""
        # Ignore zero commands when checking for new takeoff
        has_motion = (abs(msg.linear.x) >= 0.01 or 
                     abs(msg.linear.y) >= 0.01 or 
                     abs(msg.linear.z) >= 0.01 or 
                     abs(msg.angular.z) >= 0.01)

        # Check if this is a zero velocity command
        if not has_motion:
            # Store current position for position control
            self._hold_position = {
                'x': self._current_pos[0],
                'y': self._current_pos[1],
                'z': self._current_pos[2],
                'yaw': self._current_yaw
            }
            self.get_logger().debug('Zero velocity command - holding position')
        else:
            # Clear hold position for velocity control
            self._hold_position = None
            
        self._last_velocity_command = msg
        self._last_command_time = time.time()
        
        # If not flying and we get a non-zero command, initiate takeoff sequence
        if has_motion and not self._takeoff_complete and self._state == self.IDLE:
            self._state = self.TAKEOFF
            self.get_logger().info('Initiating takeoff sequence')

    def publish_velocity_setpoint(self, vx: float, vy: float, vz: float, yaw_rate: float):
        """Publish velocity setpoint."""
        msg = TrajectorySetpoint()
        msg.velocity = [float(vx), float(vy), float(vz)]
        msg.position = [float('nan'), float('nan'), float('nan')]
        msg.acceleration = [float('nan'), float('nan'), float('nan')]
        msg.yaw = float('nan')  # Let PX4 maintain current heading
        msg.yawspeed = float(yaw_rate)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    def publish_position_setpoint(self, x: float, y: float, z: float, yaw: float):
        """Publish position setpoint for takeoff/landing."""
        msg = TrajectorySetpoint()
        msg.position = [float(x), float(y), float(z)]
        msg.yaw = float(yaw)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    def publish_offboard_control_mode(self, position: bool = False, velocity: bool = True):
        """Publish offboard control mode."""
        msg = OffboardControlMode()
        msg.position = position
        msg.velocity = velocity
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_vehicle_command(self, command, **params):
        """Publish vehicle command."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.target_system = 1 + self.instance
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def arm(self):
        """Send arm command."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)

    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)

    def land(self):
        """Switch to land mode."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self._state = self.LANDING
        self._takeoff_complete = False
        self._last_velocity_command = None
        self._command_start_heading = None
        self._disarm_sent = False  # Reset disarm tracking

    def check_velocity_timeout(self) -> bool:
        """Check if velocity command has timed out."""
        if self._velocity_timeout <= 0.0:
            return False
            
        return (time.time() - self._last_command_time) > self._velocity_timeout

    def is_hovering(self) -> bool:
        """Check if vehicle is effectively hovering with no attitude rates."""
        if not self._last_velocity_command:
            return True
            
        # Check if all velocities and rates are effectively zero
        LINEAR_THRESHOLD = 0.1    # m/s
        ANGULAR_THRESHOLD = 0.1   # rad/s
        
        cmd = self._last_velocity_command
        return (
            abs(cmd.linear.x) < LINEAR_THRESHOLD and
            abs(cmd.linear.y) < LINEAR_THRESHOLD and
            abs(cmd.linear.z) < LINEAR_THRESHOLD and
            abs(cmd.angular.x) < ANGULAR_THRESHOLD and
            abs(cmd.angular.y) < ANGULAR_THRESHOLD and
            abs(cmd.angular.z) < ANGULAR_THRESHOLD
        )

    def process_velocity_command(self, msg: Twist):
        """
        Process and transform velocity commands based on reference frame.
        
        Args:
            msg: Twist message containing velocity commands
            
        Returns:
            tuple: (vx, vy, vz, yaw_rate) in NED frame
        """
        # Check for timeout or missing command
        if self.check_velocity_timeout() or msg is None:
            # Store current position for holding if not already stored
            if self._hold_position is None:
                self._hold_position = {
                    'x': self._current_pos[0],
                    'y': self._current_pos[1],
                    'z': self._current_pos[2],
                    'yaw': self._current_yaw
                }
                self.get_logger().debug('Holding position due to timeout/no command')
            return 0.0, 0.0, 0.0, 0.0

        # Check if this is effectively a zero command
        if (abs(msg.linear.x) < 0.01 and 
            abs(msg.linear.y) < 0.01 and 
            abs(msg.linear.z) < 0.01 and 
            abs(msg.angular.z) < 0.01):
            
            # Store position for holding if not already stored
            if self._hold_position is None:
                self._hold_position = {
                    'x': self._current_pos[0],
                    'y': self._current_pos[1],
                    'z': self._current_pos[2],
                    'yaw': self._current_yaw
                }
                self.get_logger().debug('Holding position at zero command')
            return 0.0, 0.0, 0.0, 0.0

        # Have non-zero command - clear hold position
        self._hold_position = None

        # Get raw velocities in input frame
        raw_vx = msg.linear.x
        raw_vy = msg.linear.y
        raw_vz = msg.linear.z
        yaw_rate = msg.angular.z

        # Frame conversions
        if self._frame_type == 'FLU':
            # Create rotation matrix from drone's current yaw
            cos_yaw = math.cos(self._current_yaw)
            sin_yaw = math.sin(self._current_yaw)
            
            # Rotate FLU velocities to NED
            # Forward in FLU becomes rotation of North/East in NED
            # Left in FLU becomes rotation of -East/North in NED
            vx = raw_vx * cos_yaw + raw_vy * sin_yaw  # North component
            vy = raw_vx * sin_yaw - raw_vy * cos_yaw   # East component
            vz = -raw_vz  # Up is negative in NED
            
        elif self._frame_type == 'FRD':
            # Create rotation matrix from drone's current yaw
            cos_yaw = math.cos(self._current_yaw)
            sin_yaw = math.sin(self._current_yaw)
            
            # Rotate FRD velocities to NED
            # Forward in FRD becomes rotation of North/East in NED
            # Right in FRD becomes rotation of East/-North in NED
            vx = raw_vx * cos_yaw - raw_vy * sin_yaw  # North component
            vy = raw_vx * sin_yaw + raw_vy * cos_yaw  # East component
            vz = raw_vz  # Down is already aligned with NED
                        
        else:  # NED
            vx = raw_vx          # North
            vy = raw_vy          # East
            vz = raw_vz          # Down
            
        # Apply velocity limits
        vx = max(-self._max_horizontal_speed,
                min(self._max_horizontal_speed, vx))
        vy = max(-self._max_horizontal_speed,
                min(self._max_horizontal_speed, vy))
        vz = max(-self._max_vertical_speed,
                min(self._max_vertical_speed, vz))
        yaw_rate = max(-self._max_yaw_rate,
                      min(self._max_yaw_rate, yaw_rate))

        return vx, vy, vz, yaw_rate

    def control_loop(self):
        """Main control loop."""
        # Check for auto-landing condition
        if self.check_auto_land_condition():
            self.land()

        # Set control mode based on whether we're holding position
        self.publish_offboard_control_mode(
            position=(self._state in [self.TAKEOFF, self.LANDING] or self._hold_position is not None),
            velocity=(self._state == self.VELOCITY and self._hold_position is None)
        )

        # Process state machine
        if self._state == self.IDLE:
            pass
        elif self._state == self.TAKEOFF:
            self.handle_takeoff_state()
        elif self._state == self.VELOCITY:
            if self._hold_position is not None:
                # Position control mode - maintain position
                self.publish_position_setpoint(
                    self._hold_position['x'],
                    self._hold_position['y'],
                    self._hold_position['z'],
                    self._hold_position['yaw']
                )
            else:
                # Velocity control mode
                vx, vy, vz, yaw_rate = self.process_velocity_command(
                    self._last_velocity_command if self._last_velocity_command else Twist()
                )
                self.publish_velocity_setpoint(vx, vy, vz, yaw_rate)
        elif self._state == self.LANDING:
            self.handle_landing_state()

    def check_auto_land_condition(self) -> bool:
        """Check if we should initiate auto-landing."""
        if self._state != self.VELOCITY:
            return False

        current_height = -self._current_pos[2]
        
        # Start timing when we detect low altitude
        if current_height < self._land_height_threshold:
            if self._land_start_time is None:
                self._land_start_time = time.time()
            elif (time.time() - self._land_start_time) > self._hover_timeout:
                self.get_logger().info(
                    f'Auto-landing triggered: Low altitude ({current_height:.2f}m) '
                    f'for {self._hover_timeout:.1f}s'
                )
                return True
        else:
            self._land_start_time = None
            
        return False

    def disarm(self):
        """Send disarm command."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Sending disarm command...')

    def handle_landing_state(self):
        """Handle landing sequence."""
        current_height = -self._current_pos[2]
        
        if current_height > -0.1:  # Basically on the ground
            if self._armed and not self._disarm_sent:
                self.disarm()
                self._disarm_sent = True
            elif not self._armed:
                # Reset all states to allow new takeoff
                self._state = self.IDLE
                self._takeoff_complete = False
                self._last_velocity_command = None
                self._command_start_heading = None
                self._land_start_time = None
                self._disarm_sent = False
                if hasattr(self, '_takeoff_start_pos'):
                    delattr(self, '_takeoff_start_pos')
                self.get_logger().info('Landing complete - ready for new commands')
            else:
                # Still waiting for disarm to take effect
                self.get_logger().debug('Waiting for disarm to complete...')

    def handle_takeoff_state(self):
        """
        Handle the takeoff sequence state.
        
        This function:
        1. Waits for valid position data
        2. Stores initial takeoff position
        3. Ensures vehicle is armed and in offboard mode
        4. Commands vertical motion to target height
        5. Monitors progress and handles timeout
        """
        if not self._position_valid:
            current_time = self.get_clock().now()
            if not hasattr(self, '_last_warning_time'):
                self._last_warning_time = current_time
            elif (current_time - self._last_warning_time).nanoseconds > 5e9:
                self.get_logger().warn('Waiting for valid position data before takeoff...')
                self._last_warning_time = current_time
            return

        # Store initial position when starting takeoff
        if not hasattr(self, '_takeoff_start_pos'):
            self._takeoff_start_pos = self._current_pos.copy()
            self._takeoff_start_yaw = self._current_yaw
            self._takeoff_start_time = time.time()  # Add takeoff start time
            self.get_logger().info(
                f'Starting takeoff from position: '
                f'[{self._takeoff_start_pos[0]:.2f}, {self._takeoff_start_pos[1]:.2f}, {self._takeoff_start_pos[2]:.2f}]'
            )

        # Send offboard and arming commands continuously until activated
        if not self._armed or not self._offboard_mode:
            # Send setpoints needed before offboard mode
            trajectory_msg = TrajectorySetpoint()
            trajectory_msg.position = [
                self._current_pos[0],  # Stay at current position
                self._current_pos[1],
                self._current_pos[2]
            ]
            trajectory_msg.yaw = self._current_yaw
            trajectory_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            self.trajectory_setpoint_publisher.publish(trajectory_msg)

            # Send both commands
            if not self._armed:
                self.arm()
                self.get_logger().info('Sending arm command...')
            if not self._offboard_mode:
                self.engage_offboard_mode()
                self.get_logger().info('Engaging offboard mode...')
            return

        # Once armed and in offboard mode, execute takeoff
        if not self._takeoff_complete:
            current_height = -self._current_pos[2]  # Convert NED to height
            target_height = self._takeoff_height
            height_error = abs(current_height - target_height)
            
            # Command position setpoint for takeoff
            trajectory_msg = TrajectorySetpoint()
            trajectory_msg.position = [
                self._takeoff_start_pos[0],  # Stay at initial X
                self._takeoff_start_pos[1],  # Stay at initial Y
                -target_height              # Target height (negative for NED)
            ]
            trajectory_msg.yaw = self._takeoff_start_yaw
            trajectory_msg.velocity = [0.0, 0.0, -0.5]  # Add upward velocity
            trajectory_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            self.trajectory_setpoint_publisher.publish(trajectory_msg)
            
            # Monitor vertical progress
            if height_error < 0.15:  # Within 15cm of target
                self._takeoff_complete = True
                self._state = self.VELOCITY
                self.get_logger().info(
                    f'Takeoff complete at height: {current_height:.2f}m'
                )
            else:
                # Progress logging
                current_time = self.get_clock().now()
                if not hasattr(self, '_last_progress_time'):
                    self._last_progress_time = current_time
                elif (current_time - self._last_progress_time).nanoseconds > 2e9:
                    self.get_logger().info(
                        f'Ascending... height: {current_height:.2f}m / {target_height:.2f}m'
                    )
                    self._last_progress_time = current_time
                    
                # Add timeout check
                if hasattr(self, '_takeoff_start_time') and (time.time() - self._takeoff_start_time) > 30.0:
                    self.get_logger().warn('Takeoff timeout reached, transitioning to velocity control')
                    self._takeoff_complete = True
                    self._state = self.VELOCITY

def main(args=None) -> None:
    """
    Main function to initialize and run the velocity control node.
    
    Args:
        args: Command line arguments (passed to rclpy.init())
    """
    print('Starting velocity-based offboard control node...')
    rclpy.init(args=args)
    
    try:
        node = OffboardControlVel()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
