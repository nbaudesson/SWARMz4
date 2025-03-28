"""
PX4 Offboard Control in NED (North-East-Down) Frame

This node implements position control for PX4-based drones using both an Action Server
and Topic interfaces. The Action Server provides detailed feedback and timeout-based
failure detection, while the Topic interface maintains backward compatibility.

Features:
- Action Server and Topic interfaces for position control
- Automatic timeout calculation based on distance and max velocity
- Position feedback during movement
- Automatic cancellation of previous goals
- Position control in NED frame
- Automatic takeoff sequence
- Position holding without drift
- Auto-landing after timeout
- Multi-vehicle support

Requirements:
- ROS2 Humble or newer
- PX4 Autopilot
- px4_msgs package
- px4_controllers_interfaces package with PointYaw.msg and GotoPosition.action

Parameters:
    spawn_x (float): X coordinate of drone's spawn position in simulation
    spawn_y (float): Y coordinate of drone's spawn position in simulationw
    spawn_z (float): Z coordinate of drone's spawn position in simulation (negative up, and drone are supposed span on the ground, at 0)
    spawn_yaw (float): Initial yaw angle of drone in simulation
    takeoff_height (float, default: 1.0): Height for takeoff in meters
    hover_timeout (float, default: 10.0): Time to wait before auto-landing
    land_height_threshold (float, default: 0.3): Maximum height for landing

Usage Examples:

1. Launch the controller:
   ```bash
   ros2 run offboard_control_py offboard_control_ned --ros-args -r __ns:=/px4_1 -p spawn_x:=1.0 -p spawn_y:=0.0
   ```

2. Using Action Server (Recommended):
   ```bash
   # Send goal and monitor progress
   ros2 action send_goal /px4_1/goto_position px4_controllers_interfaces/action/GotoPosition "{target: {position: {x: 5.0, y: 0.0, z: -2.0}, yaw: 0.0}}" --feedback

   # Send goal and wait for result
   ros2 action send_goal -w /px4_1/goto_position px4_controllers_interfaces/action/GotoPosition "{target: {position: {x: 5.0, y: 0.0, z: -2.0}, yaw: 0.0}}"

   # Cancel current goal
   ros2 action cancel /px4_1/goto_position
   ```

3. Using Topic Interface (Legacy):
   ```bash
   # Send position command
   ros2 topic pub --once /px4_1/target_pose px4_controllers_interfaces/msg/PointYaw "{position: {x: 5.0, y: 0.0, z: -2.0}, yaw: 0.0}"
   ```

Action Server Details:
- Goal: Target position and yaw in NED frame
- Feedback: Current position, distance to target, elapsed time
- Result: Success boolean (true if position reached, false if timed out)
- Timeout: Calculated based on distance and MPC_XY_VEL_MAX parameter

Notes:
    - Z is negative up (NED frame)
    - Yaw angle input is in degrees (0° = North, 90° = East, 180° = South, 270° = West) (works reversed with negative values)
    - Multiple drones supported via different namespaces (px4_1, px4_2, etc.)
    - Action server cancels previous goals when new ones are received
    - Topic commands are converted to action goals internally
    - Position timeouts are adaptive based on distance and vehicle capabilities
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus
from px4_controllers_interfaces.msg import PointYaw
import time
import math
import rclpy.action
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from px4_controllers_interfaces.action import GotoPosition
import asyncio

class AsyncActionServer(ActionServer):
    """Custom action server that handles async execution properly."""
    
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._loop = None
    
    def _get_loop(self):
        if self._loop is None:
            try:
                self._loop = asyncio.get_event_loop()
            except RuntimeError:
                self._loop = asyncio.new_event_loop()
                asyncio.set_event_loop(self._loop)
        return self._loop

    def execute_goal(self, goal_handle):
        """Execute goal with proper async handling."""
        loop = self._get_loop()
        return loop.run_until_complete(self._execute_async(goal_handle))
        
    async def _execute_async(self, goal_handle):
        try:
            return await self._execute_callback(goal_handle)
        except Exception as e:
            self.get_logger().error(f'Error in goal execution: {str(e)}')
            if goal_handle.is_active:
                goal_handle.abort()
            return None

class OffboardControlNED(Node):
    """Node for controlling a vehicle in offboard mode using NED coordinates."""

    # Control states
    IDLE = 0        # Waiting for commands
    TAKEOFF = 1     # Executing takeoff sequence
    NAVIGATING = 2  # Moving to target position
    HOLDING = 3     # Maintaining last position

    def __init__(self) -> None:
        super().__init__('offboard_control_ned')
        
        # Action server configuration
        self._action_server = AsyncActionServer(
            self,
            GotoPosition,
            'goto_position',
            execute_callback=self.execute_goal,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup()
        )

        # Flight parameters
        self._mpc_xy_vel_max = 12.0  # Maximum horizontal velocity in m/s, used for timeout calculations
        self._current_goal_handle = None  # Stores current action goal, used in execute_goal
        self._timeout_margin = 3  # Multiplier for timeout calculation (200% extra time)
        self._min_timeout = 20.0  # Minimum timeout duration in seconds

        # Launch parameters (set via command line or launch file)
        try:
            # Spawn position in simulation (used for position offsetting)
            self.declare_parameter('spawn_x', 0.0)  # X coordinate relative to world origin
            self.declare_parameter('spawn_y', 0.0)  # Y coordinate relative to world origin
            self.declare_parameter('spawn_z', 0.0)  # Z coordinate relative to world origin #Useless, as we are supposed to spawn on the ground
            # self.declare_parameter('spawn_yaw', 0.0)  # Initial yaw angle in degrees
            
            # Flight behavior parameters
            self.declare_parameter('takeoff_height', 1.0)  # Height for initial takeoff in meters
            self.declare_parameter('hover_timeout', 10.0)  # Time to hover at target before auto-landing
            self.declare_parameter('land_height_threshold', 0.4)  # Height threshold for landing detection
        except rclpy.exceptions.ParameterAlreadyDeclaredException:
            pass  # Parameters might be declared by parent class or launch file

        # Store spawn position for coordinate transformation
        self.spawn_position = [
            self.get_parameter('spawn_x').value,  # Used to transform between world and local coordinates
            self.get_parameter('spawn_y').value,
            self.get_parameter('spawn_z').value
        ]
        
        # self.spawn_yaw = self.get_parameter('spawn_yaw').value  # Used for yaw angle transformations
        self._takeoff_height = self.get_parameter('takeoff_height').value  # Used in takeoff sequence
        self._hover_timeout = self.get_parameter('hover_timeout').value  # Used in handle_navigation_state
        self._land_height_threshold = self.get_parameter('land_height_threshold').value  # Used in landing detection

        # Initialization flags and timeouts
        self._fcu_params_ready = False  # Set when FCU parameters are received
        self._position_valid = False    # Set when valid position data is received
        self._preflight_checks_complete = False  # Set when all pre-flight checks pass
        self._initialization_timeout = 30.0  # Maximum time to wait for initialization (seconds)
        self._initialization_start = None  # Timestamp for initialization start

        # State variables
        self._state = self.IDLE  # Current flight state (IDLE, TAKEOFF, NAVIGATING, HOLDING)
        self._current_pos = [0.0, 0.0, 0.0]  # Current position in local frame
        self._current_yaw = 0.0  # Current yaw angle in radians
        self._target = None  # Target position and yaw
        self._last_target_time = 0.0  # Time when last target was reached
        self._armed = False  # Vehicle arm state
        self._in_air = False  # True when vehicle is flying
        self._takeoff_complete = False  # Set after successful takeoff
        self._offboard_mode = False  # True when in offboard control mode
        self._last_reached_position = None  # Last successfully reached position

        # Multi-vehicle support
        self.node_namespace = self.get_namespace()  # Used for multi-drone topics
        self.instance = int(self.node_namespace.split('_')[-1]) if '_' in self.node_namespace else 0

        # Position tracking parameters
        self._initial_position = None  # Initial position after arming, used as reference
        self._target_reached = False  # True when current target is reached
        self._position_threshold = 0.15  # Distance threshold for position reached (meters)
        self._landing_in_progress = False  # Set during landing sequence
        self._landing_complete = False  # Set when landing is finished
        self._landing_start_time = 0.0  # Time when landing started
        self._debug_counter = 0  # Used to reduce log frequency

        # Command rate limiting
        self._last_command_time = self.get_clock().now()  # Last command timestamp
        self._command_interval = 2.0  # Minimum time between commands (seconds)
        self._print_interval = 50  # Log message frequency control

        # Timeout parameters
        self._takeoff_timeout = 30.0  # Maximum time for takeoff sequence

        # Setup publishers and subscribers
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.setup_publishers(qos_profile)
        self.setup_subscribers(qos_profile)
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info(f'NED Controller initialized at spawn position: {self.spawn_position}')
        
        # Add command rate limiting
        self._last_command_time = self.get_clock().now()
        self._command_interval = 2.0  # Minimum seconds between commands
        self._debug_counter = 0
        self._print_interval = 50  # Print status every 50 iterations
        

    def setup_publishers(self, qos_profile):
        """Setup all publishers with correct topics and QoS."""
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, f'{self.node_namespace}/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, f'{self.node_namespace}/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, f'{self.node_namespace}/fmu/in/vehicle_command', qos_profile)

    def setup_subscribers(self, qos_profile):
        """Setup all subscribers with correct topics and QoS."""
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, f'{self.node_namespace}/fmu/out/vehicle_local_position',
            self.vehicle_local_position_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, f'{self.node_namespace}/fmu/out/vehicle_status',
            self.vehicle_status_callback, qos_profile)
        self.point_yaw_subscriber = self.create_subscription(
            PointYaw, f'{self.node_namespace}/target_pose',
            self.point_yaw_callback, 10)

    def vehicle_local_position_callback(self, msg):
        """Update current position relative to spawn point."""
        # Store raw local position
        self._current_pos = [msg.x, msg.y, msg.z]
        self._current_yaw = msg.heading  # Raw heading from vehicle
        self._in_air = msg.z <= -0.3  # Consider in air if above 30cm

        # Mark position as valid when we get good data
        if msg.xy_valid and msg.z_valid and msg.heading != float('nan'):
            self._position_valid = True
            
            # Store initial position even if not armed yet - this helps with simulation issues
            if self._initial_position is None and self._position_valid:
                self._initial_position = [msg.x, msg.y, msg.z]
                self._initial_yaw = msg.heading  # Store initial yaw for reference
                self.get_logger().info(
                    f'Initial position set to: {self._initial_position}, '
                    f'Initial yaw: {math.degrees(self._initial_yaw):.1f}°'
                )
            
        # Start initialization timeout tracking
        if not self._initialization_start:
            self._initialization_start = self.get_clock().now()

    def vehicle_status_callback(self, msg):
        """Handle vehicle status updates."""
        prev_armed = self._armed
        
        self._armed = msg.arming_state == VehicleStatus.ARMING_STATE_ARMED
        self._offboard_mode = msg.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
        
        # Reset states when disarmed, but preserve target
        if prev_armed and not self._armed:
            self._landing_in_progress = False
            self._landing_complete = False
            self._last_reached_position = None
            self._target_reached = False
            self.get_logger().info('Vehicle disarmed - resetting states')
            
            # Try to rearm if we still have a target
            if self._target is not None:
                self.get_logger().info('Attempting to rearm and continue mission')
                self.arm()
                self.engage_offboard_mode()

    def point_yaw_callback(self, msg):
        """Convert topic commands to action goals."""
        # Create action client if not exists
        if not hasattr(self, '_action_client'):
            self._action_client = rclpy.action.ActionClient(
                self, 
                GotoPosition, 
                'goto_position'
            )
            
        # Wait for server
        if not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('Action server not available')
            return
            
        # Create and send goal
        goal_msg = GotoPosition.Goal()
        goal_msg.target = msg
        
        # Send goal async
        self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=lambda fb: None  # Ignore feedback for topic interface
        )

    def control_loop(self):
        """Main control loop."""
        # Don't process if landing is complete
        if self._landing_complete:
            return

        # Check if landing has timed out (30 seconds)
        if self._landing_in_progress and time.time() - self._landing_start_time > 30.0:
            self.get_logger().warn('Landing sequence timed out - resetting controller')
            self._landing_in_progress = False
            self._state = self.IDLE
            return

        # Regular control loop
        self.publish_offboard_control_mode()

        # Check for auto-landing condition - moved here from navigation state
        if self._target_reached and not self._landing_in_progress:
            time_hovering = time.time() - self._last_target_time
            current_height = abs(self._current_pos[2])
            
            # Check if we should land (only if close to ground and hover timeout reached)
            if time_hovering > self._hover_timeout and current_height <= self._land_height_threshold:
                self.get_logger().info(
                    f'Auto-landing condition met: '
                    f'hover time={time_hovering:.1f}s, '
                    f'height={current_height:.1f}m'
                )
                self.initiate_landing()

        # State machine
        if self._state == self.IDLE:
            self.handle_idle_state()
        elif self._state == self.TAKEOFF:
            self.handle_takeoff_state()
        elif self._state == self.NAVIGATING:
            self.handle_navigation_state()
        elif self._state == self.HOLDING:
            self.handle_holding_state()

    def handle_navigation_state(self):
        """Handle navigation to target."""
        if self._landing_in_progress:
            return

        if self._target is None:
            self._state = self.HOLDING
            self._last_reached_position = self._current_pos.copy() if self._current_pos else None
            return

        # Publish setpoint
        self.publish_position_setpoint(
            self._target['x'],
            self._target['y'],
            self._target['z'],
            self._target['yaw']
        )

        # Check if position is reached
        current_target_reached = self.check_position_reached()
        
        # Only update target reached status and timer on initial reaching of target
        if current_target_reached and not self._target_reached:
            self._target_reached = True
            self._last_target_time = time.time()
            self.get_logger().info('Target position reached')

        # Reset target reached status if we've moved significantly from target
        elif self._target_reached and not current_target_reached:
            if not self.check_position_reached(threshold_multiplier=2.0):
                self._target_reached = False
                self.get_logger().info('Lost target position')

    def handle_idle_state(self):
        """Handle idle state - wait for commands."""
        # Only try to arm and engage offboard mode if we have a target and valid position
        if self._target is not None and self._position_valid:
            current_time = self.get_clock().now()
            time_since_last_command = (current_time - self._last_command_time).nanoseconds / 1e9
            
            # Only send commands every _command_interval seconds
            if time_since_last_command >= self._command_interval:
                if not self._armed:
                    self.arm()
                if not self._offboard_mode:
                    self.engage_offboard_mode()
                self._last_command_time = current_time
        elif self._target is not None and not self._position_valid:
            self.get_logger().warn('Waiting for valid position data before arming')

    def handle_takeoff_state(self):
        """Handle takeoff sequence."""
        if not self._armed:
            self.arm()
            return
            
        if not self._offboard_mode:
            self.engage_offboard_mode()
            # Publish current position setpoint before engaging offboard
            self.publish_position_setpoint(
                self._current_pos[0],
                self._current_pos[1],
                self._current_pos[2],
                self._current_yaw
            )
            return

        if not self._takeoff_complete and self._initial_position:
            # Takeoff from initial position
            takeoff_setpoint = {
                'x': self._initial_position[0],
                'y': self._initial_position[1],
                'z': -(abs(self._takeoff_height)),  # Ensure negative for upward motion
                'yaw': self._current_yaw
            }
            
            self.publish_position_setpoint(
                takeoff_setpoint['x'],
                takeoff_setpoint['y'],
                takeoff_setpoint['z'],
                takeoff_setpoint['yaw']
            )
            
            # Check if takeoff height reached
            if abs(self._current_pos[2] + self._takeoff_height) < 0.2:  # Within 20cm
                self._takeoff_complete = True
                self._state = self.NAVIGATING
                self.get_logger().info('Takeoff complete, proceeding to target')

    def handle_holding_state(self):
        """Handle position holding."""
        if self._target is not None:
            self._state = self.NAVIGATING
            return

        # Hold at last reached position instead of current position
        if self._last_reached_position:
            self.publish_position_setpoint(
                self._last_reached_position['x'],
                self._last_reached_position['y'],
                self._last_reached_position['z'],
                self._last_reached_position['yaw']
            )
        else:
            # Fallback if no last position (shouldn't happen)
            self.get_logger().warn('No last position available for holding')
            self._state = self.IDLE

    def calculate_distance(self, pos1, pos2) -> float:
        """
        Calculate 3D Euclidean distance between two positions.
        
        Args:
            pos1: First position [x, y, z]
            pos2: Second position [x, y, z]
            
        Returns:
            float: Distance in meters
        """
        dx = pos1[0] - pos2[0]
        dy = pos1[1] - pos2[1]
        dz = pos1[2] - pos2[2]
        return math.sqrt(dx*dx + dy*dy + dz*dz)

    def calculate_distance_to_target(self) -> float:
        """Calculate current distance to target position."""
        if not self._target:
            return float('inf')
            
        target_pos = [self._target['x'], self._target['y'], self._target['z']]
        return self.calculate_distance(self._current_pos, target_pos)

    def check_position_reached(self, threshold_multiplier=1.0):
        """Check if current target position is reached."""
        if self._target is None or self._initial_position is None:
            return False
        
        distance = self.calculate_distance_to_target()
        return distance < (self._position_threshold * threshold_multiplier)

    def calculate_timeout(self, target_position) -> float:
        """Calculate timeout based on distance and velocity."""
        if not self._current_pos:
            return float('inf')
            
        # Calculate distance to target in NED frame
        target_ned = [
            target_position.x - self.spawn_position[0],
            target_position.y - self.spawn_position[1],
            target_position.z - self.spawn_position[2]
        ]
        distance = self.calculate_distance(self._current_pos, target_ned)
        
        # Calculate expected time with safety margin
        velocity = self._mpc_xy_vel_max * 0.5 # Reduced for acceleration/deceleration
        # When the QGC says the drone is moving at 12m/s, it is actually moving at 4m/s (calculating with distance over time)   
        base_time = distance / velocity
        timeout = max(self._min_timeout, base_time * self._timeout_margin)
        
        self.get_logger().info(
            f'Timeout calculation: distance={distance:.1f}m, '
            f'velocity={velocity:.1f}m/s, timeout={timeout:.1f}s'
        )
        
        return timeout

    def initiate_landing(self):
        """Initiate landing sequence."""
        if self._landing_in_progress:
            return

        self._landing_in_progress = True
        self._landing_start_time = time.time()
        self.get_logger().info('Initiating landing sequence')
        
        # First move to safe landing height
        landing_target = {
            'x': self._current_pos[0],
            'y': self._current_pos[1],
            'z': -0.5,  # Safe height before final landing
            'yaw': self._current_yaw
        }
        
        # Publish landing position
        self.publish_position_setpoint(
            landing_target['x'],
            landing_target['y'],
            landing_target['z'],
            landing_target['yaw']
        )
        
        # Wait briefly to start moving to landing position
        self.get_logger().info('Moving to safe landing height')
        time.sleep(2.0)
        
        # Send land command
        self.land()
        self._state = self.IDLE
        self._target = None
        self._last_reached_position = None
        self._target_reached = False
        self.get_logger().info('Landing command sent')

    def publish_offboard_control_mode(self):
        """Publish offboard control mode."""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_position_setpoint(self, x: float, y: float, z: float, yaw: float):
        """Publish position setpoint."""
        msg = TrajectorySetpoint()
        msg.position = [float(x), float(y), float(z)]
        msg.yaw = float(yaw)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    def publish_vehicle_command(self, command, **params):
        """Publish vehicle command."""
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

    def arm(self):
        """Send arm command."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self._debug_counter += 1
        if self._debug_counter % self._print_interval == 0:
            self.get_logger().info('Arm command sent')

    def disarm(self):
        """Send disarm command."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')

    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        # Need to send setpoints before and after mode switch
        self.publish_offboard_control_mode()
        
        # If we don't have current position, use zeros
        current_pos = self._current_pos if self._current_pos else [0.0, 0.0, 0.0]
        current_yaw = self._current_yaw if self._current_yaw else 0.0
        
        self.publish_position_setpoint(
            current_pos[0],
            current_pos[1],
            current_pos[2],
            current_yaw
        )
        
        # Send offboard mode command
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        
        # Send another setpoint immediately after mode command
        self.publish_offboard_control_mode()
        self.publish_position_setpoint(
            current_pos[0],
            current_pos[1],
            current_pos[2],
            current_yaw
        )
        
        self._debug_counter += 1
        if self._debug_counter % self._print_interval == 0:
            self.get_logger().info('Switching to offboard mode')

    def land(self):
        """Switch to land mode."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self._takeoff_complete = False
        self._target = None
        self.get_logger().info('Landing command sent')

    def goal_callback(self, goal_request):
        """
        Called when a client requests to start a new goal.
        Args:
            goal_request: Contains the requested target position and yaw
        Returns:
            GoalResponse.ACCEPT or GoalResponse.REJECT
        """
        # Log the incoming request
        self.get_logger().info(
            f'Received new goal request - '
            f'Position: [{goal_request.target.position.x:.1f}, '
            f'{goal_request.target.position.y:.1f}, '
            f'{goal_request.target.position.z:.1f}], '
            f'Yaw: {goal_request.target.yaw:.1f}°'
        )
        
        # Only reject if landing is in progress
        if self._landing_in_progress:
            self.get_logger().warn('Rejecting goal - landing in progress')
            return GoalResponse.REJECT
            
        # Accept all other goals - arming will be handled during execution
        self.get_logger().info('Goal request accepted')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """
        Called when a client requests to cancel an active goal.
        Args:
            goal_handle: The goal being cancelled
        Returns:
            CancelResponse.ACCEPT or CancelResponse.REJECT
        """
        self.get_logger().info(
            'Received cancel request for goal created at: '
            f'{goal_handle.goal_id.stamp.sec}.{goal_handle.goal_id.stamp.nanosec}'
        )
        
        # You can add conditions to reject cancellation
        if self._landing_in_progress:
            self.get_logger().warn('Rejecting cancellation - landing in progress')
            return CancelResponse.REJECT
            
        # Accept the cancellation
        self.get_logger().info('Cancel request accepted')
        return CancelResponse.ACCEPT

    def execute_goal(self, goal_handle: ServerGoalHandle):
        """Execute goal and return result."""
        try:
            self.get_logger().info('Executing goal...')
            
            # Check initialization
            if not self._position_valid:
                elapsed = (self.get_clock().now() - self._initialization_start).nanoseconds / 1e9 \
                    if self._initialization_start else 0.0
                    
                if elapsed > self._initialization_timeout:
                    self.get_logger().error('Failed to get valid position data')
                    goal_handle.abort()
                    return GotoPosition.Result(success=False)
                else:
                    self.get_logger().info('Waiting for valid position data...')
                    time.sleep(1.0)
                    return GotoPosition.Result(success=False)

            # Cancel any existing goal by setting handle to None
            if self._current_goal_handle and self._current_goal_handle.is_active:
                self._current_goal_handle.abort()
            
            self._current_goal_handle = goal_handle
            target_pose = goal_handle.request.target
            
            # Set target in NED frame, correcting for spawn yaw
            target_yaw = math.radians(target_pose.yaw)  # Convert goal yaw to radians
            
            # Store target without spawn offset for position, with spawn offset for yaw
            self._target = {
                'x': target_pose.position.x - self.spawn_position[0],
                'y': target_pose.position.y - self.spawn_position[1],
                'z': target_pose.position.z - self.spawn_position[2],
                'yaw': target_yaw  # Use absolute yaw, not relative to spawn
            }
            
            self.get_logger().info(
                f'Target set - Position: [{self._target["x"]:.1f}, {self._target["y"]:.1f}, '
                f'{self._target["z"]:.1f}], Yaw: {math.degrees(self._target["yaw"]):.1f}°'
            )
            
            # Calculate timeout
            timeout = self.calculate_timeout(target_pose.position)
            
            feedback_msg = GotoPosition.Feedback()
            start_time = self.get_clock().now()
            
            # IMPORTANT: First establish a steady stream of setpoints
            self.get_logger().info('Establishing setpoint stream before arming/offboard transition')
            
            # Send setpoints for 2 seconds (20 iterations at 100ms) before trying to arm/switch mode
            current_pos = self._current_pos.copy() if self._current_pos else [0.0, 0.0, 0.0]
            current_yaw = self._current_yaw if self._current_yaw else 0.0
            
            for i in range(20):  # Increased from 10 to 20 iterations
                self.publish_offboard_control_mode()
                self.publish_position_setpoint(
                    current_pos[0],
                    current_pos[1], 
                    current_pos[2],
                    current_yaw
                )
                time.sleep(0.1)
            
            # Now attempt arming and mode switch with more patience
            command_retries = 0
            MAX_RETRIES = 100  # Increased from 50 to 100
            
            while not (self._armed and self._offboard_mode) and command_retries < MAX_RETRIES:
                # Keep publishing setpoints during the transition process
                self.publish_offboard_control_mode()
                self.publish_position_setpoint(
                    current_pos[0],
                    current_pos[1], 
                    current_pos[2],
                    current_yaw
                )
                
                # More aggressive arming
                if not self._armed:
                    self.arm()
                    # Force-set armed status for simulation issues
                    if command_retries > 30:
                        self._armed = True
                        self.get_logger().warn('Force-setting armed status for simulation')
                
                if not self._offboard_mode:
                    self.engage_offboard_mode()
                    
                command_retries += 1
                
                # Check if arming succeeded
                if command_retries % 10 == 0:
                    self.get_logger().info(f'Waiting for arm/offboard... (try {command_retries}/{MAX_RETRIES})')
                    self.get_logger().info(f'Current state - Armed: {self._armed}, Offboard: {self._offboard_mode}')
                    
                time.sleep(0.1)
            
            # If we still couldn't arm properly, use simulation workaround
            if not self._armed:
                self._armed = True
                self.get_logger().warn('Simulation workaround: Setting armed state manually to proceed with takeoff')
            
            if not self._offboard_mode:
                self._offboard_mode = True
                self.get_logger().warn('Simulation workaround: Setting offboard mode manually to proceed with takeoff')
                
            self.get_logger().info('Vehicle armed and in offboard mode')

            # Execute takeoff
            takeoff_success = True
            if not self._takeoff_complete:
                takeoff_success = self.execute_takeoff(goal_handle)
                if not takeoff_success:
                    return GotoPosition.Result(success=False)
            
            # Now execute the actual navigation
            if takeoff_success:
                return self.execute_navigation(goal_handle, start_time, timeout, feedback_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error in execute_goal: {str(e)}')
            if goal_handle.is_active:
                goal_handle.abort()
            return GotoPosition.Result(success=False)

    def execute_takeoff(self, goal_handle: ServerGoalHandle) -> bool:
        """Execute takeoff sequence."""
        if not self._position_valid:
            self.get_logger().error('Cannot takeoff: No valid position data')
            return False
            
        self.get_logger().info('Starting takeoff sequence')
        
        # Wait for initial position with retries
        retries = 0
        MAX_RETRIES = 20
        while not self._initial_position and retries < MAX_RETRIES:
            self.get_logger().info(f'Waiting for initial position... (try {retries+1}/{MAX_RETRIES})')
            time.sleep(0.5)
            retries += 1
        
        # If still no initial position, create one from current position
        if not self._initial_position:
            self._initial_position = self._current_pos.copy() if self._current_pos else [0.0, 0.0, 0.0]
            self._initial_yaw = self._current_yaw if self._current_yaw else 0.0
            self.get_logger().warn(f'Using fallback initial position: {self._initial_position}')

        takeoff_pos = {
            'x': self._initial_position[0],
            'y': self._initial_position[1],
            'z': -(abs(self._takeoff_height)),
            'yaw': self._initial_yaw if hasattr(self, '_initial_yaw') else self._current_yaw
        }
        
        takeoff_start = self.get_clock().now()
        
        # Pre-publish current position setpoints to stabilize
        self.get_logger().info('Stabilizing at current position before takeoff')
        for i in range(20):
            self.publish_offboard_control_mode()
            self.publish_position_setpoint(
                self._initial_position[0],
                self._initial_position[1],
                self._initial_position[2],
                takeoff_pos['yaw']
            )
            time.sleep(0.1)
        
        # Keep publishing setpoint until height reached or timeout
        self.get_logger().info(f'Starting vertical takeoff to height: {self._takeoff_height}m')
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self._target = None
                return False
                
            # Publish takeoff setpoint
            self.publish_position_setpoint(
                takeoff_pos['x'],
                takeoff_pos['y'],
                takeoff_pos['z'],
                takeoff_pos['yaw']
            )
            
            # Check if takeoff height reached
            current_height = abs(self._current_pos[2])
            target_height = abs(self._takeoff_height)
            height_error = abs(current_height - target_height)
            
            if height_error < 0.2:  # Within 20cm
                self._takeoff_complete = True
                self.get_logger().info(f'Takeoff complete, reached height: {current_height:.2f}m')
                return True
                
            # Log progress periodically
            elapsed = (self.get_clock().now() - takeoff_start).nanoseconds / 1e9
            if int(elapsed) % 5 == 0:
                self.get_logger().info(f'Takeoff in progress: Current height: {current_height:.2f}m, Target: {target_height:.2f}m')
                
            # Check takeoff timeout
            if elapsed > self._takeoff_timeout:
                self.get_logger().error(f'Takeoff timed out after {self._takeoff_timeout}s')
                goal_handle.abort()
                return False
                
            time.sleep(0.1)
        return False

    def execute_navigation(self, goal_handle: ServerGoalHandle, start_time, timeout, feedback_msg):
        """Execute navigation to target."""
        self.get_logger().info('Starting navigation to target')
        
        # Reset timer after takeoff
        if self._takeoff_complete:
            start_time = self.get_clock().now()
            self.get_logger().info('Navigation timer reset after takeoff')
        
        retries = 0
        MAX_RETRIES = 3  # Maximum number of rearm attempts
        position_reached_count = 0
        STABLE_POSITION_COUNT = 5  # Number of consecutive position-reached checks
        
        while rclpy.ok():
            try:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    self._target = None
                    return GotoPosition.Result(success=False)
                
                # Handle disarming during navigation
                if not self._armed and self._target is not None:
                    if retries < MAX_RETRIES:
                        self.get_logger().info(f'Vehicle disarmed during navigation, retry {retries + 1}/{MAX_RETRIES}')
                        self.arm()
                        self.engage_offboard_mode()
                        time.sleep(1.0)
                        retries += 1
                        continue
                    else:
                        self.get_logger().error('Max retries reached, aborting goal')
                        goal_handle.abort()
                        return GotoPosition.Result(success=False)
                
                elapsed_time = (self.get_clock().now() - start_time).nanoseconds / 1e9
                
                # Verify target exists
                if self._target is None:
                    self.get_logger().error('Target lost during navigation')
                    goal_handle.abort()
                    return GotoPosition.Result(success=False)
                
                # Check timeout
                if elapsed_time > timeout:
                    self.get_logger().warn(f'Goal timed out: {elapsed_time:.1f}')
                    goal_handle.abort()
                    return GotoPosition.Result(success=False)
                
                # Publish current target
                self.publish_position_setpoint(
                    self._target['x'],
                    self._target['y'],
                    self._target['z'],
                    self._target['yaw']
                )
                
                # Check if position is reached
                if self.check_position_reached():
                    position_reached_count += 1
                    if position_reached_count >= STABLE_POSITION_COUNT:
                        self.get_logger().info('Target position reached and stable')
                        self._target_reached = True
                        self._last_target_time = time.time()
                        goal_handle.succeed()
                        return GotoPosition.Result(success=True)
                else:
                    position_reached_count = 0
                
                # Update feedback
                feedback_msg.current_position.position.x = self._current_pos[0] + self.spawn_position[0]
                feedback_msg.current_position.position.y = self._current_pos[1] + self.spawn_position[1]
                feedback_msg.current_position.position.z = self._current_pos[2] + self.spawn_position[2]
                feedback_msg.current_position.yaw = math.degrees(self._current_yaw)
                feedback_msg.distance_to_target = self.calculate_distance_to_target()
                feedback_msg.time_elapsed = elapsed_time
                
                goal_handle.publish_feedback(feedback_msg)
                time.sleep(0.5)  # feedback updates
            
            except Exception as e:
                self.get_logger().error(f'Navigation error: {str(e)}')
                if self._target is None:
                    self.get_logger().error('Target lost, aborting goal')
                    goal_handle.abort()
                    return GotoPosition.Result(success=False)
                continue
        
        return GotoPosition.Result(success=False)

def main(args=None) -> None:
    print('Starting offboard control NED node...')
    rclpy.init(args=args)
    
    # Create node and executor
    node = OffboardControlNED()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
