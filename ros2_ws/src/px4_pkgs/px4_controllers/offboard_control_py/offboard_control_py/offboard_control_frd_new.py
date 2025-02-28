"""
PX4 Offboard Control in FRD (Forward-Right-Down) Frame

This node provides position control for PX4-based drones using the Forward-Right-Down (FRD)
coordinate frame, where commands are relative to the drone's current position and orientation.

Key Features:
- Commands are relative to drone's current position/orientation
- Automatic coordinate transformation from FRD to NED
- Both Action Server and Topic interfaces
- Automatic takeoff sequence
- Position holding without drift
- Auto-landing after timeout
- Multi-vehicle support through namespaces

Requirements:
- ROS2 Humble or newer
- PX4 Autopilot running on drone/simulator
- px4_msgs package
- px4_controllers_interfaces package with:
    - PointYaw.msg
    - GotoPosition.action

Parameters:
    spawn_x (float): X coordinate of drone's spawn position
    spawn_y (float): Y coordinate of drone's spawn position
    spawn_z (float): Z coordinate of drone's spawn position
    takeoff_height (float, default: 1.0): Height for takeoff in meters
    hover_timeout (float, default: 10.0): Time to wait before auto-landing
    land_height_threshold (float, default: 0.3): Maximum height for landing

Usage Examples:

1. Launch the controller:
   ```bash
   # With namespace and parameters
   ros2 run offboard_control_py offboard_control_frd --ros-args -r __ns:=/px4_1 -p spawn_x:=1.0 -p spawn_y:=0.0
   ```

2. Send Commands via Action Server (Recommended):
   ```bash
   # Move 5 meters forward (relative to current heading)
   ros2 action send_goal /px4_1/goto_position px4_controllers_interfaces/action/GotoPosition "{target: {position: {x: 5.0, y: 0.0, z: 0.0}, yaw: 0.0}}" --feedback

   # Move 3 meters right and down 1 meter with 90° rotation
   ros2 action send_goal /px4_1/goto_position px4_controllers_interfaces/action/GotoPosition "{target: {position: {x: 0.0, y: 3.0, z: 1.0}, yaw: 90.0}}"
   ```

3. Send Commands via Topic:
   ```bash
   # Move forward and up
   ros2 topic pub --once /px4_1/target_pose px4_controllers_interfaces/msg/PointYaw "{position: {x: 2.0, y: 0.0, z: -1.0}, yaw: 0.0}"
   ```

Coordinate System:
    - x: Forward (relative to current heading)
    - y: Right
    - z: Down (positive is downward, negative is upward)
    - yaw: Clockwise rotation in degrees (relative to current heading)

Notes:
    - All input coordinates are transformed to NED frame for execution
    - Topic commands are converted to action goals internally
    - The controller maintains position even after reaching target
    - Auto-landing triggers after hover_timeout if close to ground
    - Multiple drones supported through different namespaces
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus
from px4_controllers_interfaces.msg import PointYaw
import time
import math
import numpy as np
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

class OffboardControlFRD(Node):
    """Node for controlling a vehicle in offboard mode using FRD coordinates."""
    
    # Control states
    IDLE = 0        # Waiting for commands
    TAKEOFF = 1     # Executing takeoff sequence
    NAVIGATING = 2  # Moving to target position
    HOLDING = 3     # Maintaining last position

    def __init__(self) -> None:
        super().__init__('offboard_control_frd')
        
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
        self._mpc_xy_vel_max = 12.0
        self._current_goal_handle = None
        self._timeout_margin = 2  
        self._min_timeout = 20.0

        # Launch parameters
        try:
            self.declare_parameter('spawn_x', 0.0)
            self.declare_parameter('spawn_y', 0.0)
            self.declare_parameter('spawn_z', 0.0)
            self.declare_parameter('takeoff_height', 1.0)
            self.declare_parameter('hover_timeout', 10.0)
            self.declare_parameter('land_height_threshold', 0.4)
        except rclpy.exceptions.ParameterAlreadyDeclaredException:
            pass

        # Store spawn position for coordinate transformation
        self.spawn_position = [
            self.get_parameter('spawn_x').value,
            self.get_parameter('spawn_y').value,
            self.get_parameter('spawn_z').value
        ]
        self._takeoff_height = self.get_parameter('takeoff_height').value
        self._hover_timeout = self.get_parameter('hover_timeout').value
        self._land_height_threshold = self.get_parameter('land_height_threshold').value

        # Initialization flags and timeouts
        self._fcu_params_ready = False
        self._position_valid = False
        self._preflight_checks_complete = False
        self._initialization_timeout = 30.0
        self._initialization_start = None

        # State variables
        self._state = self.IDLE
        self._current_pos = [0.0, 0.0, 0.0]
        self._current_yaw = 0.0
        self._target = None
        self._last_target_time = 0.0
        self._armed = False
        self._in_air = False
        self._takeoff_complete = False
        self._offboard_mode = False
        self._last_reached_position = None

        # Position tracking parameters
        self._initial_position = None
        self._target_reached = False
        self._position_threshold = 0.1
        self._landing_in_progress = False
        self._landing_complete = False
        self._landing_start_time = 0.0
        self._debug_counter = 0
        self._landing_requested = False
        self._active_goal = None

        # Add multi-vehicle support before publisher setup
        self.node_namespace = self.get_namespace()  # Used for multi-drone topics
        self.instance = int(self.node_namespace.split('_')[-1]) if '_' in self.node_namespace else 0

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

        # Add FRD-specific variables
        self._reference_position = None  # Position when command received
        self._reference_yaw = None      # Yaw when command received
        
        # Add command rate limiting
        self._last_command_time = self.get_clock().now()
        self._command_interval = 2.0  # Minimum seconds between commands
        self._debug_counter = 0
        self._print_interval = 50  # Print status every 50 iterations

        # Add detailed status tracking
        self._hover_debug_interval = 15.0 # Hover status print interval time in seconds
        self._last_hover_debug = 0.0
        self._debug_level = 1  # 0=minimal, 1=normal, 2=verbose

        # Add state name mapping
        self._state_names = {
            self.IDLE: 'IDLE',
            self.TAKEOFF: 'TAKEOFF',
            self.NAVIGATING: 'NAVIGATING',
            self.HOLDING: 'HOLDING'
        }

        self.get_logger().info(
            'FRD Controller initialized. All commands will be relative to '
            'drone position/orientation when received.'
        )

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
        """Update current position."""
        self._current_pos = [msg.x, msg.y, msg.z]
        self._current_yaw = msg.heading
        self._in_air = msg.z <= -0.3

        if msg.xy_valid and msg.z_valid and msg.heading != float('nan'):
            self._position_valid = True
            
        if self._armed and self._initial_position is None and self._position_valid:
            self._initial_position = [msg.x, msg.y, msg.z]
            self._initial_yaw = msg.heading
            
        if not self._initialization_start:
            self._initialization_start = self.get_clock().now()

    def vehicle_status_callback(self, msg):
        """Handle vehicle status updates."""
        prev_armed = self._armed
        self._armed = msg.arming_state == VehicleStatus.ARMING_STATE_ARMED
        self._offboard_mode = msg.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
        
        if prev_armed and not self._armed:
            self._landing_in_progress = False
            self._landing_complete = False
            self._last_reached_position = None
            self._target_reached = False
            if self._target is not None:
                self.arm()
                self.engage_offboard_mode()

    def control_loop(self):
        """Main control loop."""
        if self._landing_complete:
            return

        if self._landing_in_progress and time.time() - self._landing_start_time > 30.0:
            self.get_logger().warn('Landing sequence timed out')
            self._landing_in_progress = False
            self._state = self.IDLE
            return

        self.publish_offboard_control_mode()

        # Update landing condition check with minimal logging
        current_time = time.time()
        if self._target_reached and not self._landing_in_progress:
            time_hovering = current_time - self._last_target_time
            current_height = abs(self._current_pos[2])
            
            # Only log status if debug level > 0 and interval passed
            if (self._debug_level > 0 and 
                current_time - self._last_hover_debug > self._hover_debug_interval):
                self._last_hover_debug = current_time
                self.get_logger().info(
                    f'Status: hovering={time_hovering:.1f}s, height={current_height:.2f}m, '
                    f'state={self._state_names[self._state]}'
                )
            
            # Check landing conditions with minimal logging
            if (time_hovering > self._hover_timeout and 
                current_height <= self._land_height_threshold and 
                self._active_goal is None):
                self._landing_requested = True
                self.initiate_landing()

        if self._state == self.IDLE:
            self.handle_idle_state()
        elif self._state == self.TAKEOFF:
            self.handle_takeoff_state()
        elif self._state == self.NAVIGATING:
            self.handle_navigation_state()
        elif self._state == self.HOLDING:
            self.handle_holding_state()

    def transform_frd_to_ned(self, frd_pos, frd_yaw):
        """
        Transform coordinates from FRD to NED frame.
        
        The transformation involves:
        1. Rotating the position vector by current yaw
        2. Adding the result to current position
        3. Adding the relative yaw to current yaw
        
        Args:
            frd_pos: List[float] - Position in FRD frame [forward, right, down]
            frd_yaw: float - Yaw angle in radians relative to current heading
            
        Returns:
            tuple[List[float], float]: (ned_position, ned_yaw)
                ned_position: [north, east, down] in global frame
                ned_yaw: absolute yaw in global frame
        
        Raises:
            ValueError: If reference position/yaw not available
        """
        if self._reference_position is None or self._reference_yaw is None:
            self.get_logger().error('No reference position/yaw available')
            return None, None
            
        # Create rotation matrix from FRD to NED
        cos_yaw = math.cos(self._reference_yaw)
        sin_yaw = math.sin(self._reference_yaw)
        R = np.array([
            [cos_yaw, -sin_yaw, 0],
            [sin_yaw, cos_yaw, 0],
            [0, 0, 1]
        ])
        
        # Transform position
        frd_vec = np.array([frd_pos[0], frd_pos[1], frd_pos[2]])
        ned_vec = R @ frd_vec
        
        # Calculate absolute position in NED
        ned_position = [
            self._reference_position[0] + ned_vec[0],
            self._reference_position[1] + ned_vec[1],
            self._reference_position[2] + ned_vec[2]
        ]
        
        # Calculate absolute yaw in NED
        ned_yaw = self._reference_yaw + frd_yaw
        
        # Normalize yaw to [-pi, pi]
        ned_yaw = math.atan2(math.sin(ned_yaw), math.cos(ned_yaw))
        
        return ned_position, ned_yaw

    def goal_callback(self, goal_request):
        """Handle new goal requests by storing current position as reference."""
        # Store current position and yaw as reference
        if not self._position_valid:
            self.get_logger().error('Cannot accept goal: No valid position data')
            return GoalResponse.REJECT
            
        self._reference_position = self._current_pos.copy()
        self._reference_yaw = self._current_yaw
        
        self.get_logger().info(
            f'New goal reference set at position: [{self._reference_position[0]:.1f}, '
            f'{self._reference_position[1]:.1f}, {self._reference_position[2]:.1f}], '
            f'yaw: {math.degrees(self._reference_yaw):.1f}°'
        )
        
        return GoalResponse.ACCEPT

    def execute_goal(self, goal_handle: ServerGoalHandle):
        """Execute goal with FRD to NED transformation."""
        try:
            self._active_goal = goal_handle
            target_pose = goal_handle.request.target
            
            # Transform FRD coordinates to NED
            frd_pos = [
                target_pose.position.x,  # Forward
                target_pose.position.y,  # Right
                target_pose.position.z   # Down
            ]
            frd_yaw = math.radians(target_pose.yaw)  # Relative yaw
            
            ned_pos, ned_yaw = self.transform_frd_to_ned(frd_pos, frd_yaw)
            if ned_pos is None:
                self.get_logger().error('Failed to transform coordinates')
                goal_handle.abort()
                self._active_goal = None
                return GotoPosition.Result(success=False)
            
            # Store transformed target
            self._target = {
                'x': ned_pos[0],
                'y': ned_pos[1],
                'z': ned_pos[2],
                'yaw': ned_yaw
            }
            
            self.get_logger().info(
                f'FRD target: [{frd_pos[0]:.1f}, {frd_pos[1]:.1f}, {frd_pos[2]:.1f}], '
                f'yaw: {math.degrees(frd_yaw):.1f}° -> '
                f'NED target: [{ned_pos[0]:.1f}, {ned_pos[1]:.1f}, {ned_pos[2]:.1f}], '
                f'yaw: {math.degrees(ned_yaw):.1f}°'
            )
            
            # Calculate timeout based on transformed NED coordinates
            timeout = self.calculate_timeout(target_pose.position)
            self.get_logger().info(f'Calculated timeout: {timeout:.1f}s')
            
            feedback_msg = GotoPosition.Feedback()
            start_time = self.get_clock().now()
            
            # First ensure vehicle is armed and in offboard mode
            command_retries = 0
            MAX_RETRIES = 50
            
            while not (self._armed and self._offboard_mode) and command_retries < MAX_RETRIES:
                if not self._armed:
                    self.arm()
                if not self._offboard_mode:
                    self.engage_offboard_mode()
                command_retries += 1
                time.sleep(0.1)
                
                if command_retries % 10 == 0:
                    self.get_logger().info(f'Waiting for arm/offboard... (try {command_retries}/{MAX_RETRIES})')
            
            # Execute takeoff if needed
            if not self._takeoff_complete:
                takeoff_success = self.execute_takeoff(goal_handle)
                if not takeoff_success:
                    self._active_goal = None
                    return GotoPosition.Result(success=False)
                    
                # Reset start time after takeoff to give full timeout for navigation
                start_time = self.get_clock().now()
                self.get_logger().info('Reset timeout counter after takeoff')
            
            # Execute navigation with transformed coordinates
            return self.execute_navigation(goal_handle, start_time, timeout, feedback_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error in execute_goal: {str(e)}')
            if goal_handle.is_active:
                goal_handle.abort()
            self._active_goal = None
            return GotoPosition.Result(success=False)

    def execute_takeoff(self, goal_handle: ServerGoalHandle) -> bool:
        """
        Execute takeoff sequence with safety checks and monitoring.
        
        Process:
        1. Verify position data validity
        2. Check initial position availability
        3. Maintain takeoff setpoint until target height reached
        4. Monitor for cancellation requests
        5. Handle timeouts
        
        Args:
            goal_handle: Current action goal handle
            
        Returns:
            bool: True if takeoff successful, False otherwise
            
        Safety:
        - Checks position validity before attempting takeoff
        - Monitors height during ascent
        - Includes timeout protection
        - Handles cancellation gracefully
        """
        if not self._position_valid:
            self.get_logger().error('Cannot takeoff: No valid position data')
            return False
            
        self.get_logger().info('Starting takeoff sequence')
        
        # Make sure we have initial position
        if not self._initial_position:
            self.get_logger().error('Cannot takeoff: No initial position set')
            return False

        takeoff_pos = {
            'x': self._current_pos[0],
            'y': self._current_pos[1],
            'z': -(abs(self._takeoff_height)),
            'yaw': self._current_yaw
        }
        
        takeoff_start = self.get_clock().now()
        
        # Keep publishing setpoint until height reached or timeout
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
            if abs(self._current_pos[2] + self._takeoff_height) < 0.2:
                self._takeoff_complete = True
                self.get_logger().info('Takeoff complete')
                return True
                
            # Check takeoff timeout (30 seconds)
            if (self.get_clock().now() - takeoff_start).nanoseconds / 1e9 > 30.0:
                self.get_logger().error('Takeoff timed out')
                goal_handle.abort()
                return False
                
            time.sleep(0.1)
        return False

    def execute_navigation(self, goal_handle: ServerGoalHandle, start_time, timeout, feedback_msg):
        """
        Execute navigation to target with position tracking and feedback.
        
        Process:
        1. Monitor position and stability
        2. Handle disarming events with retries
        3. Provide continuous feedback
        4. Check timeout conditions
        5. Verify target reaching with stability counter
        
        Args:
            goal_handle: Current action goal handle
            start_time: Navigation start time
            timeout: Maximum allowed execution time
            feedback_msg: Feedback message structure
            
        Returns:
            GotoPosition.Result: Success status with final result
            
        Features:
        - Position stability verification
        - Automatic rearm attempts
        - Continuous feedback publishing
        - Graceful error handling
        """
        self.get_logger().info('Starting navigation to transformed target')
        
        retries = 0
        MAX_RETRIES = 3
        position_reached_count = 0
        STABLE_POSITION_COUNT = 5
        
        while rclpy.ok():
            try:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    self._target = None
                    self._active_goal = None
                    return GotoPosition.Result(success=False)
                
                # Handle disarming during navigation
                if not self._armed and self._target is not None:
                    if retries < MAX_RETRIES:
                        self.get_logger().info(f'Vehicle disarmed, retry {retries + 1}/{MAX_RETRIES}')
                        self.arm()
                        self.engage_offboard_mode()
                        time.sleep(1.0)
                        retries += 1
                        continue
                    else:
                        self.get_logger().error('Max retries reached')
                        goal_handle.abort()
                        self._active_goal = None
                        return GotoPosition.Result(success=False)
                
                elapsed_time = (self.get_clock().now() - start_time).nanoseconds / 1e9
                
                if self._target is None:
                    self.get_logger().error('Target lost during navigation')
                    goal_handle.abort()
                    self._active_goal = None
                    return GotoPosition.Result(success=False)
                
                if elapsed_time > timeout:
                    self.get_logger().warn(f'Goal timed out after {elapsed_time:.1f}s')
                    goal_handle.abort()
                    self._active_goal = None
                    return GotoPosition.Result(success=False)
                
                # Publish current target in NED frame
                self.publish_position_setpoint(
                    self._target['x'],
                    self._target['y'],
                    self._target['z'],
                    self._target['yaw']
                )
                
                # Check if position is reached with stability counter
                if self.check_position_reached():
                    position_reached_count += 1
                    if position_reached_count >= STABLE_POSITION_COUNT:
                        if self._debug_level > 0:
                            self.get_logger().info(
                                f'Target reached: distance={self.calculate_distance_to_target():.2f}m'
                            )
                        self._target_reached = True
                        self._last_target_time = time.time()  # Ensure this is set
                        self._active_goal = None  # Clear active goal
                        goal_handle.succeed()
                        return GotoPosition.Result(success=True)
                else:
                    position_reached_count = 0
                
                # Update feedback (positions already in NED frame)
                feedback_msg.current_position.position.x = self._current_pos[0] + self.spawn_position[0]
                feedback_msg.current_position.position.y = self._current_pos[1] + self.spawn_position[1]
                feedback_msg.current_position.position.z = self._current_pos[2] + self.spawn_position[2]
                feedback_msg.current_position.yaw = math.degrees(self._current_yaw)
                feedback_msg.distance_to_target = self.calculate_distance_to_target()
                feedback_msg.time_elapsed = elapsed_time
                
                goal_handle.publish_feedback(feedback_msg)
                time.sleep(1.0)
            
            except Exception as e:
                self.get_logger().error(
                    f'Navigation error: {str(e)}\n'
                    f'- Current state: {self._state}\n'
                    f'- Target exists: {self._target is not None}\n'
                    f'- Position valid: {self._position_valid}'
                )
                if self._target is None:
                    goal_handle.abort()
                    self._active_goal = None
                    return GotoPosition.Result(success=False)
                continue
        
        return GotoPosition.Result(success=False)

    def cancel_callback(self, goal_handle):
        """Handle goal cancellation requests."""
        self.get_logger().info('Received cancel request')
        
        if self._landing_in_progress:
            self.get_logger().warn('Cannot cancel during landing')
            return CancelResponse.REJECT
            
        self._target = None
        return CancelResponse.ACCEPT

    def point_yaw_callback(self, msg):
        """
        Handle incoming topic commands by converting to action goals.
        
        This maintains backward compatibility with topic interface while
        using the more robust action server implementation internally.
        
        Args:
            msg: PointYaw - Target position/yaw in FRD frame
        """
        # Store current position as reference before converting to action
        if not self._position_valid:
            self.get_logger().warn('Cannot process topic command: No valid position data')
            return
            
        self._reference_position = self._current_pos.copy()
        self._reference_yaw = self._current_yaw
        
        # Create and send action goal
        if not hasattr(self, '_action_client'):
            self._action_client = rclpy.action.ActionClient(
                self, 
                GotoPosition, 
                'goto_position'
            )
            
        # Wait for action server
        if not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('Action server not available')
            return
            
        # Convert topic message to action goal
        goal_msg = GotoPosition.Goal()
        goal_msg.target = msg
        
        self.get_logger().info(
            f'Converting topic command to action goal - '
            f'FRD target: [{msg.position.x:.1f}, {msg.position.y:.1f}, {msg.position.z:.1f}], '
            f'yaw: {msg.yaw:.1f}°'
        )
        
        # Send goal async
        self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self._action_feedback_callback
        )

    def _action_feedback_callback(self, feedback_msg):
        """Handle feedback from action execution."""
        feedback = feedback_msg.feedback
        self.get_logger().debug(
            f'Position: [{feedback.current_position.position.x:.1f}, '
            f'{feedback.current_position.position.y:.1f}, '
            f'{feedback.current_position.position.z:.1f}], '
            f'Distance: {feedback.distance_to_target:.1f}m, '
            f'Time: {feedback.time_elapsed:.1f}s'
        )

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

    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        # Need to send setpoints before switching to offboard mode
        self.publish_offboard_control_mode()
        self.publish_position_setpoint(
            self._current_pos[0],
            self._current_pos[1],
            self._current_pos[2],
            self._current_yaw
        )
        
        # Send offboard mode command
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self._debug_counter += 1
        if self._debug_counter % self._print_interval == 0:
            self.get_logger().info('Switching to offboard mode')

    def land(self):
        """Switch to land mode."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self._takeoff_complete = False
        self._target = None

    def handle_idle_state(self):
        """Handle idle state - wait for commands."""
        if self._target is not None and self._position_valid:
            current_time = self.get_clock().now()
            time_since_last_command = (current_time - self._last_command_time).nanoseconds / 1e9
            
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
            self.publish_position_setpoint(
                self._current_pos[0],
                self._current_pos[1],
                self._current_pos[2],
                self._current_yaw
            )
            return

        if not self._takeoff_complete and self._initial_position:
            takeoff_setpoint = {
                'x': self._initial_position[0],
                'y': self._initial_position[1],
                'z': -(abs(self._takeoff_height)),
                'yaw': self._current_yaw
            }
            
            self.publish_position_setpoint(
                takeoff_setpoint['x'],
                takeoff_setpoint['y'],
                takeoff_setpoint['z'],
                takeoff_setpoint['yaw']
            )
            
            if abs(self._current_pos[2] + self._takeoff_height) < 0.2:
                self._takeoff_complete = True
                self._state = self.NAVIGATING
                self.get_logger().info('Takeoff complete, proceeding to target')

    def handle_navigation_state(self):
        """
        Handle navigation state with position tracking and stability checks.
        
        Process:
        1. Check for landing in progress
        2. Verify target existence
        3. Publish current setpoint
        4. Monitor target reaching
        5. Handle position loss events
        
        State Transitions:
        - NAVIGATING -> HOLDING: When target is None
        - Position reached -> Position lost: When distance exceeds threshold
        
        Features:
        - Continuous position monitoring
        - Stable position verification
        - Position loss detection
        - Automatic state transitions
        """
        if self._landing_in_progress:
            return

        if self._target is None:
            self._state = self.HOLDING
            self._last_reached_position = self._current_pos.copy() if self._current_pos else None
            return

        self.publish_position_setpoint(
            self._target['x'],
            self._target['y'],
            self._target['z'],
            self._target['yaw']
        )

        current_target_reached = self.check_position_reached()
        
        if current_target_reached and not self._target_reached:
            self._target_reached = True
            self._last_target_time = time.time()
            self.get_logger().info('Target position reached')
        elif self._target_reached and not current_target_reached:
            if not self.check_position_reached(threshold_multiplier=2.0):
                self._target_reached = False
                self.get_logger().info('Lost target position')

    def handle_holding_state(self):
        """Handle position holding."""
        if self._target is not None:
            self._state = self.NAVIGATING
            return

        if self._last_reached_position:
            self.publish_position_setpoint(
                self._last_reached_position[0],
                self._last_reached_position[1],
                self._last_reached_position[2],
                self._current_yaw
            )
        else:
            self.get_logger().warn('No last position available for holding')
            self._state = self.IDLE

    def calculate_distance(self, pos1, pos2) -> float:
        """
        Calculate 3D Euclidean distance between two positions.
        
        Args:
            pos1 (List[float]): First position [x, y, z]
            pos2 (List[float]): Second position [x, y, z]
            
        Returns:
            float: Distance in meters
            
        Math:
            distance = √((x₂-x₁)² + (y₂-y₁)² + (z₂-z₁)²)
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
        
        # More conservative timeout calculation
        velocity = self._mpc_xy_vel_max * 0.7
        base_time = distance / velocity
        timeout = max(self._min_timeout, base_time * self._timeout_margin)
        
        self.get_logger().info(
            f'Timeout calculation: distance={distance:.1f}m, '
            f'velocity={velocity:.1f}m/s, timeout={timeout:.1f}s'
        )
        
        return timeout

    def initiate_landing(self):
        """
        Initiate and execute landing sequence with safety checks.
        
        Process:
        1. Set landing flags and timer
        2. Move to safe landing height
        3. Wait for stabilization
        4. Execute land command
        5. Reset navigation states
        
        Safety Features:
        - Safe height approach before landing
        - Position stabilization period
        - State cleanup after landing
        - Landing progress monitoring
        """
        if self._landing_in_progress:
            return

        # Abort any active goal before landing
        if self._active_goal and self._active_goal.is_active:
            self._active_goal.abort()
            self._active_goal = None

        self._landing_in_progress = True
        self._landing_start_time = time.time()
        self.get_logger().info('Starting landing sequence')
        
        landing_target = {
            'x': self._current_pos[0],
            'y': self._current_pos[1],
            'z': -0.5,
            'yaw': self._current_yaw
        }
        
        self.publish_position_setpoint(
            landing_target['x'],
            landing_target['y'],
            landing_target['z'],
            landing_target['yaw']
        )
        
        time.sleep(2.0)
        self.land()
        self._state = self.IDLE
        self._target = None
        self._last_reached_position = None
        self._target_reached = False
        
        if self._debug_level > 0:
            self.get_logger().info('Landing command sent')

def main(args=None) -> None:
    """
    Main function to initialize and run the FRD controller node.
    
    Sets up:
    - Multi-threaded executor
    - Signal handling
    - Clean shutdown
    """
    print('Starting offboard control FRD node...')
    rclpy.init(args=args)
    
    node = OffboardControlFRD()
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
