"""
PX4 Offboard Control

This node implements comprehensive control for PX4-based drones using both an Action Server
and Topic interfaces. It provides multiple coordinate system support and various control modes.

Features:
- Multi-coordinate system support (NED, FRD, FLU)
- Multiple control modes (position, velocity)
- Action Server with detailed feedback and timeout-based failure handling
- Topic interface for simplicity and backward compatibility
- Automatic takeoff sequence with multiple fallback methods
- Automatic landing with safety checks
- Adaptive position control with timeout calculation
- Hovering capability with automatic landing after timeout
- Multi-vehicle support through namespacing

Coordinate Systems:
- NED (North-East-Down): Standard aviation coordinate system
- FRD (Forward-Right-Down): Body-fixed frame relative to vehicle orientation
- FLU (Forward-Left-Up): Body-fixed frame commonly used in robotics

Control Modes:
- Position: Direct control of vehicle position and heading
- Velocity: Control of vehicle velocity vector and yaw rate

Parameters:
- coordinate_system (str, default: 'NED'): Coordinate system used for external commands
- offboard_mode (str, default: 'position'): Control mode (position, velocity)
- spawn_x, spawn_y (float, default: 0.0): Spawn position for coordinate transformations
- takeoff_height (float, default: 1.0): Height for automatic takeoff in meters
- hover_timeout (float, default: 10.0): Time to hover before auto-landing
- land_height_threshold (float, default: 0.4): Height threshold for landing detection
- mpc_xy_vel_max (float, default: 12.0): Maximum horizontal velocity for position control
- command_velocity_timeout (float, default: 2.0): Timeout for velocity commands

Usage Examples:

1. Launch the controller:
   ```bash
   ros2 run offboard_control_py offboard_control_px4 --ros-args -r __ns:=/px4_0 -p spawn_x:=0.0 -p spawn_y:=0.0 -p coordinate_system:=NED

2. Using Action Server (Recommended):
   ```bash
   # Send goal and monitor progress
   ros2 action send_goal /px4_0/goto_position px4_controllers_interfaces/action/GotoPosition "{target: {position: {x: 5.0, y: 0.0, z: -2.0}, yaw: 0.0}}" --feedback

   # Send goal and wait for result
   ros2 action send_goal -w /px4_0/goto_position px4_controllers_interfaces/action/GotoPosition "{target: {position: {x: 5.0, y: 0.0, z: -2.0}, yaw: 0.0}}"

   # Cancel current goal
   ros2 action cancel /px4_0/goto_position
   ```

3. Using Topic Interface (Position Mode or Velocity Mode):
   ```bash
   # Send position command
   ros2 topic pub --once /px4_0/target_pose px4_controllers_interfaces/msg/PointYaw "{position: {x: 5.0, y: 0.0, z: -2.0}, yaw: 0.0}"
   ```

Action Server Details:
- Goal: Target position and yaw in the frame of your choosing. Make sure to correclty set the same frame in the parameter coordinate_system of this node
- Feedback: Current position, distance to target, elapsed time
- Result: Success boolean (true if position reached, false if timed out)
- Timeout: Calculated based on distance and MPC_XY_VEL_MAX parameter

Notes:
    - Z is negative up (NED frame)
    - Yaw angle input is in degrees (0° = North, 90° = East, 180° = South, 270° = West) (works reversed with negative values)
    - Multiple drones supported via different namespaces (px4_1, px4_2, etc.)
    - Action server cancels previous goals when new ones are received or when a cancelation request is made
    - Action server feedback is published every 2 seconds
    - Action server result is published when the goal is reached or when the timeout is reached
    - Topic commands are dosentn't use the action server, thus it doesn't return feedback nor response
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus
from px4_controllers_interfaces.msg import PointYaw
import time
import math
import numpy as np
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from action_msgs.msg import GoalStatus
from px4_controllers_interfaces.action import GotoPosition
import threading, asyncio

class Pose:
    """Represents a pose with x, y, z coordinates and yaw."""
    def __init__(self, x: float = 0.0, y: float = 0.0, z: float = 0.0, yaw: float = 0.0):
        self.x = x
        self.y = y
        self.z = z
        self.yaw = yaw

    def to_dict(self):
        """Convert the Pose object to a dictionary."""
        return {'x': self.x, 'y': self.y, 'z': self.z, 'yaw': self.yaw}
    
    def to_list(self):
        """Convert the Pose object to a list [x, y, z, yaw]."""
        return [self.x, self.y, self.z, self.yaw]
    
    def yaw_in_degrees(self):
        return math.degrees(self.yaw)

    @classmethod
    def from_dict(cls, data: dict):
        """Create a Pose object from a dictionary."""
        return cls(x=data.get('x', 0.0), y=data.get('y', 0.0), z=data.get('z', 0.0), yaw=data.get('yaw', 0.0))
    
    @classmethod
    def from_msg(cls, msg):
        """Create a Pose object from a PointYaw.msg ROS message."""
        return cls(
            x=msg.position.x,
            y=msg.position.y,
            z=msg.position.z,
            yaw=msg.yaw
        )

    def __repr__(self):
        """String representation of the Pose object."""
        return f"Pose(x={self.x}, y={self.y}, z={self.z}, yaw={self.yaw})"

class PX4OffboardControl(Node):
    """Node for controlling a vehicle in offboard mode using NED, global NED, FRD, FLU coordinates, position and velocity requests."""

    # Control states
    IDLE = 0        # Waiting for commands
    TAKEOFF = 1     # Executing takeoff sequence
    LANDING = 2   # Executing landing sequence
    NAVIGATING = 3  # Moving to target position
    HOVER = 4     # Maintaining last position

    def __init__(self) -> None:
        # Fix the parameter_overrides format
        super().__init__(
            'offboard_control_px4', 
            parameter_overrides=[
                rclpy.parameter.Parameter(
                    'use_sim_time', 
                    rclpy.parameter.Parameter.Type.BOOL, 
                    True
                )
            ]
        )

        #######################
        ### ROS2 PARAMETERS ###
        #######################

        # Coordinate system parameters
        self.declare_parameter('coordinate_system', 'NED')  # Coordinate system used (NED, FRD or FLU)
        self.declare_parameter('offboard_mode', 'position')  # Offboard modes used (position, velocity). Acceleration, attitude, body_rate are not implemnented yet

        # Spawn position in simulation (used for position offsetting)
        self.declare_parameter('spawn_x', 0.0)  # X coordinate relative to world origin
        self.declare_parameter('spawn_y', 0.0)  # Y coordinate relative to world origin
        
        # Flight behavior parameters
        self.declare_parameter('takeoff_height', 2.0)  # Height for initial takeoff in meters
        self.declare_parameter('hover_timeout', 10.0)  # Time to hover at target before auto-landing
        self.declare_parameter('land_height_threshold', 0.4)  # Height threshold for landing detection
        self.declare_parameter('mpc_xy_vel_max', 12.0)  # Max horizontal speed for position control
        self.declare_parameter('command_velocity_timeout', 2.0)  # Max horizontal speed for position control

        # Store world spawn position for coordinate transformation between world and local coordinates
        self._spawn_position = Pose(x=self.get_parameter('spawn_x').value, y=self.get_parameter('spawn_y').value)

        self._takeoff_height = self.get_parameter('takeoff_height').value  # Used in takeoff sequence
        self._hover_timeout = self.get_parameter('hover_timeout').value  # Used in handle_navigation_state
        self._land_height_threshold = self.get_parameter('land_height_threshold').value  # Used in landing detection
        self._command_velocity_timeout = self.get_parameter('command_velocity_timeout').value  # Timeout for velocity command in seconds

        #################################
        ### Initialization parameters ###
        #################################

        # Flight parameters
        self._timeout_margin = 3  # Multiplier for timeout calculation (200% extra time)
        self._min_timeout = 20.0  # Minimum timeout duration in seconds

        # State variables
        self._state = self.IDLE  # Current flight state (IDLE, TAKEOFF, NAVIGATING, HOVER)
        self.vehicle_local_position = VehicleLocalPosition() # Current local position data
        self.vehicle_status = VehicleStatus()  # Current vehicle status
        self._armed = False  # Vehicle arm state
        self._pre_flight_checks_passed = False # True if pre-flight checks are passed
        self._is_offboard = False  # True when in offboard control mode

        # Multi-vehicle support
        self.node_namespace = self.get_namespace()  # Used for multi-drone topics
        self.node_namespace = '' if self.node_namespace == '/' else self.node_namespace
        self._instance = int(self.node_namespace.split('_')[-1]) if '_' in self.node_namespace else 0

        # Position tracking parameters
        self._position_valid = False    # Set when valid position data is received
        self._current_pos = Pose() # Current position in local frame, yaw in rad
        self._target = False  # True when a new target is received
        self._target_pos = Pose() # Target position in user frame, yaw in degrees
        self._local_target_pos = Pose() # Target position in local frame, yaw in rad
        self._last_reached_position = Pose()  # Last successfully reached position
        self._position_threshold = 0.15  # Distance threshold for position reached (meters)
        self._takeoff_case = False  # Which takeoff case are we in auto, setpoint or fail
        self._takeoff_complete = False  # Set when takeoff is finished, also serves as a flag to check if the drone is supposed to be in the air
        self._takeoff_start_time = 0.0  # Time when takeoff started
        self._landing_in_progress = False  # Set during landing sequence
        self._navigation_in_progress = False  # Set during navigation sequence
        self._navigation_start_time = 0.0  # Time when navigation started
        self._navigation_timeout = 0.0  # Timeout for navigation (seconds)
        self._hovering_in_progress = False  # Set during hovering sequence
        self._low_altitude_hovering_timer = None # Time when low altitude hovering started  in secs

        self._last_log_message = ""

        # Add setpoint tracking for command loop
        self._setpoint_to_run = None  # Will store current setpoint to be executed in a Pose

        # Setup publishers and subscribers
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.setup_publishers(qos_profile)
        self.setup_subscribers(qos_profile)
        
        # Create separate timers for control logic and command publishing
        self.control_timer = self.create_timer(float(1/4), self.control_loop)  # Main control logic at 4Hz
        self.command_timer = self.create_timer(float(1/10), self.command_loop)  # Command publishing at 3Hz (should be > 2Hz)
        
        self.get_logger().info(f'NED Controller initialized at spawn position: {self._spawn_position}')

    ##################
    ### ROS2 SETUP ###
    ##################

        # Integrated Action Server configuration
        self._action_server = ActionServer(
            self,
            GotoPosition,                     # The action type
            'goto_position',                  # The action name
            self.execute_callback,            # The callback for executing goals
            goal_callback=self.goal_callback, # The callback for accepting/rejecting new goals
            cancel_callback=self.cancel_callback # The callback for handling goal cancellations
        )
        # Variables to keep track of the current goal and its result
        self._current_goal_handle = None
        self._goal_result = None
        # Event to signal when the goal processing is complete
        self._goal_event = threading.Event()

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
            VehicleStatus, f'{self.node_namespace}/fmu/out/vehicle_status_v1',
            self.vehicle_status_callback, qos_profile)
        self.point_yaw_subscriber = self.create_subscription(
            PointYaw, f'{self.node_namespace}/target_pose',
            self.point_yaw_callback, 10)

    ################################
    ### TOPIC CALLBACK FUNCTIONS ###
    ################################

    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback function for vehicle_local_position topic subscriber."""
        self.vehicle_local_position = vehicle_local_position
        # Store position in PX4 coordinate system which is local NED
        self._current_pos = Pose(x=vehicle_local_position.x, y=vehicle_local_position.y, z=vehicle_local_position.z, yaw=vehicle_local_position.heading)
        # Mark position as valid when we get good data to validate for reaching goal
        self._position_valid = vehicle_local_position.xy_valid and vehicle_local_position.z_valid and vehicle_local_position.heading != float('nan')
    
    def vehicle_status_callback(self, vehicle_status):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = vehicle_status
        self._armed = vehicle_status.arming_state == VehicleStatus.ARMING_STATE_ARMED
        self._is_offboard = vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
        self._auto_takeoff_in_progress = vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_TAKEOFF
        self._landing_in_progress = vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LAND
        self._pre_flight_checks_passed = vehicle_status.pre_flight_checks_pass

    def point_yaw_callback(self, msg):
        """
        Convert topic commands to Pose object and set target.
        WARNING: This method does not use the action server, thus it does not return feedback nor response.
        """

        # Check if a goal is currently beign handled by the action server
        # If so, cancel it

        # If the offboard_mode is position, we need to convert the message to a Pose object
        if (self.get_parameter('offboard_mode').value == 'position' 
            or self.get_parameter('offboard_mode').value == 'position_velocity'):
            # Convert the PointYaw message to a Pose object
            self._target_pos = Pose.from_msg(msg)
            self._target = True
        elif (self.get_parameter('offboard_mode').value == 'acceleration' 
              or self.get_parameter('offboard_mode').value == 'attitude'
              or self.get_parameter('offboard_mode').value == 'body_rate'):
            self.log(f"Offboard mode {self.get_parameter('offboard_mode')} not implemented yet", level='error') 
        else:
            self.log(f"Offboard mode {self.get_parameter('offboard_mode')} not recognized", level='error')  

    def command_loop(self):
        """
        Loop that publishes trajectory setpoint commands in offboard mode.
        In offboard mode the rate of commands must be higher than 2Hz.
        This loop doesn't interact with targets, only with setpoints converted to this drone's reference frame.
        """

        # Skip if no target
        if self._setpoint_to_run is None:
            return
        
        # In IDLE state, we don't want to publish anything
        if self._state == self.IDLE:
            self._setpoint_to_run = None
            return
        
        ### Safety Checks ###
        # Check if the drone is in a valid state to send setpoints

        #  Valid position check a.k.a. is the ekf working
        if self._position_valid is False:
            # If the drone is not in IDLE and the position is not valid, it means the drone has lost it and we need to land
            self.log('Position estimation failed and drone is supposedly flying - EMERGENCY Landing', level='error')
            # If the drone has no valid position, we can't even use the LANDING state.
            self._state = self.IDLE
            # The sleep will outlast the frequency of the command loop, I hope changing the state first will avoid looping over this case
            self.land()
            return
  
        # Arming check
        if not self._armed:
            self.log('Cannot send commands - vehicle not armed', level='warn')
            self.arm()
            # If not armed, while in the air (Which should not be possible) we need to land
            if self._current_pos.z < 0.0 and self._takeoff_complete and (self._state == self.NAVIGATING or self._state == self.HOVER):
                self.log('Not armed and detected in the air - EMERGENCY Landing', level='error')
                # In this scenario, the drone is not able to run tracjectory setpoints
                # So it needs to use the land command to get to the ground immediatly and restart the arming sequence of the IDLE state
                self._setpoint_to_run = None 
                self._landing_in_progress = True
                self._state = self.LANDING
            return
    
        # # Offboard mode check
        if self._is_offboard is False:
            self.offboard_mode()
            # return
        
            # # Takeoff and landing states are the only states where we can be in an other navigation state than offboard mode (in this code)
            # # If not in offboard mode, while in the air we need to land
            # if self._current_pos.z < 0.0 and self._takeoff_complete and (self._state == self.NAVIGATING): # or self._state == self.HOVER
            #     self.log('Not in offboard mode and detected in the air - EMERGENCY Landing', level='error')
            #     # In this scenario, the drone is not able to run tracjectory setpoints
            #     # So it needs to use the land command to get to the ground immediatly and restart the arming sequence of the IDLE state
            #     self._landing_in_progress = True
            #     self._state = self.LANDING
            #     # Aboort the goal if it exists
            #     if self._current_goal_handle is not None:
            #         result = GotoPosition.Result()
            #         result.success = False
            #         self._goal_result = result  # Store the result before aborting
            #         # Abort the goal instead of succeed for semantic correctness
            #         self._current_goal_handle.abort()
            #         self._goal_event.set()  # Signal that the goal is completed

        # Always publish control mode message
        # If the drone is in taking off state, landing state, or hovering state, we HAVE TO use the position mode
        if self._state == self.TAKEOFF or self._state == self.LANDING or self._state == self.HOVER:
            self.publish_offboard_control_mode("position")
            # Publish the trajectory setpoint in position mode
            self.publish_position_setpoint(x=self._setpoint_to_run.x, y=self._setpoint_to_run.y, z=self._setpoint_to_run.z, yaw=self._setpoint_to_run.yaw)
        else:
            if self.get_parameter('offboard_mode').value == 'position':
                # If the drone is in navigating state, we HAVE TO use the position mode
                self.publish_offboard_control_mode("position")
                # Publish the trajectory setpoint in position mode
                self.publish_position_setpoint(x=self._setpoint_to_run.x, y=self._setpoint_to_run.y, z=self._setpoint_to_run.z, yaw=self._setpoint_to_run.yaw)
            elif self.get_parameter('offboard_mode').value == 'velocity':
                self.publish_offboard_control_mode(self.get_parameter('offboard_mode').value)
                # Publish the trajectory setpoint in velocity mode
                self.publish_velocity_setpoint(vx=self._setpoint_to_run.x, vy=self._setpoint_to_run.y, vz=self._setpoint_to_run.z, yawspeed=self._setpoint_to_run.yaw)

    def control_loop(self):
        """
        Main control loop for state machine and decision making (runs at 4Hz).
        It transforms targets into setponts for the command loop to publish.
        """

        # State machine - this now updates setpoint_to_run instead of publishing directly
        # Or send PX4 commands for arming, disarming, takeoff and landing

        #####################################################################
        # ONLY HANDLE_*_STATE FUNCTIONS SHOULD UPDATE SELF._SETPOINT_TO_RUN #
        #####################################################################

        if self._state == self.IDLE:
            self.handle_idle_state()
        elif self._state == self.TAKEOFF:
            self.handle_takeoff_state()  
        elif self._state == self.LANDING:
            self.handle_landing_state()
        elif self._state == self.NAVIGATING:
            self.handle_navigation_state()
        elif self._state == self.HOVER:
            self.handle_hover_state()

    ################################
    ### STATE HANDLING FUNCTIONS ###
    ################################

    # 0: IDLE
    def handle_idle_state(self):
        """Handle idle state - wait for commands."""
        # Only try to arm and engage offboard mode if we have a target and valid position and pre-flight checks passed
        if self._position_valid is False:
            self.log('Waiting for valid position data', level='info')
            return
        
        if self._target is False :
            self.log('Waiting for target', level='info')
            return
        
        if self._pre_flight_checks_passed is False:
            self.log('Pre-flight checks not passed! Check PX4 terminal.', level='warn')
            if self._is_offboard:
                self.hold_mode()
            return
        
        # if self._armed is False :
        #     self.arm()
        #     return

        # If we get to this point, we're armed and we have somewhere to go to
        # Make the drone takeoff before sending the target as _setpoint_to_run
        self._state = self.TAKEOFF
        ### TODO: TAKEOFF WITH COMMAND DOEAN'T WORK YET, SKIP IT FOR NOW
        self._takeoff_case = 'auto'   

    # 1: TAKEOFF
    def handle_takeoff_state(self):
        match self._takeoff_case:
            case 'auto':
                self.handle_command_takeoff_state()
            case 'setpoint':
                self.handle_setpoint_takeoff_state()
            case 'fail':
                # Stop _setpoint_to_run, log error, start landing and cancel action if there is one
                self.log('Takeoff failed, switching to LANDING state', level='error')
                self._state = self.LANDING
                self._setpoint_to_run = None
                self.log('Takeoff timeout reached', level='warn')
                self.log(f'Current position: {self._current_pos}', level='warn')
                self.log(f'Setpoint to run: {self._setpoint_to_run}', level='warn')
                self.log(self.vehicle_status, level='debug')
                self._takeoff_complete = False
                if self._current_goal_handle is not None:
                    # Abort the goal instead of succeed for semantic correctness
                    self._current_goal_handle.abort()
                    self._goal_event.set()  # Signal that the goal is completed


    def handle_command_takeoff_state(self):
        """
        Handle takeoff state using the auto takeoff command.
        """

        # 1. Cancel any set goal if it exists
        if self._setpoint_to_run is not None:
            self._setpoint_to_run = None
            self.log('Setpoint to run cancelled', level='info')

        # 2. If auto takeoff sequence hasn't started yet, start it. And start the takeoff timer
        if self._auto_takeoff_in_progress is False:
            self.take_off_mode()
            # self.take_off()
            self.arm()
            self._auto_takeoff_in_progress = True
            self._takeoff_start_time = self.get_clock().now()

        # 3. Check if the drone is at the right height (with the same objective threshold as navigating), if so pass to NAVIGATING state
        # auto takeoff make it go a bit higher than the takeoff height, so we need to modify the threshold
        if math.isclose(self._current_pos.z, -self._takeoff_height, abs_tol=self._position_threshold+0.3):
            self._state = self.NAVIGATING
            self._takeoff_complete = True
            self.log('Takeoff complete, transitioning to NAVIGATING state', level='info')
            ### TODO: To test if necessary. Setting setpoint to run just in case
            self.offboard_mode()
            self._setpoint_to_run = self._current_pos
            ###
            return
        
        # 4. Check if timeout has been reached, if so raise error to try setpoint takeoff
        if self._auto_takeoff_in_progress is True and self._takeoff_complete is False:
            try:
                if ((self.get_clock().now() - self._takeoff_start_time).nanoseconds / 1e9) > self._min_timeout:
                    self._takeoff_case = 'setpoint'
                    self.log('Takeoff timeout reached, switching to setpoint takeoff', level='warn')
            except TypeError:
                self._takeoff_start_time = self.get_clock().now()

    def handle_setpoint_takeoff_state(self):
        """
        Handle takeoff state in the state machine control loop, using setpoint method to takeoff.
        """
        if self._armed is False :
                self.arm()
        if self._is_offboard is False:
            self.offboard_mode()

        # 1. if _setpoint_to_run is none, give the drone a setpoint with the drone's current position and yaw and and make a setpoint with z = takeoff_height and start timeout counting for takeoff
        if self._setpoint_to_run is None:
            self._setpoint_to_run = Pose(
                x=self._current_pos.x,
                y=self._current_pos.y,
                z=-self._takeoff_height,
                yaw=self._current_pos.yaw
            )
            # Start the takeoff timer
            self._takeoff_start_time = self.get_clock().now()
            self.log('Setpoint takeoff initiated', level='info')
            return

        # 2. check if the drone is at the right height (with the same objective threshold as navigating), if so pass to NAVIGATING state
        if math.isclose(self._current_pos.z, -self._takeoff_height, abs_tol=self._position_threshold):
            self._state = self.NAVIGATING
            self.log('Takeoff complete, transitioning to NAVIGATING state', level='info')
            self._takeoff_complete = True
            return 

        # 3. check if timeout has been reached, if so give logs about the drones status and setpoint to help debug the sequence,
        #   then pass to landing mode, then return something to the action server to says the the action failed
        if ((self.get_clock().now() - self._takeoff_start_time).nanoseconds / 1e9) > self._min_timeout:
            self._takeoff_case = 'fail'

    # 2: LANDING
    def handle_landing_state(self):
        """
        Handle landing state in the state machine control loop.
        Stop the drone where it is at (to brake its speed) without checking if the goal has been reached (just check if horizontal speed is low enough),
        and then use the PX4 land command to land the drone.
        """

        # In this case we are hovering in loiter/hold state and the offboard mode method will not work.        
        if self._is_offboard is False:
            self.land()

        # 1. If the drone is not already landing, set the setpoint to the current position to make the drone stop and start the landing process
        if self._landing_in_progress is False:
            self._setpoint_to_run = Pose(
                x=self._current_pos.x,
                y=self._current_pos.y,
                z=-self._land_height_threshold,
                yaw=self._current_pos.yaw
                )
            self._landing_in_progress = True
        else:
            # 2. Check if horizontal speed is slow enough to start vertical landing. Run the land command only once to avoid flooding PX4
            if abs(self.vehicle_local_position.vx) < 0.1 and abs(self.vehicle_local_position.vy) < 0.1:
                self.log('Horizontal speed low enough, landing initiated', level='info')
                self._setpoint_to_run = None
                self.land()
                self.log('Landing complete', level='info')
                self._landing_in_progress = False
                self._state = self.IDLE
                return

    # 3: NAVIGATING
    def handle_navigation_state(self):
        """
        Handle navigation to target in the state machine control loop.
        This state is responsible for moving the drone to the target position.
        It checks if the target is reached and updates the action server accordingly.
        """

        # 1. Do we still have a target?
        if self._navigation_in_progress is False and self._target is False:
            # If not, go to HOVER mode
            self._state = self.HOVER
            return

        # 2. Are we in offboard velocity mode? If so, we are not navigating, we are just sending a velocity command
        if self.get_parameter('offboard_mode').value == 'velocity':
            # Check if there is a new target
            if self._target is True:
                self._target = False  # New targets are one time uses
                # a. We need to convert the target vector to local vector
                # Then set the _setpoint_to_run to the target position in local frame and start the command velocity timer
                self._setpoint_to_run = convert_user_velocity_to_local_velocity(self._current_pos,
                                                                                self._target_pos,
                                                                                self.get_parameter('coordinate_system').value)
                self._navigation_in_progress = True  # Start the navigation timer
                self._command_velocity_timer = self.get_clock().now()
                self.log('Velocity command initiated', level='info')
            if self._navigation_in_progress is True:
                # b. If there is no new target, we need to check if the velocity command has timed out
                if ((self.get_clock().now() - self._command_velocity_timer).nanoseconds / 1e9) > self._command_velocity_timeout:
                    # If it has, change the state to HOVER
                    self.log('Velocity command timeout reached', level='info')
                    self._target = False
                    self._navigation_in_progress = False
                    self._last_reached_position = self._current_pos
            return

        # 2. Are we in offboard position mode? If so, we are navigating, sending a position command and track the progress
        if self.get_parameter('offboard_mode').value == 'position':
            # a. Check if a new target has been received
            if self._target is True and self._navigation_in_progress is False:
                self.log(f'Going to {self._target_pos}', level='info')
                self._target = False
                #   raise the flag, start the timer and set the timeout to get there
                #   set _setpoint_to_run to the target position (from the requested coordinate system) converted to the local frame
                self._local_target_pos = convert_objective_coordinates_to_local_coordinates(self._spawn_position,
                                                                                        self._target_pos,
                                                                                        self._current_pos,
                                                                                        self.get_parameter('coordinate_system').value)
                self.log(f'Local target position: {self._local_target_pos}', level='info')
                self._navigation_timeout = self.calculate_timeout(self._current_pos.to_list()[:3], self._local_target_pos.to_list()[:3])
                # Go to the target position
                self._setpoint_to_run = self._local_target_pos
                self._navigation_in_progress = True
                self._navigation_start_time = self.get_clock().now()
                return

            # 3. Update the action server feedback with the current position, distance to target and elapsed time
            elapsed_time = (self.get_clock().now() - self._navigation_start_time).nanoseconds / 1e9
            distance_to_target = calculate_distance(self._current_pos.to_list()[:3], self._local_target_pos.to_list()[:3])
            # If there is an action running, we need to update the feedback
            if self._current_goal_handle is not None and int(elapsed_time) % 2 == 0:
                feedback_msg = GotoPosition.Feedback()
                feedback_msg.current_position.position.x = self._current_pos.x
                feedback_msg.current_position.position.y = self._current_pos.y
                feedback_msg.current_position.position.z = self._current_pos.z
                # User yaw is in degrees
                feedback_msg.current_position.yaw = self._current_pos.yaw_in_degrees()
                feedback_msg.distance_to_target = distance_to_target
                feedback_msg.time_elapsed = elapsed_time
                self._current_goal_handle.publish_feedback(feedback_msg)

            # 4. Check if the drone has timed out
            if elapsed_time > self._navigation_timeout:
                self.log('Navigation timeout reached', level='warn')
                self.log(f'Current position: {self._current_pos}', level='warn')
                self.log(f'Setpoint to run: {self._setpoint_to_run}', level='warn')
                self.log('Hovering initiated due to timeout', level='error')
                self.log(self.vehicle_status, level='debug')
                self._last_reached_position = self._current_pos
                self._navigation_in_progress = False
                if self._current_goal_handle is not None:
                    result = GotoPosition.Result()
                    result.success = False
                    self._goal_result = result  # Store the result before aborting
                    # Abort the goal instead of succeed for semantic correctness
                    self._current_goal_handle.abort()
                    self._goal_event.set()  # Signal that the goal is completed
                return
            
        # 5. Check if the drone has reached the target position
            if distance_to_target <= self._position_threshold:
                self.log('Target position reached', level='info')
                self.log(f'Current position: {self._current_pos}', level='info')
                self._last_reached_position = self._local_target_pos
                self._navigation_in_progress = False
                # Check if there is still a valid goal handle before succeeding
                if self._current_goal_handle is not None:
                    result = GotoPosition.Result()
                    result.success = True
                    self._goal_result = result  # Store the result before signaling completion
                    self._current_goal_handle.succeed()
                    self._goal_event.set()  # Signal that the goal is completed
                return

    # 4 : HOVER
    def handle_hover_state(self):
        """
        Handle position holding when no target is active.
        If a new target is received, transition to NAVIGATING state.
        If no new target just maintains the last reached position.
        If current height is below the land_height_threshold, it will start a timer.
        If the timer exceeds hover_timeout and the current height is below the land_height_threshold during the whole hover_timeout,
        it will initiate landing.
        """

        # 1. If a target exists, transition to NAVIGATING state, lower the hovering flag
        if self._target:
            self._hovering_start_time = None
            self._low_altitude_hovering_timer = None
            self._hovering_in_progress = False
            self._state = self.NAVIGATING
            return

        # 2. If it does not, we set the setpoint to the last reached position
        # To avoid continoulsy changing the setpoint to the same value check for hovering flag
        if self._hovering_in_progress is False:
            self._setpoint_to_run = self._last_reached_position
            self._hovering_in_progress = True
            return
        
        # 3. If the drone is below the land height threshold and timer has not already started, we start the timer
        if -self._current_pos.z < self._land_height_threshold:
            if self._low_altitude_hovering_timer is None:
                self._low_altitude_hovering_timer = self.get_clock().now()
            else:
                # We check if the timer has timed out
                if ((self.get_clock().now() - self._low_altitude_hovering_timer).nanoseconds / 1e9) > self._hover_timeout:
                    # If it has, we start the landing sequence
                    self.log('Hovering timeout reached, initiating landing', level='warn')
                    self._state = self.LANDING
        else:
            if self._low_altitude_hovering_timer is not None:
                # If the drone is above the land height threshold, we stop the timer
                self._low_altitude_hovering_timer = None
        return


    ####################
    ### PX4 COMMANDS ###
    ####################

    def publish_offboard_control_mode(self, mode="position"):
        """Publish offboard control mode."""
        msg = OffboardControlMode()
        msg.position = (mode == "position")
        msg.velocity = (mode == "velocity")
        msg.acceleration = (mode == "acceleration")
        msg.attitude = (mode == "attitude")
        msg.body_rate = (mode == "body_rate")
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_velocity_setpoint(self, vx: float, vy: float, vz: float, yawspeed: float):
        """Publish velocity setpoint."""
        msg = TrajectorySetpoint()
        msg.velocity = [float(vx), float(vy), float(vz)]
        msg.yawspeed = float(yawspeed)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

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
        msg.target_system = 1 + self._instance
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
        self.log('Arm command sent', level='info')
        
    def disarm(self):
        """Send disarm command."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.log('Disarm command sent', level='info')

    def offboard_mode(self):
        """Switch to offboard mode."""
        # Send nav_state request for offboard mode
        # self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1., 6.)
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_SET_NAV_STATE, param1=14.0)   
        self.log('Switching to offboard mode', level='info')
    
    def hold_mode(self):
        """Switch to hold mode."""
        # Send nav_state request for hold mode
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_SET_NAV_STATE, param1=4.0)    
        self.log('Switching to hold mode', level='info')

    def take_off_mode(self):
        """Switch to takeoff mode."""
        # Send nav_state request for takeoff mode
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_SET_NAV_STATE, param1=17.0)   
        self.log('Switching to takeoff mode', level='info')

    def take_off(self):
        """Command the vehicle to take off to specified altitude"""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, param1 = 1.0, param7=self._takeoff_height) # param7 is altitude in meters
        self.log("Takeoff command send", level='info')

    def land_mode(self):
        """Switch to land mode."""
        # Send nav_state request for land mode
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_SET_NAV_STATE, param1=18.0)   
        self.log('Switching to land mode', level='info')

    def land(self):
        """Switch to land mode."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.land_mode()
        self._takeoff_complete = False
        self._target = False
        self.log('Landing command sent', level='info')

    ###############################
    ### ACTION SERVER CALLBACKS ###
    ###############################

    def goal_callback(self, goal_handle):
        """
        Accept or reject a new goal. If a goal is already being processed,
        cancel it before accepting the new one.
        """
        self.log('Received new goal', level='info')

        # Cancel the current goal if there is one
        if self._current_goal_handle is not None:
            self.log('Canceling current goal', level='info')
            # Instead of directly calling canceled(), set a flag and notify
            self._goal_event.set()  # Signal the control loop to stop processing
            # Set result to None to indicate cancellation
            self._goal_result = None
            # The current goal will be properly handled in execute_callback
            
        # Accept the new goal
        self._goal_result = None
        self._goal_event.clear()  # Reset the event to wait for the new goal's completion

        # Convert the goal to the target position
        self._target_pos = Pose.from_msg(goal_handle.target)
        self._target = True

        return GoalResponse.ACCEPT

    async def execute_callback(self, goal_handle):
            """
            Execute the goal by waiting for the control loop to signal completion.
            This method uses asynchronous waiting to avoid blocking the processor.
            """
            # Store the new goal handle
            previous_goal_handle = self._current_goal_handle
            self._current_goal_handle = goal_handle

            # If there was a previous goal, mark it as aborted
            if previous_goal_handle is not None and previous_goal_handle != goal_handle:
                # Check if the previous goal is still active before aborting
                if previous_goal_handle.status == GoalStatus.STATUS_EXECUTING:
                    previous_goal_handle.abort()

            # Wait for the control loop to signal that the goal is complete
            while not self._goal_event.is_set():
                # Poll to check if the goal is complete
                self._goal_event.wait(timeout=1.0)

            # Handle result based on how the goal completed
            if self._goal_result is not None:
                self.log('Goal completed with result: ' + str(self._goal_result.success))
                # The goal was already marked as succeeded or aborted in the control loop
                return self._goal_result
            else:
                self.log('Goal was canceled')
                # Only cancel if the goal is not already in a terminal state
                if goal_handle.status != GoalStatus.STATUS_ABORTED and goal_handle.status != GoalStatus.STATUS_SUCCEEDED:
                    goal_handle.canceled()
                return GotoPosition.Result()

    def cancel_callback(self, cancel_request):
        """
        Handle cancel requests. If a goal is currently being processed,
        cancel it and signal the control loop to stop processing.
        """
        self.log('Received cancel request')

        if self._current_goal_handle is not None:
            self.log('Canceling current goal')
            self._current_goal_handle.canceled()
            self._current_goal_handle = None
            self._goal_event.set()  # Signal the control loop to stop processing the current goal
            return CancelResponse.ACCEPT

        return CancelResponse.REJECT 

    ###############
    ###  TOOLS  ###
    ###############

    def log(self, message, level='info'):
        """Log messages only if it is new"""
        if str(message) != str(self._last_log_message):
            if level == 'debug':
                self.get_logger().debug(message)
            elif level == 'info':
                self.get_logger().info(message)
            elif level == 'warn':
                self.get_logger().warn(message)
            elif level == 'error':
                self.get_logger().error(message)
            self._last_log_message = message

    def calculate_timeout(self, current_position: list, target_position: list, expected_mean_speed=None) -> float:
        """Calculate timeout based on distance and velocity."""
        self.log('calculate timeout current position {} target position {}'.format(current_position, target_position), level='info')
        # Calculate distance to target using the existing method
        distance = calculate_distance(current_position, target_position)
        # Calculate expected time with safety margin
        if expected_mean_speed is None:
            expected_mean_speed = self.get_parameter('mpc_xy_vel_max').value
        velocity = expected_mean_speed * 0.7  # Reduced for acceleration/deceleration
        base_time = distance / velocity
        timeout = max(self._min_timeout, base_time * self._timeout_margin)

        self.log(f'Timeout calculation: distance={distance:.1f}m', level='info')
        self.log(f'velocity={velocity:.1f}m/s, timeout={timeout:.1f}s', level='info')
        return timeout

#################
### Functions ###
#################

def convert_local_NED_to_global_NED(spawn_position: Pose, current_position: Pose):
    """
    Vehicle local position returns NED local coordinates, convert current position and yaw to selected coordinate system.

    Convert vehicle local position and yaw to the specified coordinate system.

    Parameters:
    - spawn_position: Pose object with x, y, z, yaw representing the spawn position.
    - current_position: Pose object with x, y, z, yaw representing the current position in local NED.

    Returns:
    - Pose object with x, y, z, yaw
    """

    # Convert to local NED coordinates to GLOBAL NED coordinates
    x = current_position.x + spawn_position.x
    y = current_position.y + spawn_position.y
    z = current_position.z
    yaw = current_position.yaw

    return Pose(x=x, y=y, z=z, yaw=yaw)

def convert_objective_coordinates_to_local_coordinates(spawn_position: Pose, goal_position: Pose, current_position: Pose, system='NED'):
    """
    Trajectory setpoint takes NED local coordinates as input.
    Convert goal position and yaw from selected coordinate system to local NED.
    Parameters:
    - spawn_position: Pose object with x, y, z, yaw representing the spawn position.
    - goal_position: Pose object with x, y, z, yaw representing the goal position in the selected coordinate system, yaw in degrees.
    - current_position: Pose object with x, y, z, yaw  representing the current position in local NED.
    - system: String representing the target coordinate system ('NED', 'local_NED', 'FRD', 'FLU').
    Returns:
    - Pose object with x, y, z, yaw
    """

    # Convert goal_yaw from degrees to radians
    goal_yaw_rad = math.radians(goal_position.yaw)
    # Ensure yaw is within -pi to pi range
    goal_yaw_rad = (goal_yaw_rad + math.pi) % (2 * math.pi) - math.pi

    # Convert to GLOBAL NED coordinates to local NED coordinates
    if system == 'NED':
        x = goal_position.x - spawn_position.x
        y = goal_position.y - spawn_position.y
        z = goal_position.z
        yaw = goal_yaw_rad

    # Just use regular PX4 local NED coordinates
    elif system == 'local_NED':
        x = current_position.x
        y = current_position.y
        z = current_position.z
        yaw = current_position.yaw

    # Convert to LOCAL FRD coordinates to local NED coordinates
    elif system == 'FRD':
        # Calculate the rotation matrix for converting FRD to NED
        rotation_matrix = np.array([
            [math.cos(current_position.yaw), -math.sin(current_position.yaw)],
            [math.sin(current_position.yaw), math.cos(current_position.yaw)]
        ])
        ned_offset_horizontal = rotation_matrix @ np.array([goal_position.x, goal_position.y])

        x = current_position.x + ned_offset_horizontal.x,
        y = current_position.y + ned_offset_horizontal.y,
        z = current_position.z + goal_position.z
        yaw = current_position.yaw + goal_yaw_rad

    # Convert to LOCAL FLU coordinates to local NEDcoordinates
    elif system == 'FLU':
        # Calculate the rotation matrix for converting NED to FLU
        rotation_matrix = np.array([
            [math.cos(current_position.yaw), math.sin(current_position.yaw)],
            [-math.sin(current_position.yaw), math.cos(current_position.yaw)]
        ])
        # Rotate horizontal (forward, left) vector
        ned_offset_horizontal = rotation_matrix @ np.array([goal_position.x, -goal_position.y])
        ned_offset_vertical = -goal_position.z  # Up in FLU is Down in NED

        x = current_position.x + ned_offset_horizontal[0],
        y = current_position.y + ned_offset_horizontal[1],
        z = current_position.z + ned_offset_vertical
        yaw = current_position.yaw + goal_yaw_rad
    
    else:
        raise ValueError(f"Unsupported coordinate system: {system}")

    return Pose(x=x, y=y, z=z, yaw=yaw)

def convert_user_velocity_to_local_velocity(current_position: Pose, goal_velocity: Pose, system='FLU'):
    """
    Convert goal velocity from selected coordinate system to local NED.
    Parameters:
    - current_position: Pose object with yaw representing the current orientation in local NED.
    - goal_velocity: Pose object with x, y, z, and yaw representing the goal velocity in the selected coordinate system.
                     (linear x,y,z for velocity, and yaw for yawspeed)
    - system: String representing the target coordinate system ('NED', 'local_NED', 'FRD', 'FLU').
    Returns:
    - Pose object representing the velocity in local NED.
    """
    # No conversion needed for local NED
    if system == 'local_NED':
        return goal_velocity
    
    # For global NED, FRD, and FLU, the conversion is a rotation
    elif system in ('NED', 'FRD', 'FLU'):
        goal_velocity_np = np.array(goal_velocity.to_list()[:3])  # Extract x, y, z velocity

        # Rotation matrix for NED to local NED (inverse of local to NED)
        yaw = current_position.yaw
        rotation_matrix = np.array([
            [math.cos(yaw), math.sin(yaw), 0],
            [-math.sin(yaw), math.cos(yaw), 0],
            [0, 0, 1]
        ])

        if system == 'FRD':
            # FRD to NED rotation
            frd_to_ned_rotation = np.array([
                [1, 0, 0],
                [0, 1, 0],
                [0, 0, 1]
            ])
            goal_velocity_np = frd_to_ned_rotation @ goal_velocity_np

        elif system == 'FLU':
            # FLU to NED rotation
            flu_to_ned_rotation = np.array([
                [0, 1, 0],
                [1, 0, 0],
                [0, 0, -1]
            ])
            goal_velocity_np = flu_to_ned_rotation @ goal_velocity_np

        # Rotate to local NED
        local_velocity_np = rotation_matrix @ goal_velocity_np
        
        # Create and return a Pose object
        return Pose(x=local_velocity_np[0], y=local_velocity_np[1], z=local_velocity_np[2], yaw=goal_velocity.yaw)

    else:
        raise ValueError(f"Unsupported coordinate system: {system}")

def calculate_distance(pos1: list, pos2: list) -> float:
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

def main(args=None) -> None:
    print('Starting offboard control NED node...')
    rclpy.init(args=args)
    
    # Create node and executor
    node = PX4OffboardControl()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
