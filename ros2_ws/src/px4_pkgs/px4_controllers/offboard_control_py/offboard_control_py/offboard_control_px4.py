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
- horizontal_speed (float, default: 12.0): Maximum horizontal velocity for position control
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

4.ROS2 Commands for Testing Velocity Control Mode.
    # To start the controller with velocity mode enabled:
    ```bash
    ros2 run offboard_control_py offboard_control_px4 --ros-args -r __ns:=/px4_0 -p offboard_mode:=velocity
    ```

    # Commands to send velocity messages:
    ```bash
    # Move forward at 1.0 m/s (in FRD frame)
    ros2 topic pub --once /px4_0/target_pose px4_controllers_interfaces/msg/PointYaw "{position: {x: 1.0, y: 0.0, z: 0.0}, yaw: 0.0}"

    # Move right at 0.5 m/s (in FRD frame)
    ros2 topic pub --once /px4_0/target_pose px4_controllers_interfaces/msg/PointYaw "{position: {x: 0.0, y: 0.5, z: 0.0}, yaw: 0.0}"

    # Move up at 0.3 m/s (in FRD frame)
    ros2 topic pub --once /px4_0/target_pose px4_controllers_interfaces/msg/PointYaw "{position: {x: 0.0, y: 0.0, z: -0.3}, yaw: 0.0}"

    # Turn at 15 degrees/second to the right (in FRD frame)
    ros2 topic pub --once /px4_0/target_pose px4_controllers_interfaces/msg/PointYaw "{position: {x: 0.0, y: 0.0, z: 0.0}, yaw: 15.0}"

    # Combined motion (forward + right + turning) (in FRD frame)
    ros2 topic pub --once /px4_0/target_pose px4_controllers_interfaces/msg/PointYaw "{position: {x: 0.5, y: 0.5, z: 0.0}, yaw: 10.0}"
    ```
    Note: The velocity commands will time out after the command_velocity_timeout parameter (default 2.0 seconds).

Action Server Details:
- Goal: Target position and yaw in the frame of your choosing. Make sure to correclty set the same frame in the parameter coordinate_system of this node
- Feedback: Current position, distance to target, elapsed time
- Result: Success boolean (true if position reached, false if timed out)
- Timeout: Calculated based on distance and horizontal_speed (MPC_XY_VEL_MAX parameter)

Notes:
    - Z is negative up (NED frame)
    - Yaw angle input is in degrees (0° = North, 90° = East, 180° = South, 270° = West) (works reversed with negative values)
    - Multiple drones supported via different namespaces (px4_1, px4_2, etc.)
    - Action server cancels previous goals when new ones are received or when a cancelation request is made
    - Action server feedback is published every 2 seconds
    - Action server result is published when the goal is reached or when the timeout is reached
    - Topic commands are dosentn't use the action server, thus it doesn't return feedback nor response
    - This node requires a mavros node to be running for the drone. You can launch it with:
    ```bash
    ros2 launch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14550" namespace:=/px4_0
    ```
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus
from px4_controllers_interfaces.msg import PointYaw
import math
import numpy as np
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from action_msgs.msg import GoalStatus
from px4_controllers_interfaces.action import GotoPosition
import threading
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv import SetParameters
from px4_controllers_interfaces.srv import SetVelocities
import os, yaml
from geometry_msgs.msg import Twist

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
    
    def yaw_in_radians(self):
        return math.radians(self.yaw)

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
        self._coordinate_system = self.get_parameter('coordinate_system').value # Get initial value

        self.declare_parameter('offboard_mode', 'position')  # Offboard modes used (position, velocity). Acceleration, attitude, body_rate are not implemnented yet

        # Spawn position in simulation (used for position offsetting)
        self.declare_parameter('spawn_x', -1.0)  # X coordinate relative to world origin, -1.0 as special value to trigger lookup
        self.declare_parameter('spawn_y', -1.0)  # Y coordinate relative to world origin, -1.0 as special value to trigger lookup
        
        # Spawn position YAML file path
        self.declare_parameter('spawn_position_file', 'spawn_position.yaml')  # Path to spawn position YAML file
        
        # Flight behavior parameters
        self.declare_parameter('takeoff_height', 5.0)  # Height for initial takeoff in meters
        self.declare_parameter('hover_timeout', 10.0)  # Time to hover at target before auto-landing
        self.declare_parameter('land_height_threshold', 0.4)  # Height threshold for landing detection
        self.declare_parameter('horizontal_speed', 12.0)  # Max horizontal speed for position control
        self.declare_parameter('vertical_speed', 3.0)  # Max vertical speed for position control
        self.declare_parameter('command_velocity_timeout', 2.0)  # Max horizontal speed for position control

        # Multi-vehicle support - set up before spawn position lookup
        self.node_namespace = self.get_namespace()  # Used for multi-drone topics
        self.node_namespace = '' if self.node_namespace == '/' else self.node_namespace
        self._instance = int(self.node_namespace.split('_')[-1]) if '_' in self.node_namespace else 0
        
        # Initialize spawn position based on parameters or lookup
        spawn_x = self.get_parameter('spawn_x').value
        spawn_y = self.get_parameter('spawn_y').value
        
        # If spawn position parameters are not explicitly set (or set to -1.0), try to get them from the YAML file
        self._last_log_message = ""
        if spawn_x < 0.0 or spawn_y < 0.0:
            spawn_x, spawn_y, _ = self.get_spawn_position_from_yaml(self._instance)
            self.log(f'Retrieved spawn position from YAML file: x={spawn_x}, y={spawn_y}', level='info')
        
        # Store world spawn position for coordinate transformation between world and local coordinates
        self._spawn_position = Pose(x=spawn_x, y=spawn_y)

        self._takeoff_height = self.get_parameter('takeoff_height').value  # Used in takeoff sequence
        self._hover_timeout = self.get_parameter('hover_timeout').value  # Used in handle_navigation_state
        self._land_height_threshold = self.get_parameter('land_height_threshold').value  # Used in landing detection
        self._command_velocity_timeout = self.get_parameter('command_velocity_timeout').value  # Timeout for velocity command in seconds

        #################################
        ### Initialization parameters ###
        #################################

        # Flight parameters
        self._timeout_margin = 3  # Multiplier for timeout calculation (200% extra time)
        self._min_timeout = 10.0  # Minimum timeout duration in seconds

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
        self._auto_takeoff_in_progress = False  # Set when auto takeoff is in progress
        self._takeoff_complete = False  # Set when takeoff is finished, also serves as a flag to check if the drone is supposed to be in the air
        self._takeoff_start_time = 0.0  # Time when takeoff started
        self._landing_in_progress = False  # Set during landing sequence
        self._navigation_in_progress = False  # Set during navigation sequence
        self._navigation_start_time = 0.0  # Time when navigation started
        self._navigation_timeout = 0.0  # Timeout for navigation (seconds)
        self._hovering_in_progress = False  # Set during hovering sequence
        self._low_altitude_hovering_timer = None # Time when low altitude hovering started  in secs
        self._velocity_stopping = False  # Flag to indicate when we're in velocity stopping phase
        self._velocity_stop_timer = None # Timer for velocity stopping phase
        # vehicle status
        self._is_hold = False  # True when in hold mode
        self.vehicle_status = False
        self._armed = False
        self._is_offboard = False
        self._is_hold = False
        self._landing_in_progress = False
        self._pre_flight_checks_passed = False
        # ros2 parameter client parameters
        self._last_param_values = {}  # Tracks the last value set for each parameter
        self._last_param_update_times = {}  # Tracks when each parameter was last updated


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
        
        self.log(f'{self.get_parameter("coordinate_system").value} Controller initialized at spawn position: {self._spawn_position}', level='info')

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
        
        # Setup PX4 max velocity service server
        self.set_velocities_service = self.create_service(
            SetVelocities,
            f'{self.node_namespace}/set_velocities',
            self.set_velocities_callback
        )

        # Initialize a single parameter service client
        if self.node_namespace.startswith('/'):
            self._param_service_name = f"{self.node_namespace}/param/set_parameters"
        else:
            self._param_service_name = f"/{self.node_namespace}/param/set_parameters"
        self._param_client = self.create_client(SetParameters, self._param_service_name)

    def set_ros2_parameter(self, param_id: str, param_value: float):
        """Set a PX4 parameter using ROS2 parameter client API (non-blocking) with optimization."""
        # Convert to float for consistent comparison
        param_value = float(param_value)
        current_time = self.get_clock().now()
        
        # Check if we've set this parameter before
        if param_id in self._last_param_values:
            # If value is the same, check if enough time has passed
            if math.isclose(self._last_param_values[param_id], param_value, abs_tol=1e-1):
                # Get time since last update
                last_update_time = self._last_param_update_times[param_id]
                time_since_update = (current_time - last_update_time).nanoseconds / 1e9
                
                # Skip if less than 1 second has passed
                if time_since_update < 1.0:
                    self.log(f'Skipping parameter update for {param_id} (unchanged, last set {time_since_update:.2f}s ago)', 
                            level='debug')
                    return True
                else:
                    self.log(f'Refreshing parameter {param_id} after {time_since_update:.2f}s', level='debug')
            else:
                self.log(f'Updating parameter {param_id} from {self._last_param_values[param_id]} to {param_value}', 
                        level='debug')
        
        # Wait for the service to be available
        if not self._param_client.wait_for_service(timeout_sec=0.1):
            self.log(f'Parameter service {self._param_service_name} not available, falling back to vehicle command', level='warn')
            
            # Fallback: Try to use vehicle command to set parameter if we know the parameter mapping
            param_mappings = {
                'MPC_XY_VEL_MAX': 1114,
                'MPC_Z_VEL_MAX_UP': 1126, 
                'MPC_Z_VEL_MAX_DN': 1125,
                'MC_SLOW_DEF_HVEL': 1019,
                'MPC_VEL_MANUAL': 1101,
                'MPC_XY_CRUISE': 1106,
                'MPC_XY_VEL_ALL': 1111,
                'MC_SLOW_DEF_VVEL': 1020,
                'MPC_Z_VEL_ALL': 1122,
                'MIS_TAKEOFF_ALT': 1043
            }
            
            if param_id in param_mappings:
                param_id_num = param_mappings[param_id]
                self.log(f'Setting parameter {param_id} using vehicle command', level='info')
                self.publish_vehicle_command(
                    VehicleCommand.VEHICLE_CMD_DO_SET_PARAMETER, 
                    param1=float(param_id_num), 
                    param2=param_value
                )
                
                # Update tracking dictionaries
                self._last_param_values[param_id] = param_value
                self._last_param_update_times[param_id] = current_time
                return True
            else:
                self.log(f'No mapping found for parameter {param_id}, cannot set using fallback method', level='warn')
                return False

        # Create parameter message
        param_value_msg = ParameterValue()
        param_value_msg.type = ParameterType.PARAMETER_DOUBLE
        param_value_msg.double_value = param_value

        # Create parameter request
        param = Parameter()
        param.name = param_id
        param.value = param_value_msg
        request = SetParameters.Request()
        request.parameters = [param]

        # Send the request without waiting
        self._param_client.call_async(request)
        
        # Update our tracking dictionaries
        self._last_param_values[param_id] = param_value
        self._last_param_update_times[param_id] = current_time
        
        self.log(f'Parameter update sent: {param_id}={param_value}', level='debug')
        return True

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
        self.cmd_vel_subscriber = self.create_subscription(
            Twist, f'{self.node_namespace}/cmd_vel',
            self.cmd_vel_callback, 10)

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
        self._is_hold = vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LOITER
        # self._auto_takeoff_in_progress = vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_TAKEOFF
        self._landing_in_progress = vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LAND
        self._pre_flight_checks_passed = vehicle_status.pre_flight_checks_pass

    def point_yaw_callback(self, msg):
        """
        Convert topic commands to Pose object and set target.
        WARNING: This method does not use the action server, thus it does not return feedback nor response.
        """

        # Cancel the action goal if a new target is received
        self.cancel_action_goal()

        # If the offboard_mode is position, we need to convert the message to a Pose object
        if (self.get_parameter('offboard_mode').value == 'position' 
            or self.get_parameter('offboard_mode').value == 'velocity'):
            # Convert the PointYaw message to a Pose object
            self._target_pos = Pose.from_msg(msg)
            self._target = True
        elif (self.get_parameter('offboard_mode').value == 'acceleration' 
              or self.get_parameter('offboard_mode').value == 'attitude'
              or self.get_parameter('offboard_mode').value == 'body_rate'):
            self.log(f"Offboard mode {self.get_parameter('offboard_mode').value} not implemented yet", level='error') 
        else:
            self.log(f"Offboard mode {self.get_parameter('offboard_mode').value} not recognized", level='error')  

    def cmd_vel_callback(self, msg):
            """
            Convert topic Twist commands to Pose object and set target.
            WARNING: This method does not use the action server, thus it does not return feedback nor response.
            """

            # Cancel the action goal if a new target is received
            self.cancel_action_goal()

            # If the offboard_mode is not in velocity mode, we need to switch to it
            if (self.get_parameter('offboard_mode').value != 'velocity'):
                self.set_ros2_parameter('offboard_mode', 'velocity')
                self.log(f"Offboard mode changed to velocity", level='info')
        
            # Convert the Twist to target
            self._target_pos.x=msg.linear.x
            self._target_pos.y=msg.linear.y
            self._target_pos.z=msg.linear.z
            self._target_pos.yaw=msg.angular.z
            self._target = True

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
            self.log('[cmd_loop] Position estimation failed and drone is supposedly flying - EMERGENCY Landing', level='error')
            # If the drone has no valid position, we can't even use the LANDING state.
            self._state = self.IDLE
            # The sleep will outlast the frequency of the command loop, I hope changing the state first will avoid looping over this case
            self.land()
            return
  
        # Arming check
        if not self._armed:
            self.log('[cmd_loop] Cannot send commands - vehicle not armed', level='warn')
            self.arm()
            # If not armed, while in the air (Which should not be possible) we need to land
            if self._current_pos.z < 0.0 and self._takeoff_complete and (self._state == self.NAVIGATING or self._state == self.HOVER):
                self.log('[cmd_loop] Not armed and detected in the air - EMERGENCY Landing', level='error')
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
        
        # Always publish control mode message
        # If the drone is in taking off state, landing state, or hovering state, we HAVE TO use the position mode
        if self._state == self.TAKEOFF or self._state == self.LANDING or self._state == self.HOVER:
            self.publish_offboard_control_mode("position")
            # Publish the trajectory setpoint in position mode
            self.publish_position_setpoint(x=self._setpoint_to_run.x, y=self._setpoint_to_run.y, z=self._setpoint_to_run.z, yaw=self._setpoint_to_run.yaw)
        else:
            if self.get_parameter('offboard_mode').value == 'position':
                self.publish_offboard_control_mode("position")
                # Publish the trajectory setpoint in position mode
                # self.log(f"[cmd_loop] Publishing position setpoint: {self._setpoint_to_run}", level='info')
                self.publish_position_setpoint(x=self._setpoint_to_run.x, y=self._setpoint_to_run.y, z=self._setpoint_to_run.z, yaw=self._setpoint_to_run.yaw)
            elif self.get_parameter('offboard_mode').value == 'velocity':
                self.publish_offboard_control_mode("velocity")
                # Publish the trajectory setpoint in velocity mode
                # self.log(f"[cmd_loop] Publishing velocity setpoint: {self._setpoint_to_run}", level='info')
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
            self.log(f'[ctrl_loop: {self.get_state_name(self._state)}] Waiting for valid position data', level='info')
            return
        
        if self._target is False :
            self.log(f'[ctrl_loop: {self.get_state_name(self._state)}] Waiting for target', level='info')
            return
        
        if self._pre_flight_checks_passed is False:
            self.log(f'[ctrl_loop: {self.get_state_name(self._state)}] Pre-flight checks not passed! Check PX4 terminal.', level='warn')
            # Pre-flight check can be passed only in hold mode (as far as I know) on the ground
            if self._is_hold is False:
                self.hold_mode()
            return

        # If we get to this point, we're armed and we have somewhere to go to
        # Make the drone takeoff before sending the target as _setpoint_to_run
        self._state = self.TAKEOFF
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
                self.log(f'[ctrl_loop: {self.get_state_name(self._state)}] Takeoff failed, switching to LANDING state', level='error')
                self._state = self.LANDING
                self._setpoint_to_run = None
                self.log(f'[ctrl_loop: {self.get_state_name(self._state)}] Takeoff timeout reached', level='warn')
                self.log(f'[ctrl_loop: {self.get_state_name(self._state)}] Current position: {self._current_pos}', level='warn')
                self.log(f'[ctrl_loop: {self.get_state_name(self._state)}] Setpoint to run: {self._setpoint_to_run}', level='warn')
                self.log(self.vehicle_status, level='debug')
                self._takeoff_complete = False
                if self._current_goal_handle is not None:
                    # Create failure result for the goal
                    result = GotoPosition.Result()
                    result.success = False
                    self._current_goal_handle._goal_result = result
                    # Signal the goal handle's event
                    self._current_goal_handle._goal_event.set()
                    self._current_goal_handle.abort()


    def handle_command_takeoff_state(self):
        """
        Handle takeoff state using the auto takeoff command.
        """

        # 1. Cancel any set goal if it exists
        if self._setpoint_to_run is not None:
            self._setpoint_to_run = None
            self.log(f'[ctrl_loop: {self.get_state_name(self._state)}] Setpoint to run cancelled', level='info')

        # 2. If auto takeoff sequence hasn't started yet, start it. And start the takeoff timer
        if self._auto_takeoff_in_progress is False:
            self.set_auto_takeoff_height()
            self.update_vertical_speed(5.0) # Default takeoff speed to avoid velocity control from changing it 
            # MPC_TKO_SPEED is not modified by update_vertical_speed but it is limited by it
            self.take_off_mode()
            self.arm()
            # self.take_off()
            self._auto_takeoff_in_progress = True
            self._takeoff_start_time = self.get_clock().now()

        # 3. Check if the drone is at the right height (with the same objective threshold as navigating), if so pass to NAVIGATING state
        # auto takeoff make it go a bit higher than the takeoff height, so we need to modify the threshold
        # if math.isclose(self._current_pos.z, -self._takeoff_height, abs_tol=self._position_threshold):
        if self._current_pos.z < -self._takeoff_height+self._position_threshold:
            self.log(f'[ctrl_loop: {self.get_state_name(self._state)}] Current height: {self._current_pos.z}, Takeoff height: {-self._takeoff_height}', level='info')
            self._takeoff_complete = True
            self._auto_takeoff_in_progress = False
            self._last_reached_position = Pose(
                x=self._current_pos.x,
                y=self._current_pos.y,
                z=-self._takeoff_height,
                yaw=self._current_pos.yaw
            )
            self.log(f'[ctrl_loop: {self.get_state_name(self._state)}] Takeoff complete, transitioning to NAVIGATING state', level='info')
            self._state = self.NAVIGATING
            return
        
        # 4. Check if timeout has been reached, if so raise error to try setpoint takeoff
        if self._auto_takeoff_in_progress is True and self._takeoff_complete is False:
            try:
                if ((self.get_clock().now() - self._takeoff_start_time).nanoseconds / 1e9) > self._min_timeout:
                    self._takeoff_case = 'setpoint'
                    self._auto_takeoff_in_progress = False
                    self.log(f'[ctrl_loop: {self.get_state_name(self._state)}] Takeoff timeout reached, switching to setpoint takeoff', level='warn')
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
            self.update_vertical_speed(5.0) # Default takeoff speed to avoid velocity control from changing it 
            # MPC_TKO_SPEED is not modified by update_vertical_speed but it is limited by it
            self._setpoint_to_run = Pose(
                x=self._current_pos.x,
                y=self._current_pos.y,
                z=-self._takeoff_height,
                yaw=self._current_pos.yaw
            )
            # Start the takeoff timer
            self._takeoff_start_time = self.get_clock().now()
            self.log(f'[ctrl_loop: {self.get_state_name(self._state)}] Setpoint takeoff initiated', level='info')
            return

        # 2. check if the drone is at the right height (with the same objective threshold as navigating), if so pass to NAVIGATING state
        # if math.isclose(self._current_pos.z, -self._takeoff_height, abs_tol=self._position_threshold):
        if self._current_pos.z < -self._takeoff_height+self._position_threshold:
            self.log(f'[ctrl_loop: {self.get_state_name(self._state)}] Takeoff complete, transitioning to NAVIGATING state', level='info')
            self._state = self.NAVIGATING
            self._takeoff_complete = True
            self._last_reached_position = self._setpoint_to_run
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
            self.update_vertical_speed(2.5) # Default takeoff speed to avoid velocity control from changing it 
            # MPC_TKO_SPEED is not modified by update_vertical_speed but it is limited by it
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
            self.update_vertical_speed(2.5) # Default takeoff speed to avoid velocity control from changing it 
            # MPC_TKO_SPEED is not modified by update_vertical_speed but it is limited by it
        else:
            # 2. Check if horizontal speed is slow enough to start vertical landing. Run the land command only once to avoid flooding PX4
            if abs(self.vehicle_local_position.vx) < 0.1 and abs(self.vehicle_local_position.vy) < 0.1:
                self.log(f'[ctrl_loop: {self.get_state_name(self._state)}] Horizontal speed low enough, landing initiated', level='info')
                self._setpoint_to_run = None
                self.land()
                self.log(f'[ctrl_loop: {self.get_state_name(self._state)}] Landing complete', level='info')
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
                # Reset stopping phase when new command is received
                self._velocity_stopping = False  
                self._velocity_stop_timer = None

                # Update the horizontal and vertical max speed to the max speed of the drone to the value given by the user's target velocity
                # Not Necessary anymore (if the max speed parameters are set to default (12 and 3)).
                # Drone will normally fllow the velcity copmmand given by the user.
                self.update_horizontal_speed(abs(max(abs(self._target_pos.x)+0.2, abs(self._target_pos.y)+0.2)))
                self.update_vertical_speed(abs(self._target_pos.z) + 0.2)
                
                # a. Set the setpoint to the target velocity converted to the local frame
                self.log(f'[ctrl_loop: {self.get_state_name(self._state)}] Target velocity is x :{self._target_pos.x} m/s, y: {self._target_pos.y} m/s, z : {self._target_pos.z}, yawspeed : {self._target_pos.yaw_in_radians()} rad/s', level='info')
                self._setpoint_to_run = convert_user_velocity_to_local_velocity(self._current_pos,
                                                                              self._target_pos,
                                                                              self.get_parameter('coordinate_system').value)

                self._navigation_in_progress = True  # Start the navigation timer
                self._command_velocity_timer = self.get_clock().now()
                    
            if self._navigation_in_progress is True:
                # b. If there is no new target, we need to check if the velocity command has timed out
                elapsed_time = (self.get_clock().now() - self._command_velocity_timer).nanoseconds / 1e9
                
                if elapsed_time > self._command_velocity_timeout:
                    # Modified timeout handling:
                    # If we're not already in stopping phase, enter it
                    if not self._velocity_stopping:
                        self.log(f'[ctrl_loop: {self.get_state_name(self._state)}] Velocity command timeout, sending stop command', level='info')
                        # Send a zero velocity command to smoothly stop the drone
                        self._setpoint_to_run = Pose(x=0.0, y=0.0, z=0.0, yaw=0.0)
                        self._velocity_stopping = True
                        self._velocity_stop_timer = self.get_clock().now()
                        return
                    
                    # c. If we're in stopping phase, check if 1 second has elapsed
                    if ((self.get_clock().now() - self._velocity_stop_timer).nanoseconds / 1e9) > 1.0:
                        # If 1 second has passed and no new command was received, switch to HOVER
                        self.log(f'[ctrl_loop: {self.get_state_name(self._state)}] stop complete, switching to HOVER state', level='info')
                        self._target = False
                        self._navigation_in_progress = False
                        self._velocity_stopping = False
                        self._velocity_stop_timer = None
                        self._last_reached_position = self._current_pos
                        self._state = self.HOVER
                return

        # 2. Are we in offboard position mode? If so, we are navigating, sending a position command and track the progress
        if self.get_parameter('offboard_mode').value == 'position':
            # a. Check if a new target has been received
            if self._target is True and self._navigation_in_progress is False:
                self.log(f'[ctrl_loop: {self.get_state_name(self._state)}] Going to {self._target_pos}', level='info')
                self._target = False
                #   raise the flag, start the timer and set the timeout to get there
                #   set _setpoint_to_run to the target position (from the requested coordinate system) converted to the local frame
                self._local_target_pos = convert_objective_coordinates_to_local_coordinates(self._spawn_position,
                                                                                        self._target_pos,
                                                                                        self._current_pos,
                                                                                        self.get_parameter('coordinate_system').value)
                self.log(f'[ctrl_loop: {self.get_state_name(self._state)}] Local target position: {self._local_target_pos}', level='info')
                self.update_vertical_speed(self.get_parameter('vertical_speed').value) # change to parameter speed    
                self.update_horizontal_speed(self.get_parameter('horizontal_speed').value) # change to parameter speed    
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
                self.log(f'[ctrl_loop: {self.get_state_name(self._state)}] Navigation timeout reached', level='warn')
                self.log(f'[ctrl_loop: {self.get_state_name(self._state)}] Current position: {self._current_pos}', level='warn')
                self.log(f'[ctrl_loop: {self.get_state_name(self._state)}] Setpoint to run: {self._setpoint_to_run}', level='warn')
                self.log(f'[ctrl_loop: {self.get_state_name(self._state)}] Hovering initiated due to timeout', level='error')
                self.log(self.vehicle_status, level='debug')
                self._last_reached_position = self._current_pos
                self._navigation_in_progress = False
                if self._current_goal_handle is not None:
                    result = GotoPosition.Result()
                    result.success = True
                    # Set the result directly on the goal handle
                    self._current_goal_handle._goal_result = result
                    # Signal the goal handle's event
                    self._current_goal_handle._goal_event.set()
                    self._current_goal_handle.succeed()
                return
            
        # 5. Check if the drone has reached the target position
        if distance_to_target <= self._position_threshold:
            self.log(f'[ctrl_loop: {self.get_state_name(self._state)}] Target position reached', level='info')
            self.log(f'[ctrl_loop: {self.get_state_name(self._state)}] Current position: {self._current_pos}', level='info')
            self._last_reached_position = self._local_target_pos
            self._navigation_in_progress = False
            # Check if there is still a valid goal handle before succeeding
            if self._current_goal_handle is not None:
                # NEW: Check goal status before attempting to succeed it
                if self._current_goal_handle.status == GoalStatus.STATUS_EXECUTING:
                    result = GotoPosition.Result()
                    result.success = True
                    # Set the result directly on the goal handle
                    self._current_goal_handle._goal_result = result
                    # Signal the goal handle's event
                    self._current_goal_handle._goal_event.set()
                    self._current_goal_handle.succeed()
                    self.log(f'Goal completed with result: {result.success}', level='info')
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
            self.log(f'[ctrl_loop: {self.get_state_name(self._state)}] Hovering at : {self._last_reached_position}', level='info')
            return
        
        # 3. If the drone is below the land height threshold and timer has not already started, we start the timer
        if -self._current_pos.z < self._land_height_threshold:
            if self._low_altitude_hovering_timer is None:
                self._low_altitude_hovering_timer = self.get_clock().now()
            else:
                # We check if the timer has timed out
                if ((self.get_clock().now() - self._low_altitude_hovering_timer).nanoseconds / 1e9) > self._hover_timeout:
                    # If it has, we start the landing sequence
                    self.log(f'[ctrl_loop: {self.get_state_name(self._state)}] Hovering timeout reached, initiating landing', level='warn')
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
        msg.position = [float('nan'), float('nan'), float('nan')]
        msg.velocity = [float(vx), float(vy), float(vz)]
        msg.acceleration = [float('nan'), float('nan'), float('nan')]
        msg.yaw = float('nan')
        msg.yawspeed = float(yawspeed)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    def publish_position_setpoint(self, x: float, y: float, z: float, yaw: float):
        """Publish position setpoint."""
        msg = TrajectorySetpoint()
        msg.position = [float(x), float(y), float(z)]
        msg.velocity = [float('nan'), float('nan'), float('nan')]
        msg.acceleration = [float('nan'), float('nan'), float('nan')]
        msg.yaw = float(yaw)
        msg.yawspeed = float('nan')
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
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.log('Arm command sent', level='info')
        
    def disarm(self):
        """Send disarm command."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
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
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_SET_NAV_STATE, param1=4.0)    
        self.log('Switching to hold mode', level='info')

    def take_off_mode(self):
        """Switch to takeoff mode."""
        # Send nav_state request for takeoff mode
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_SET_NAV_STATE, param1=17.0)   
        self.log('Switching to takeoff mode', level='info')

    def set_auto_takeoff_height(self):
        """Set the auto takeoff height.
        PX4 Takeoff height parameter :
        MIS_TAKEOFF_ALT : id= 1043  default=2.5(m) Default takeoff altitude
        """
        # self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_PARAMETER, param1 = 1043.0, param2=self._takeoff_height)
        self.set_ros2_parameter('MIS_TAKEOFF_ALT', self._takeoff_height)
        self.log(f"Takeoff altitude set to {self._takeoff_height}", level='info')

    # I CANT MAKE THIS COMMAND WORK
    def take_off(self): 
        """Command the vehicle to take off to specified altitude"""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, param1 = 1.0, param7=self._takeoff_height) # param7 is altitude in meters
        self.log("Takeoff command send", level='info')

    def land_mode(self):
        """Switch to land mode."""
        # Send nav_state request for land mode
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_SET_NAV_STATE, param1=18.0)   
        self.log('Switching to land mode', level='info')

    def land(self):
        """Switch to land mode."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.land_mode()
        self._takeoff_complete = False
        self._target = False
        self.log('Landing command sent', level='info')

    def update_horizontal_speed(self, horizontal_speed):
        """Update the horizontal speed.
        PX4 Horizontall speed parameters :
        MC_SLOW_DEF_HVEL : id= 1019  default=3.0(m/s) Default horizontal velocity limit
        MPC_VEL_MANUAL   : id= 1101  default=10.0(m/s) Manual velocity limit
        MPC_XY_VEL_MAX   : id= 1114  default=12.0(m/s) Maximum horizontal velocity
        MPC_XY_CRUISE    : id= 1106  default=5.0(m/s) Cruise horizontal velocity
        MPC_XY_VEL_ALL   : id= 1111  default=-10.0(m/s) Overall horizontal velocity
        """
        horizontal_speed = float(horizontal_speed)
        # self.log(f'Updating horizontal speed to {horizontal_speed}', level='info')
        
        # Set MC_SLOW_DEF_HVEL
        param_id = 'MC_SLOW_DEF_HVEL'
        self.log(f'Setting parameter {param_id} to {horizontal_speed}', level='debug')
        self.set_ros2_parameter(param_id, horizontal_speed)
        # self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_PARAMETER, param1=1019.0, param2=horizontal_speed)
        
        # Set MPC_VEL_MANUAL
        param_id = 'MPC_VEL_MANUAL'
        self.set_ros2_parameter(param_id, horizontal_speed)
        # self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_PARAMETER, param1=1101.0, param2=horizontal_speed)
        
        # Set MPC_XY_VEL_MAX
        param_id = 'MPC_XY_VEL_MAX'
        self.log(f'Setting parameter {param_id} to {horizontal_speed}', level='debug')
        self.set_ros2_parameter(param_id, horizontal_speed)
        # self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_PARAMETER, param1=1114.0, param2=horizontal_speed)
        
        # Set MPC_XY_CRUISE
        param_id = 'MPC_XY_CRUISE'
        self.log(f'Setting parameter {param_id} to {horizontal_speed}', level='debug')
        self.set_ros2_parameter(param_id, horizontal_speed)
        # self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_PARAMETER, param1=1106.0, param2=horizontal_speed)
        
        # Set MPC_XY_VEL_ALL
        param_id = 'MPC_XY_VEL_ALL'
        self.log(f'Setting parameter {param_id} to {horizontal_speed}', level='debug')
        self.set_ros2_parameter(param_id, horizontal_speed)
        # self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_PARAMETER, param1=1111.0, param2=horizontal_speed)

    def update_vertical_speed(self, vertical_speed):
        """Update the vertical speed.
        PX4 Vertical speed parameters :
        MC_SLOW_DEF_VVEL : id= 1020  default=1.0(m/s) Default vertical velocity limit
        MPC_Z_VEL_ALL    : id= 1122  default=-3.0(m/s) Overall vertical velocity limit
        MPC_Z_VEL_MAX_DN : id= 1125  default=1.5(m/s) Maximum downward vertical velocity
        MPC_Z_VEL_MAX_UP : id= 1126  default=3.0(m/s) Maximum upward vertical velocity
        """
        vertical_speed = float(vertical_speed)
        # self.log(f'Updating vertical speed to {vertical_speed}', level='info')

        # Set MC_SLOW_DEF_VVEL
        param_id = 'MC_SLOW_DEF_VVEL'
        self.log(f'Setting parameter {param_id} to {vertical_speed}', level='debug')
        self.set_ros2_parameter(param_id, vertical_speed)

        # Set MPC_Z_VEL_ALL
        param_id = 'MPC_Z_VEL_ALL'
        self.log(f'Setting parameter {param_id} to {vertical_speed}', level='debug')
        self.set_ros2_parameter(param_id, vertical_speed)

        # Set MPC_Z_VEL_MAX_DN
        param_id = 'MPC_Z_VEL_MAX_DN'
        self.log(f'Setting parameter {param_id} to {vertical_speed}', level='debug')
        self.set_ros2_parameter(param_id, vertical_speed)

        # Set MPC_Z_VEL_MAX_UP
        param_id = 'MPC_Z_VEL_MAX_UP'
        self.log(f'Setting parameter {param_id} to {vertical_speed}', level='debug')
        self.set_ros2_parameter(param_id, vertical_speed)
        
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
            # Explicitly abort the existing goal if it's still executing
            if self._current_goal_handle.status == GoalStatus.STATUS_EXECUTING:
                self.log('Aborting current goal', level='info')
                
                # Create a failure result for the aborted goal
                result = GotoPosition.Result()
                result.success = False
                
                # Set the result on the goal handle's properties
                if hasattr(self._current_goal_handle, '_goal_result'):
                    self._current_goal_handle._goal_result = result
                    # Signal that the goal is complete
                    if hasattr(self._current_goal_handle, '_goal_event'):
                        self._current_goal_handle._goal_event.set()
                
                # Then call abort WITHOUT passing the result
                self._current_goal_handle.abort()
                    
            self._goal_event.set()  # Signal the control loop to stop processing
        
        # Start fresh for the new goal
        self._current_goal_handle = None  # Clear before creating a new one
        self._goal_event.clear()  # Reset the event for the new goal

        # Convert the goal to the target position
        self._target_pos = Pose.from_msg(goal_handle.target)
        self._target = True
        self._navigation_in_progress = False

        return GoalResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        """
        Execute the goal by waiting for the control loop to signal completion.
        This method uses asynchronous waiting to avoid blocking the processor.
        """
        # Create a per-goal event and result
        goal_handle._goal_event = threading.Event()
        goal_handle._goal_result = None

        # Store the new goal handle
        previous_goal_handle = self._current_goal_handle
        self._current_goal_handle = goal_handle

        # If there was a previous goal, mark it as aborted
        if previous_goal_handle is not None and previous_goal_handle != goal_handle:
            # Check if the previous goal is still active before aborting
            if previous_goal_handle.status == GoalStatus.STATUS_EXECUTING:
                # Create failure result for previous goal
                result = GotoPosition.Result()
                result.success = False
                previous_goal_handle._goal_result = result
                previous_goal_handle._goal_event.set()  # Signal completion
                previous_goal_handle.abort()

        # Wait for the control loop to signal that the goal is complete
        while not goal_handle._goal_event.is_set():
            # Poll to check if the goal is complete
            goal_handle._goal_event.wait(timeout=1.0)

        # Return THIS goal's result, not the shared one
        return goal_handle._goal_result if goal_handle._goal_result is not None else GotoPosition.Result()

    def cancel_callback(self, cancel_request):
        """
        Handle cancel requests. If a goal is currently being processed,
        cancel it and signal the control loop to stop processing.
        """
        self.log('Received cancel request', level='info')

        if self._current_goal_handle is not None:
            self.log('Canceling current goal', level='info')
            
            # Create a result with success=False for the canceled goal
            result = GotoPosition.Result()
            result.success = False
            
            # Set the result directly on the goal handle
            if hasattr(self._current_goal_handle, '_goal_result'):
                self._current_goal_handle._goal_result = result
                # Signal the goal handle's event if it exists
                if hasattr(self._current_goal_handle, '_goal_event'):
                    self._current_goal_handle._goal_event.set()
            
            # Use abort() instead of canceled() - this is the valid transition
            if self._current_goal_handle.status == GoalStatus.STATUS_EXECUTING:
                self._current_goal_handle.abort()
            
            self._current_goal_handle = None
            
            # Stop the navigation by setting the navigation flag to False and setting the setpoint to current position
            self._target = False
            self._navigation_in_progress = False
            self._velocity_stopping = False
            self._velocity_stop_timer = None
            self._last_reached_position = self._current_pos

            # Signal the control loop to stop processing
            self._goal_event.set()
            return CancelResponse.ACCEPT
        
        # Reset flags
        self._target = False
        self._navigation_in_progress = False
        return CancelResponse.REJECT
    
    def cancel_action_goal(self):
        """Cancel any active action goal when receiving a direct topic command."""
        if self._current_goal_handle is not None:
            self.log('Canceling current action goal due to topic command', level='info')
            
            # First set the result to failure and signal the event
            if hasattr(self._current_goal_handle, '_goal_result'):
                result = GotoPosition.Result()
                result.success = False
                self._current_goal_handle._goal_result = result
                
                # Signal the goal handle's event if it exists
                if hasattr(self._current_goal_handle, '_goal_event'):
                    self._current_goal_handle._goal_event.set()
            
            # Use abort() instead of canceled() - this is the valid transition
            if self._current_goal_handle.status == GoalStatus.STATUS_EXECUTING:
                self._current_goal_handle.abort()
            
            # Reset everything
            self._current_goal_handle = None
            self._goal_event.set()  # Signal any code waiting on the shared event
            self._target = False
            self._navigation_in_progress = False

    
    ################################
    ### SERVICE SERVER CALLBACKS ###
    ################################

    def set_velocities_callback(self, request, response):
        """
        Service callback to update horizontal and vertical speeds.
        """
        try:
            if request.horizontal_speed != 0.0:
                self.update_horizontal_speed(request.horizontal_speed)
            if request.vertical_speed != 0.0:
                self.update_vertical_speed(request.vertical_speed)

            response.success = True
            response.message = "Velocities updated successfully."
            self.log(response.message, level='info')

        except Exception as e:
            response.success = False
            response.message = f"Failed to update velocities: {str(e)}"
            self.log(response.message, level='error')

        return response

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
    
    def get_state_name(self, state):
        """Get the name of the current state."""
        if state == self.IDLE:
            return "IDLE"
        elif state == self.TAKEOFF:
            return "TAKEOFF"
        elif state == self.LANDING:
            return "LANDING"
        elif state == self.NAVIGATING:
            return "NAVIGATING"
        elif state == self.HOVER:
            return "HOVER"
        else:
            return "UNKNOWN"

    def calculate_timeout(self, current_position: list, target_position: list, expected_mean_speed=None) -> float:
        """Calculate timeout based on distance and velocity."""
        self.log('calculate timeout current position {} target position {}'.format(current_position, target_position), level='info')
        # Calculate distance to target using the existing method
        distance = calculate_distance(current_position, target_position)
        # Calculate expected time with safety margin
        if expected_mean_speed is None:
            expected_mean_speed = self.get_parameter('horizontal_speed').value
        velocity = expected_mean_speed * 0.7  # Reduced for acceleration/deceleration
        base_time = distance / velocity
        timeout = max(self._min_timeout, base_time * self._timeout_margin)

        self.log(f'Timeout calculation: distance={distance:.1f}m', level='info')
        self.log(f'velocity={velocity:.1f}m/s, timeout={timeout:.1f}s', level='info')
        return timeout
    
    def get_spawn_position_from_yaml(self, drone_id):
        """
        Get the spawn position from the YAML file by searching for the drone ID in all teams.
        Looks for the YAML in:
        1. Any config/ folder under offboard_control_py in src/
        2. Absolute path (if given)
        3. Current working directory
        """
        default_spawn = (0.0, 0.0, 90.0)
        try:
            yaml_file = self.get_parameter('spawn_position_file').value
            yaml_paths = []

            # 1. Go up to ros2_ws, then down into src and search for offboard_control_py/config/yaml_file
            current_dir = os.path.abspath(__file__)
            while current_dir != '/' and os.path.basename(current_dir) != 'ros2_ws':
                current_dir = os.path.dirname(current_dir)
            if os.path.basename(current_dir) == 'ros2_ws':
                src_dir = os.path.join(current_dir, 'src')
                # Recursively search for offboard_control_py/config/yaml_file under src
                for root, dirs, files in os.walk(src_dir):
                    if os.path.basename(root) == 'offboard_control_py':
                        config_path = os.path.join(root, 'config', yaml_file)
                        if os.path.exists(config_path):
                            yaml_paths.append(config_path)
            self.log(f'Looking for spawn position YAML file in source tree: {yaml_paths}', level='info')

            # 2. Absolute path (if user gave one)
            if os.path.isabs(yaml_file):
                yaml_paths.append(yaml_file)

            # 3. Current directory (last resort)
            yaml_paths.append(yaml_file)

            # Try each path until we find the file
            yaml_path = None
            for path in yaml_paths:
                if os.path.exists(path):
                    yaml_path = path
                    self.log(f'Found spawn position YAML file at: {yaml_path}', level='info')
                    break

            if yaml_path is None:
                self.log(f'Spawn position YAML file not found in any of the searched locations', level='warn')
                return default_spawn

            with open(yaml_path, 'r') as file:
                spawn_data = yaml.safe_load(file)

            if spawn_data is None or not hasattr(spawn_data, 'items'):
                self.log('Invalid YAML file format - root should be a dictionary', level='warn')
                return default_spawn

            drone_id_str = str(drone_id)
            for team_id, team_data in spawn_data.items():
                if hasattr(team_data, 'get'):
                    drone_data = team_data.get(drone_id_str)
                    if drone_data:
                        x = float(drone_data.get('x', 0.0))
                        y = float(drone_data.get('y', 0.0))
                        yaw = float(drone_data.get('yaw', 90.0))
                        self.log(f'Found spawn position in YAML for drone {drone_id_str} in team {team_id}: x={x}, y={y}, yaw={yaw}', level='info')
                        return (x, y, yaw)

            self.log(f'Drone ID {drone_id_str} not found in any team in spawn position YAML', level='warn')
            return default_spawn

        except Exception as e:
            self.log(f'Error reading spawn position from YAML: {str(e)}', level='error')
            return default_spawn

#################
### Functions ###
#################

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
        x = goal_position.x
        y = goal_position.y
        z = goal_position.z
        yaw = goal_position.yaw

    # Convert to LOCAL FRD coordinates to local NED coordinates
    elif system == 'FRD':
        # Calculate the rotation matrix for converting FRD to NED
        rotation_matrix = np.array([
            [math.cos(current_position.yaw), -math.sin(current_position.yaw)],
            [math.sin(current_position.yaw), math.cos(current_position.yaw)]
        ])
        ned_offset_horizontal = rotation_matrix @ np.array([goal_position.x, goal_position.y])
        
        x = current_position.x + ned_offset_horizontal[0]
        y = current_position.y + ned_offset_horizontal[1]
        z = current_position.z + goal_position.z
        yaw = current_position.yaw + goal_yaw_rad

    # Convert to LOCAL FLU coordinates to local NED coordinates
    elif system == 'FLU':
        # Calculate the rotation matrix for body-fixed FLU to world NED
        cos_yaw = math.cos(current_position.yaw)
        sin_yaw = math.sin(current_position.yaw)
        
        # Forward component contribution to NED
        x_forward = cos_yaw * goal_position.x
        y_forward = sin_yaw * goal_position.x
        
        # Left component contribution to NED (90° CCW from forward)
        # Corrected signs for proper "left" direction
        x_left = -sin_yaw * goal_position.y  # Fix: negative sine for left component
        y_left = cos_yaw * goal_position.y   # Fix: positive cosine for left component
        
        # Combine forward and left contributions
        x = current_position.x + x_forward + x_left
        y = current_position.y + y_forward + y_left
        z = current_position.z - goal_position.z  # Up in FLU is negative z in NED
        yaw = current_position.yaw + goal_yaw_rad
    
    else:
        raise ValueError(f"Unsupported coordinate system: {system}")

    return Pose(x=x, y=y, z=z, yaw=yaw)


def convert_user_velocity_to_local_velocity(current_position: Pose, goal_velocity: Pose, system='NED'):
    """
    Convert goal velocity from selected coordinate system to local NED.
    Parameters:
    - current_position: Pose object with yaw representing the current orientation in local NED.
    - goal_velocity: Pose object with x, y, z, and yaw representing the goal velocity in the selected coordinate system.
                     (linear x,y,z for velocity, and yaw for yawspeed in degrees/second)
    - system: String representing the target coordinate system ('NED', 'local_NED', 'FRD', 'FLU').
    Returns:
    - Pose object representing the velocity in local NED, with yaw in radians/second.
    """
    # Always convert yaw from degrees/second to radians/second
    yaw_in_radians = goal_velocity.yaw_in_radians()
    
    # Create result Pose
    result = Pose()
    
    # Get current orientation
    yaw = current_position.yaw
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    
    # For NED or local_NED, no transformation needed as velocity is already in world frame
    if system == 'NED' or system == 'local_NED':
        # Already in NED coordinate system, no conversion needed
        result.x = goal_velocity.x
        result.y = goal_velocity.y
        result.z = goal_velocity.z
    
    # For FRD (Forward-Right-Down) body frame to NED
    elif system == 'FRD':
        # Body-fixed FRD to local NED transformation
        body_forward = goal_velocity.x
        body_right = goal_velocity.y
        body_down = goal_velocity.z
        
        # Apply correct coordinate transformation
        # North = Forward * cos(yaw) - Right * sin(yaw)
        result.x = body_forward * cos_yaw - body_right * sin_yaw
        # East = Forward * sin(yaw) + Right * cos(yaw)
        result.y = body_forward * sin_yaw + body_right * cos_yaw
        # Down stays the same
        result.z = body_down
    
    # Default to FLU body frame or any unrecognized system
    else:
        # Body-fixed FLU (Forward-Left-Up) to local NED transformation
        body_forward = goal_velocity.x
        body_left = goal_velocity.y
        body_up = goal_velocity.z
        
        # Apply coordinate transformation 
        result.x = body_forward * cos_yaw + body_left * sin_yaw
        result.y = body_forward * sin_yaw - body_left * cos_yaw
        result.z = -body_up  # Negative because Up in FLU is opposite of Down in NED
    
    # Set the yaw rate - this is always just the conversion from degrees to radians
    result.yaw = yaw_in_radians
    
    return result

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
    print('Starting offboard control node...')
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
