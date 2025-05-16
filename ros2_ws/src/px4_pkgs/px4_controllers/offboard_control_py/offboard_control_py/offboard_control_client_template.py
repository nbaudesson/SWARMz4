#!/usr/bin/env python3
"""
Drone Controller Template

This module provides a template for controlling a PX4 drone in a combat simulation,
focused on essential control tools and state machine structure with improved mechanisms
for state management, attitude tracking, and mission completion.

Usage:
    ros2 run offboard_control_py offboard_control_client_template --ros-args -r __ns:=/px4_1
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv import SetParameters

from px4_controllers_interfaces.action import GotoPosition
from px4_controllers_interfaces.msg import PointYaw

from swarmz_interfaces.msg import Detections, Detection
from swarmz_interfaces.srv import Kamikaze, Missile

from px4_msgs.msg import VehicleLocalPosition, VehicleAttitude

from std_msgs.msg import String, Int32

import yaml
import os
import math
import time
from ament_index_python.packages import get_package_share_directory


class DroneController(Node):
    """
    Controller for a single drone in a combat simulation.
    
    This template provides the essential interfaces to control your drone:
    - Position and velocity commands
    - Combat actions (missiles, kamikaze)
    - Detection handling
    - Team communication
    
    Customize the state handler methods to implement your control logic.
    """
    
    # State machine states - modify these for your mission
    STATE_INIT = 0
    STATE_TAKEOFF = 1
    STATE_MISSION = 2
    STATE_COMBAT = 3
    STATE_RTL = 4  # Return To Launch
    STATE_COMPLETE = 5
    
    def __init__(self):
        """Initialize the drone controller with all necessary connections."""
        super().__init__('drone_controller')
        
        # ====== CALLBACK GROUPS (for thread safety) ======
        self.action_group = MutuallyExclusiveCallbackGroup()
        self.service_group = MutuallyExclusiveCallbackGroup()
        self.timer_group = ReentrantCallbackGroup()
        self.subscriber_group = ReentrantCallbackGroup()
        
        # ====== PARAMETERS ======
        self.declare_parameter('offboard_mode', 'position')  # 'position' or 'velocity'
        self.declare_parameter('coordinate_system', 'NED')   # 'local_NED', 'NED', 'FLU', 'FRD'
        self.declare_parameter('drone_id', -1)               # -1 means determine from namespace
        self.declare_parameter('spawn_position_file', 'spawn_position.yaml')
        self.declare_parameter('return_to_base', True)       # Whether to return to base after mission
        
        # ====== DRONE IDENTIFICATION ======
        # Determine which drone we're controlling from parameter or namespace
        self.setup_drone_identity()
        
        # ====== SPAWN POSITION ======
        # Get the spawn position of this drone from configuration
        self.spawn_position = self.get_spawn_position()
        self.get_logger().info(f'Spawn position: x={self.spawn_position[0]}, y={self.spawn_position[1]}')
        
        # ====== STATE TRACKING ======
        self.current_position = None      # Updated via action feedback
        self.estimated_position = None    # Updated via vehicle_local_position (more precise)
        self.current_attitude = None      # Updated via vehicle_attitude
        self.last_detection_time = None   # When we last saw something
        self.health = 1                   # Current health (0-1)
        self.team_id = 1 if self.drone_id <= 5 else 2  # Team assignment
        self.missile_count = 2            # Default missiles
        self.action_in_progress = False   # Is a movement action in progress?
        self.goal_handle = None           # Current action goal handle
        self.latest_detections = []       # Most recent sensor detections
        self.latest_message = None        # Most recent received message
        
        # ====== BEHAVIOR STATE MACHINE ======
        # Current state in the behavior state machine
        self.state = self.STATE_INIT
        self.state_changed_time = time.time()
        self.return_to_base = self.get_parameter('return_to_base').value
        
        # ====== OFFBOARD CONTROL ======
        # Get default parameters
        self.default_offboard_mode = self.get_parameter('offboard_mode').value
        self.default_coordinate_system = self.get_parameter('coordinate_system').value
        self.get_logger().info(f'Using defaults: offboard_mode={self.default_offboard_mode}, '
                            f'coordinate_system={self.default_coordinate_system}')
        
        # Set up action client for position commands
        self.action_client = ActionClient(
            self,
            GotoPosition,
            f'{self.namespace}/goto_position',
            callback_group=self.action_group
        )
        
        # Set up publisher for direct pose/velocity commands
        self.target_pose_publisher = self.create_publisher(
            PointYaw,
            f'{self.namespace}/target_pose',
            10,
            callback_group=self.action_group
        )
        
        # Set up parameter client for offboard_control_px4
        self.param_client = self.create_client(
            SetParameters,
            f'{self.namespace}/offboard_control_px4/set_parameters',
            callback_group=self.service_group
        )
        
        # Subscribe to vehicle_local_position for precise position information
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition,
            f'{self.namespace}/fmu/out/vehicle_local_position',
            self.vehicle_local_position_callback,
            rclpy.qos.qos_profile_sensor_data,
            callback_group=self.subscriber_group
        )
        
        # Subscribe to vehicle_attitude for orientation information
        self.vehicle_attitude_subscriber = self.create_subscription(
            VehicleAttitude,
            f'{self.namespace}/fmu/out/vehicle_attitude',
            self.vehicle_attitude_callback,
            rclpy.qos.qos_profile_sensor_data,
            callback_group=self.subscriber_group
        )
        
        # ====== GAME MASTER INTERFACE ======
        self.setup_game_master_interface()
        
        # ====== BEHAVIOR TIMER ======
        # Create a behavior timer that runs our state machine (5 Hz)
        self.behavior_timer = self.create_timer(
            0.2,  # 5 Hz for behavior updates
            self.behavior_callback,
            callback_group=self.timer_group
        )
        
        # Apply default offboard parameters
        self.set_offboard_parameters(
            offboard_mode=self.default_offboard_mode,
            coordinate_system=self.default_coordinate_system
        )
        
        self.get_logger().info(f'Drone {self.drone_id} controller initialized (Team: {self.team_id})')

    def setup_drone_identity(self):
        """Determine drone ID and namespace from parameters or ROS namespace."""
        self.drone_id = self.get_parameter('drone_id').value
        self.namespace = self.get_namespace()
        
        # If drone_id parameter is not set (default -1), extract from namespace
        if self.drone_id < 0:
            if self.namespace == '/':
                self.get_logger().fatal('Namespace not specified and drone_id parameter not set!')
                raise RuntimeError('Cannot determine drone ID')
                
            # Extract numeric ID from namespace
            try:
                self.drone_id = int(self.namespace.split('_')[-1])
            except (ValueError, IndexError) as e:
                self.get_logger().fatal(f'Failed to extract drone ID from namespace: {e}')
                raise RuntimeError('Invalid namespace format, expected /px4_N')
        
        self.drone_id_str = str(self.drone_id)
        
        # If namespace wasn't specified but drone_id was, construct namespace
        if self.namespace == '/' and self.drone_id >= 0:
            self.namespace = f'/px4_{self.drone_id}'
            self.get_logger().info(f'Namespace not specified, using constructed namespace: {self.namespace}')
        
        self.get_logger().info(f'Initializing controller for drone {self.drone_id} (namespace: {self.namespace})')

    def get_spawn_position(self):
        """
        Get this drone's spawn position from the YAML file.
        
        The spawn position is used by offboard_control_px4 to convert between global NED 
        and local NED coordinates. This is important for takeoff operations where 
        we want to use local coordinates.
        
        Returns:
            tuple: (x, y) spawn position of this drone in global NED coordinates
        """
        default_spawn = (0.0, 0.0)
        
        try:
            yaml_file = self.get_parameter('spawn_position_file').value
            yaml_paths = []
            
            # 1. Try package share directory first
            try:
                package_dir = get_package_share_directory('offboard_control_py')
                config_path = os.path.join(package_dir, 'config', yaml_file)
                if os.path.exists(config_path):
                    yaml_paths.append(config_path)
            except Exception as e:
                self.get_logger().debug(f'Error finding package share directory: {str(e)}')
            
            # 2. Go up to ros2_ws, then down into src and search for offboard_control_py/config/yaml_file
            current_dir = os.path.dirname(os.path.abspath(__file__))
            while current_dir != '/' and os.path.basename(current_dir) != 'ros2_ws':
                current_dir = os.path.dirname(current_dir)
                
            if os.path.basename(current_dir) == 'ros2_ws':
                src_dir = os.path.join(current_dir, 'src')
                # Recursively search for offboard_control_py/config/yaml_file under src
                for root, dirs, files in os.walk(src_dir):
                    if os.path.basename(root) == 'config' and os.path.exists(os.path.join(root, yaml_file)):
                        yaml_paths.append(os.path.join(root, yaml_file))
            
            # 3. Absolute path (if user gave one)
            if os.path.isabs(yaml_file):
                yaml_paths.append(yaml_file)
            
            # 4. Current directory (last resort)
            yaml_paths.append(yaml_file)
            
            # Try each path until we find the file
            yaml_path = None
            for path in yaml_paths:
                if os.path.exists(path):
                    yaml_path = path
                    self.get_logger().info(f'Found spawn position YAML file at: {yaml_path}')
                    break
            
            if yaml_path is None:
                self.get_logger().warning('Spawn position YAML file not found in any searched locations')
                return default_spawn
            
            # Read and parse the YAML file
            with open(yaml_path, 'r') as file:
                spawn_data = yaml.safe_load(file)
            
            if spawn_data is None or not isinstance(spawn_data, dict):
                self.get_logger().warning('Invalid YAML file format - root should be a dictionary')
                return default_spawn
            
            # Look for this drone's ID in all teams
            for team_id, team_data in spawn_data.items():
                if isinstance(team_data, dict) and self.drone_id_str in team_data:
                    drone_data = team_data[self.drone_id_str]
                    if isinstance(drone_data, dict):
                        x = float(drone_data.get('x', 0.0))
                        y = float(drone_data.get('y', 0.0))
                        self.get_logger().info(f'Found spawn position for drone {self.drone_id_str} in team {team_id}: x={x}, y={y}')
                        return (x, y)
            
            self.get_logger().warning(f'Drone ID {self.drone_id_str} not found in any team in spawn position YAML')
            return default_spawn
            
        except Exception as e:
            self.get_logger().error(f'Error reading spawn position from YAML: {str(e)}')
            return default_spawn

    def setup_game_master_interface(self):
        """Set up all interfaces with the game master."""
        
        # Detection subscriber
        self.detections_subscription = self.create_subscription(
            Detections,
            f'{self.namespace}/detections',
            self.detection_callback,
            10,
            callback_group=self.subscriber_group
        )
        
        # Health subscriber
        self.health_subscription = self.create_subscription(
            Int32,
            f'{self.namespace}/health',
            self.health_callback,
            10,
            callback_group=self.subscriber_group
        )
        
        # Message communication
        self.message_publisher = self.create_publisher(
            String,
            f'{self.namespace}/incoming_messages',
            10,
            callback_group=self.action_group
        )
        
        self.message_subscription = self.create_subscription(
            String,
            f'{self.namespace}/out_going_messages',
            self.message_callback,
            10,
            callback_group=self.subscriber_group
        )
        
        # Missile service client
        self.missile_client = self.create_client(
            Missile,
            '/game_master/fire_missile',
            callback_group=self.service_group
        )
        
        # Kamikaze service client
        self.kamikaze_client = self.create_client(
            Kamikaze,
            '/game_master/kamikaze',
            callback_group=self.service_group
        )
    
    #################################################
    # BEHAVIOR IMPLEMENTATION                       #
    # Implement your drone's behavior here          #
    #################################################
    
    def behavior_callback(self):
        """
        Main behavior control loop, called at regular intervals (5 Hz).
        
        This method dispatches to state-specific handlers based on the current state.
        Implement state handlers to define your drone's behavior.
        """
        # Check for state timeouts if not in COMPLETE state
        if self.state != self.STATE_COMPLETE:
            self.check_state_timeout()
            
        # Dispatch to the appropriate state handler
        if self.state == self.STATE_INIT:
            self.handle_init_state()
            
        elif self.state == self.STATE_TAKEOFF:
            self.handle_takeoff_state()
            
        elif self.state == self.STATE_MISSION:
            self.handle_mission_state()
            
        elif self.state == self.STATE_COMBAT:
            self.handle_combat_state()
            
        elif self.state == self.STATE_RTL:
            self.handle_rtl_state()
            
        elif self.state == self.STATE_COMPLETE:
            self.handle_complete_state()
    
    def check_state_timeout(self):
        """
        Check if the drone has been in the current state for too long.
        Override this with appropriate timeout behaviors for your mission.
        """
        current_time = time.time()
        time_in_state = current_time - self.state_changed_time
        
        # Example timeout checks - customize as needed
        if self.state == self.STATE_TAKEOFF and time_in_state > 30.0:
            self.get_logger().warning(f'Takeoff taking longer than expected ({time_in_state:.1f}s)')
            self.state_changed_time = current_time  # Reset timer to avoid repeated warnings
    
    def change_state(self, new_state):
        """
        Change to a new state with logging and timing.
        
        Args:
            new_state: The new state to transition to
        """
        old_state = self.state
        self.state = new_state
        self.state_changed_time = time.time()
        
        # Map state numbers to names for better logging
        state_names = {
            self.STATE_INIT: "INIT",
            self.STATE_TAKEOFF: "TAKEOFF",
            self.STATE_MISSION: "MISSION",
            self.STATE_COMBAT: "COMBAT",
            self.STATE_RTL: "RTL",
            self.STATE_COMPLETE: "COMPLETE"
        }
        
        self.get_logger().info(f'State transition: {state_names[old_state]} -> {state_names[new_state]}')
    
    def handle_init_state(self):
        """
        Handle the INIT state behavior.
        Implement your initialization logic here.
        """
        # Example: Transition to TAKEOFF after initialization
        # self.change_state(self.STATE_TAKEOFF)
        pass
    
    def handle_takeoff_state(self):
        """
        Handle the TAKEOFF state behavior.
        Implement your takeoff logic here.
        """
        # Example: Check if ready to take off, then execute takeoff
        # if not self.action_in_progress and self.estimated_position:
        #     height = 10.0  # Takeoff height in meters
        #     self.set_offboard_parameters(offboard_mode='position', coordinate_system='local_NED')
        #     success = self.navigate_to(
        #         self.estimated_position['x'],
        #         self.estimated_position['y'],
        #         -height,  # Negative is up in NED
        #         0.0
        #     )
        #     if success:
        #         self.get_logger().info(f'Takeoff command sent, climbing to {height}m')
        pass
    
    def handle_mission_state(self):
        """
        Handle the MISSION state behavior.
        Implement your main mission logic here.
        """
        # Example: Navigate to mission waypoints
        # if not self.action_in_progress:
        #     self.navigate_to(target_x, target_y, target_z, target_yaw)
        pass
    
    def handle_combat_state(self):
        """
        Handle the COMBAT state behavior.
        Implement your combat logic here.
        """
        # Example: Track enemies and fire missiles
        # if self.latest_detections:
        #     for detection in self.latest_detections:
        #         if not detection.is_friend:
        #             # React to enemy detection
        pass
    
    def handle_rtl_state(self):
        """
        Handle the RTL (Return To Launch) state behavior.
        Implement your return to base logic here.
        """
        # Example: Navigate back to spawn position
        # if not self.action_in_progress:
        #     x, y = self.spawn_position
        #     self.navigate_to(x, y, -5.0, 0.0)  # Return at 5m altitude
        pass
    
    def handle_complete_state(self):
        """
        Handle mission completion behavior.
        Implement your mission completion logic here.
        """
        # Initialize completion steps if needed
        if not hasattr(self, 'completion_phase'):
            self.completion_phase = 'initializing'
            self.completion_start_time = time.time()
            self.get_logger().info('Mission complete, determining next action')
        
        # Example completion phases:
        if self.completion_phase == 'initializing':
            # Choose next action based on configuration
            if self.return_to_base:
                self.get_logger().info('Returning to spawn position')
                self.completion_phase = 'returning'
            else:
                self.get_logger().info('Hovering at current position')
                self.completion_phase = 'hovering'
        
        elif self.completion_phase == 'returning':
            # Implement return to base logic
            pass
            
        elif self.completion_phase == 'hovering':
            # Just hover at current position
            pass
            
        elif self.completion_phase == 'landed':
            # Final shutdown procedures
            pass
    
    #################################################
    # MOVEMENT AND NAVIGATION METHODS               #
    # Use these methods to control drone movement   #
    #################################################
    
    def set_offboard_parameters(self, offboard_mode=None, coordinate_system=None):
        """
        Set parameters for offboard_control_px4 node.
        
        Args:
            offboard_mode: 'position' or 'velocity' (uses default if None)
            coordinate_system: 'local_NED', 'NED', 'FLU', etc. (uses default if None)
            
        Returns:
            bool: True if parameters were set successfully
        
        Example:
            # Set to velocity mode in FLU frame
            controller.set_offboard_parameters('velocity', 'FLU')
        """
        # Use provided values or defaults
        offboard_mode = offboard_mode or self.default_offboard_mode
        coordinate_system = coordinate_system or self.default_coordinate_system
        
        if not self.param_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Parameter service not available')
            return False
            
        # Create parameter request
        request = SetParameters.Request()
        
        # Add offboard_mode parameter
        mode_param = Parameter()
        mode_param.name = 'offboard_mode'
        mode_param.value.type = ParameterType.PARAMETER_STRING
        mode_param.value.string_value = offboard_mode
        request.parameters.append(mode_param)
        
        # Add coordinate_system parameter
        coord_param = Parameter()
        coord_param.name = 'coordinate_system'
        coord_param.value.type = ParameterType.PARAMETER_STRING
        coord_param.value.string_value = coordinate_system
        request.parameters.append(coord_param)
        
        # Send parameter update request
        future = self.param_client.call_async(request)
        future.add_done_callback(self.parameter_callback)
        
        self.get_logger().info(f'Setting offboard parameters: mode={offboard_mode}, coordinate_system={coordinate_system}')
        return True
    
    def navigate_to(self, x, y, z, yaw, hover_time=0.0):
        """
        Navigate to a specific position and orientation.
        
        Args:
            x, y, z: Position coordinates (in current coordinate system)
            yaw: Yaw angle in degrees
            hover_time: Time to hover at destination (seconds)
            
        Returns:
            bool: True if command was sent successfully
        
        Example:
            # Go to position x=10, y=20, z=-5, with 45 degree heading
            controller.navigate_to(10, 20, -5, 45)
            
            # Go to position and hover for 5 seconds
            controller.navigate_to(10, 20, -5, 90, hover_time=5.0)
            
            # Return to spawn position
            x, y = controller.spawn_position
            controller.navigate_to(x, y, -5, 90)
        """
        self.get_logger().info(f'Navigating to position: x={x}, y={y}, z={z}, yaw={yaw}')
        
        self.action_in_progress = True
        
        # Create goal message
        goal_msg = GotoPosition.Goal()
        goal_msg.target.position.x = float(x)
        goal_msg.target.position.y = float(y)
        goal_msg.target.position.z = float(z)
        goal_msg.target.yaw = float(yaw)
        
        # Send goal
        if not self.action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error('Action server not available')
            self.action_in_progress = False
            return False
        
        send_goal_future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)
        
        # Store hover time if provided
        if hover_time > 0.0:
            self._pending_hover_time = hover_time
            
        return True
    
    def send_velocity(self, vx, vy, vz, yaw_rate):
        """
        Send a velocity command to the drone.
        
        Args:
            vx, vy, vz: Velocity components (m/s)
            yaw_rate: Yaw rate in degrees/second
            
        Returns:
            bool: True if command was published
        
        Example:
            # Move forward at 2 m/s and turn right at 10 deg/s
            controller.send_velocity(2.0, 0.0, 0.0, 10.0)
            
            # Strafe right at 1 m/s
            controller.send_velocity(0.0, 1.0, 0.0, 0.0)
        """
        try:
            # Set appropriate parameters in offboard_control_px4
            self.set_offboard_parameters(offboard_mode='velocity', coordinate_system='FLU')
            
            msg = PointYaw()
            msg.position.x = float(vx)
            msg.position.y = float(vy)
            msg.position.z = float(vz)
            msg.yaw = float(yaw_rate)
            
            self.target_pose_publisher.publish(msg)
            self.get_logger().debug(f'Sending velocity: vx={vx}, vy={vy}, vz={vz}, yaw_rate={yaw_rate}')
            return True
        except Exception as e:
            self.get_logger().error(f'Error sending velocity command: {str(e)}')
            return False
    
    def return_to_spawn(self, altitude=-5.0, yaw=90.0):
        """
        Navigate back to the drone's spawn position in global NED coordinates.
        
        Args:
            altitude: Z coordinate for return height (negative is up)
            yaw: Yaw angle in degrees (default 90.0)
            
        Returns:
            bool: True if command was sent successfully
        
        Example:
            # Return to spawn at 5m altitude
            controller.return_to_spawn(-5.0)
        """
        x, y = self.spawn_position
        self.get_logger().info(f'Returning to spawn position: x={x}, y={y}, z={altitude}, yaw={yaw}')
        return self.navigate_to(x, y, altitude, yaw)
    
    def cancel_current_goal(self):
        """
        Cancel the current navigation goal.
        
        Returns:
            bool: True if cancellation was requested
        
        Example:
            # Stop current movement
            controller.cancel_current_goal()
        """
        if not self.goal_handle or not self.action_in_progress:
            return False
            
        self.get_logger().info('Canceling current goal')
        future = self.goal_handle.cancel_goal_async()
        future.add_done_callback(self.cancel_goal_callback)
        return True
    
    #################################################
    # COMBAT AND WEAPON SYSTEMS                     #
    # Use these methods for combat operations       #
    #################################################
    
    def fire_missile(self):
        """
        Fire a missile at the current target.
        
        Returns:
            bool: True if missile firing request was sent
        
        Example:
            # Fire a missile
            if controller.missile_count > 0:
                controller.fire_missile()
        """
        if not self.missile_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Missile service not available')
            return False
        
        request = Missile.Request()
        request.robot_name = self.namespace
        
        self.get_logger().info('Firing missile!')
        future = self.missile_client.call_async(request)
        future.add_done_callback(self.missile_callback)
        return True
    
    def execute_kamikaze(self):
        """
        Execute a kamikaze attack (self-destruct near enemy).
        
        Returns:
            bool: True if kamikaze request was sent
        
        Example:
            # Last resort attack when heavily damaged
            if controller.health < 20:
                controller.execute_kamikaze()
        """
        if not self.kamikaze_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Kamikaze service not available')
            return False
        
        request = Kamikaze.Request()
        request.robot_name = self.namespace
        
        self.get_logger().info('Executing kamikaze attack!')
        future = self.kamikaze_client.call_async(request)
        # No callback needed as drone will likely be destroyed
        return True
    
    #################################################
    # TEAM COMMUNICATION                            #
    # Use these methods to communicate with team    #
    #################################################
    
    def send_message(self, message):
        """
        Send a message to other team members.
        
        Args:
            message: String message to send
            
        Returns:
            bool: True if message was published
        
        Example:
            # Report enemy positions
            controller.send_message("Enemy at x=100, y=200")
            
            # Coordinate attack
            controller.send_message("Attacking target 3")
        """
        try:
            msg = String()
            msg.data = message
            self.message_publisher.publish(msg)
            self.get_logger().info(f'Sent message: {message}')
            return True
        except Exception as e:
            self.get_logger().error(f'Error sending message: {str(e)}')
            return False
    
    #################################################
    # UTILITY METHODS                               #
    # Helper functions for drone control            #
    #################################################
    
    def distance_to_point(self, x, y, z):
        """
        Calculate distance from current position to given point.
        
        Args:
            x, y, z: Target coordinates
            
        Returns:
            float: Distance in meters or None if current position unknown
        
        Example:
            # Check if close enough to target
            dist = controller.distance_to_point(10, 20, -5)
            if dist and dist < 3.0:
                print("Close to target!")
        """
        if not self.current_position:
            return None
            
        cx, cy, cz = self.current_position
        return math.sqrt((x - cx)**2 + (y - cy)**2 + (z - cz)**2)
    
    def distance_to_spawn(self):
        """
        Calculate distance from current position to spawn point.
        
        Returns:
            float: Distance in meters or None if current position unknown
        
        Example:
            # Check if close to home
            dist = controller.distance_to_spawn()
            if dist and dist < 5.0:
                print("Almost home!")
        """
        if not self.current_position or not self.spawn_position:
            return None
            
        cx, cy, cz = self.current_position
        sx, sy = self.spawn_position
        return math.sqrt((sx - cx)**2 + (sy - cy)**2)
    
    def has_enemy_detections(self):
        """
        Check if there are any enemy detections.
        
        Returns:
            bool: True if enemies are detected
            
        Example:
            # React to enemy presence
            if controller.has_enemy_detections():
                controller.change_state(controller.STATE_COMBAT)
        """
        return any(not d.is_friend for d in self.latest_detections)
    
    def find_closest_enemy(self):
        """
        Find the closest enemy in current detections.
        
        Returns:
            tuple: (detection, distance) or (None, None) if no enemies
            
        Example:
            # Target closest enemy
            enemy, distance = controller.find_closest_enemy()
            if enemy and distance < 50.0:
                # Close enough to engage
                print(f"Engaging enemy at {distance:.1f}m")
        """
        closest_enemy = None
        min_distance = float('inf')
        
        for detection in self.latest_detections:
            if not detection.is_friend:
                # Calculate 3D distance
                distance = math.sqrt(
                    detection.relative_position.x**2 + 
                    detection.relative_position.y**2 + 
                    detection.relative_position.z**2
                )
                
                if distance < min_distance:
                    min_distance = distance
                    closest_enemy = detection
        
        if closest_enemy:
            return (closest_enemy, min_distance)
        else:
            return (None, None)
    
    #################################################
    # INTERNAL CALLBACKS                            #
    # You don't need to call these directly         #
    #################################################
    
    def vehicle_local_position_callback(self, msg):
        """
        Handle vehicle local position updates for precise positioning.
        
        This callback stores the precise position from PX4's local position estimation,
        including position, velocity, and heading information.
        """
        # Store the precise position from PX4's local position estimation
        self.estimated_position = {
            'x': msg.x,
            'y': msg.y,
            'z': msg.z,
            'vx': msg.vx,
            'vy': msg.vy,
            'vz': msg.vz,
            'heading': msg.heading
        }
    
    def vehicle_attitude_callback(self, msg):
        """
        Handle vehicle attitude updates to track the drone's orientation.
        
        This callback stores the quaternion attitude data from PX4's estimator.
        """
        # Store the quaternion [w, x, y, z]
        self.current_attitude = [
            msg.q[0],  # w
            msg.q[1],  # x
            msg.q[2],  # y
            msg.q[3]   # z
        ]
    
    def parameter_callback(self, future):
        """Handle response from parameter service."""
        try:
            response = future.result()
            success_count = 0
            failure_count = 0
            for result in response.results:
                if result.successful:
                    success_count += 1
                else:
                    failure_count += 1
                    self.get_logger().warning(f'Failed to set parameter: {result.reason}')
            
            if failure_count > 0:
                self.get_logger().warning(f'Parameter update: {success_count} succeeded, {failure_count} failed')
            
        except Exception as e:
            self.get_logger().error(f'Error in parameter callback: {str(e)}')
    
    def goal_response_callback(self, future):
        """Handle the response from the action server."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warning('Goal rejected by action server')
            self.action_in_progress = False
            return
        
        self.get_logger().debug('Goal accepted by action server')
        self.goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)
    
    def goal_result_callback(self, future):
        """Handle the result of the action."""
        try:
            result = future.result().result
            success = result.success
            
            if success:
                self.get_logger().info('Goal succeeded')
                
                # Check if there's pending hover time
                if hasattr(self, '_pending_hover_time') and self._pending_hover_time > 0:
                    self.get_logger().info(f'Hovering for {self._pending_hover_time} seconds')
                    self.hover_start_time = time.time()
                    self.hover_time = self._pending_hover_time
                    delattr(self, '_pending_hover_time')
                    
                # Handle state transitions based on goal completion
                # (implement this based on your state machine logic)
            else:
                self.get_logger().warning('Goal failed')
            
            self.action_in_progress = False
            self.goal_handle = None
            
        except Exception as e:
            self.get_logger().error(f'Error in goal result callback: {str(e)}')
            self.action_in_progress = False
            self.goal_handle = None
    
    def feedback_callback(self, feedback_msg):
        """Handle feedback from the action server."""
        # Store position for future reference
        pos = feedback_msg.feedback.current_position.position
        self.current_position = (pos.x, pos.y, pos.z)
        
        # Only log occasionally to avoid flooding
        if hasattr(self, '_feedback_counter'):
            self._feedback_counter += 1
            if self._feedback_counter % 10 != 0:  # Log every 10th feedback
                return
        else:
            self._feedback_counter = 0
            
        distance = feedback_msg.feedback.distance_to_target
        self.get_logger().debug(f'Distance to target: {distance:.2f}m')
    
    def cancel_goal_callback(self, future):
        """Handle response to goal cancellation request."""
        try:
            future.result()
            self.get_logger().info('Goal cancellation request completed')
            self.action_in_progress = False
            self.goal_handle = None
        except Exception as e:
            self.get_logger().error(f'Error in cancel callback: {str(e)}')
    
    def detection_callback(self, msg):
        """Process detection information from game master."""
        if not msg.detections:
            return
            
        self.last_detection_time = time.time()
        
        # Store most recent detections for behavior implementations to use
        self.latest_detections = msg.detections
        
        # Log summary of detections
        friend_count = sum(1 for d in msg.detections if d.is_friend)
        enemy_count = len(msg.detections) - friend_count
        
        # Track if enemy count has changed
        changed = not hasattr(self, 'previous_enemy_count') or enemy_count != self.previous_enemy_count
        if changed and enemy_count > 0:
            self.get_logger().info(f'Detected: {friend_count} friends, {enemy_count} enemies')
            
        # Store the current count for next comparison
        self.previous_enemy_count = enemy_count
    
    def health_callback(self, msg):
        """Handle health updates from game master."""
        old_health = getattr(self, 'health', 1)
        self.health = msg.data
        
        # Only log significant health changes
        if self.health < old_health:
            self.get_logger().warning(f'Health changed: {old_health} -> {self.health}')
    
    def message_callback(self, msg):
        """Handle incoming messages from game master."""
        self.get_logger().info(f'Received message: {msg.data}')
        # Store most recent message for behavior implementations
        self.latest_message = msg.data
    
    def missile_callback(self, future):
        """Handle missile service response."""
        try:
            response = future.result()
            if response.has_fired:
                self.get_logger().info(f'Missile fired successfully, ammo remaining: {response.ammo}')
                self.missile_count = response.ammo
            else:
                self.get_logger().warning(f'Failed to fire missile: {response.message}')
                self.missile_count = response.ammo
        except Exception as e:
            self.get_logger().error(f'Error in missile callback: {str(e)}')


def main(args=None):
    """
    Main function to initialize and run the drone controller.
    """
    rclpy.init(args=args)
    
    try:
        # Create node with multithreaded executor
        controller = DroneController()
        executor = MultiThreadedExecutor()
        executor.add_node(controller)
        
        # Run the node
        try:
            executor.spin()
        finally:
            executor.shutdown()
            controller.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()