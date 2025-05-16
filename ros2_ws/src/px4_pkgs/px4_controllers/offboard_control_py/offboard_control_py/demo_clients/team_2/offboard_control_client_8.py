#!/usr/bin/env python3
"""
Drone 8 Controller

This module implements a controller for drone 8 that:
1. Takes off to 10m altitude
2. Navigates to position x=119.46, y=266.26, z=-10
3. Stays at that position

Usage:
    ros2 run offboard_control_py offboard_control_client_8 --ros-args -r __ns:=/px4_8
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
    Controller for drone 8 that positions it at a specific location.
    Uses improved state management and position tracking from offboard_control_client_0.
    """
    
    # State machine states
    STATE_INIT = 0
    STATE_TAKEOFF = 1
    STATE_NAVIGATE = 2
    STATE_WAIT = 3
    STATE_COMPLETE = 4
    
    def __init__(self):
        """Initialize the drone controller with all necessary connections."""
        super().__init__('drone_controller')
        
        # ====== CALLBACK GROUPS (for thread safety) ======
        self.action_group = MutuallyExclusiveCallbackGroup()
        self.service_group = MutuallyExclusiveCallbackGroup()
        self.timer_group = ReentrantCallbackGroup()
        self.subscriber_group = ReentrantCallbackGroup()
        
        # ====== PARAMETERS ======
        self.declare_parameter('offboard_mode', 'position')
        self.declare_parameter('coordinate_system', 'NED')
        self.declare_parameter('drone_id', -1)
        self.declare_parameter('spawn_position_file', 'spawn_position.yaml')
        self.declare_parameter('return_to_base', True)
        
        # ====== DRONE IDENTIFICATION ======
        self.setup_drone_identity()
        
        # ====== SPAWN POSITION ======
        self.spawn_position = self.get_spawn_position()
        self.get_logger().info(f'Spawn position: x={self.spawn_position[0]}, y={self.spawn_position[1]}')
        
        # ====== STATE TRACKING ======
        self.current_position = None      # Updated via action feedback
        self.estimated_position = None    # Updated via vehicle_local_position
        self.current_attitude = None      # Updated via vehicle_attitude
        self.last_detection_time = None
        self.health = 1
        self.team_id = 1 if self.drone_id <= 5 else 2
        self.missile_count = 2
        self.action_in_progress = False
        self.goal_handle = None
        self.latest_detections = []
        self.latest_message = None
        
        # Mission-specific variables
        self.state = self.STATE_INIT
        self.state_changed_time = time.time()
        self.target_position = (119.46, 266.26, -10.0, 0.0)  # x, y, z, yaw
        self.return_to_base = self.get_parameter('return_to_base').value
        self.position_refresh_interval = 60.0  # Refresh position command every minute
        self.last_position_refresh = 0.0
        self.wait_duration = 300.0  # 5 minutes of waiting before considering mission complete
        
        # ====== OFFBOARD CONTROL ======
        self.default_offboard_mode = self.get_parameter('offboard_mode').value
        self.default_coordinate_system = self.get_parameter('coordinate_system').value
        
        self.action_client = ActionClient(
            self,
            GotoPosition,
            f'{self.namespace}/goto_position',
            callback_group=self.action_group
        )
        
        self.target_pose_publisher = self.create_publisher(
            PointYaw,
            f'{self.namespace}/target_pose',
            10,
            callback_group=self.action_group
        )
        
        self.param_client = self.create_client(
            SetParameters,
            f'{self.namespace}/offboard_control_px4/set_parameters',
            callback_group=self.service_group
        )
        
        # Vehicle position subscription
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition,
            f'{self.namespace}/fmu/out/vehicle_local_position',
            self.vehicle_local_position_callback,
            rclpy.qos.qos_profile_sensor_data,
            callback_group=self.subscriber_group
        )
        
        # Vehicle attitude subscription
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
        self.behavior_timer = self.create_timer(
            0.2,  # 5 Hz
            self.behavior_callback,
            callback_group=self.timer_group
        )
        
        # Apply default parameters
        self.set_offboard_parameters(
            offboard_mode=self.default_offboard_mode,
            coordinate_system=self.default_coordinate_system
        )
        
        self.get_logger().info(f'Drone {self.drone_id} controller initialized (Team: {self.team_id})')

    def setup_drone_identity(self):
        """Determine drone ID and namespace from parameters or ROS namespace."""
        self.drone_id = self.get_parameter('drone_id').value
        self.namespace = self.get_namespace()
        
        if self.drone_id < 0:
            if self.namespace == '/':
                self.get_logger().fatal('Namespace not specified and drone_id parameter not set!')
                raise RuntimeError('Cannot determine drone ID')
                
            try:
                self.drone_id = int(self.namespace.split('_')[-1])
            except (ValueError, IndexError) as e:
                self.get_logger().fatal(f'Failed to extract drone ID from namespace: {e}')
                raise RuntimeError('Invalid namespace format, expected /px4_N')
        
        self.drone_id_str = str(self.drone_id)
        
        if self.namespace == '/' and self.drone_id >= 0:
            self.namespace = f'/px4_{self.drone_id}'
            self.get_logger().info(f'Namespace not specified, using constructed namespace: {self.namespace}')
        
        self.get_logger().info(f'Initializing controller for drone {self.drone_id} (namespace: {self.namespace})')

    def get_spawn_position(self):
        """Get this drone's spawn position from the YAML file."""
        default_spawn = (0.0, 0.0)
        
        try:
            yaml_file = self.get_parameter('spawn_position_file').value
            yaml_paths = []
            
            # 1. Try package share directory
            try:
                package_dir = get_package_share_directory('offboard_control_py')
                config_path = os.path.join(package_dir, 'config', yaml_file)
                if os.path.exists(config_path):
                    yaml_paths.append(config_path)
            except Exception:
                pass
            
            # 2. Search in ros2_ws
            current_dir = os.path.dirname(os.path.abspath(__file__))
            while current_dir != '/' and os.path.basename(current_dir) != 'ros2_ws':
                current_dir = os.path.dirname(current_dir)
                
            if os.path.basename(current_dir) == 'ros2_ws':
                src_dir = os.path.join(current_dir, 'src')
                for root, dirs, files in os.walk(src_dir):
                    if os.path.basename(root) == 'config' and os.path.exists(os.path.join(root, yaml_file)):
                        yaml_paths.append(os.path.join(root, yaml_file))
            
            # 3. Absolute path
            if os.path.isabs(yaml_file):
                yaml_paths.append(yaml_file)
            
            # 4. Current directory
            yaml_paths.append(yaml_file)
            
            # Try each path
            yaml_path = None
            for path in yaml_paths:
                if os.path.exists(path):
                    yaml_path = path
                    self.get_logger().info(f'Found spawn position YAML file at: {yaml_path}')
                    break
            
            if yaml_path is None:
                self.get_logger().warning('Spawn position YAML file not found in any searched locations')
                return default_spawn
            
            # Parse YAML
            with open(yaml_path, 'r') as file:
                spawn_data = yaml.safe_load(file)
            
            if spawn_data is None or not isinstance(spawn_data, dict):
                self.get_logger().warning('Invalid YAML file format - root should be a dictionary')
                return default_spawn
            
            # Find drone in teams
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
    #################################################
    
    def behavior_callback(self):
        """Main behavior control loop, defining the drone's state machine."""
        
        # Check for state timeouts
        if self.state != self.STATE_COMPLETE:
            self.check_state_timeout()
        
        # State machine
        if self.state == self.STATE_INIT:
            self.handle_init_state()
        
        elif self.state == self.STATE_TAKEOFF:
            self.handle_takeoff_state()
        
        elif self.state == self.STATE_NAVIGATE:
            self.handle_navigate_state()
            
        elif self.state == self.STATE_WAIT:
            self.handle_wait_state()
            
        elif self.state == self.STATE_COMPLETE:
            self.handle_complete_state()
    
    def check_state_timeout(self):
        """Check if we've been in the current state too long and take recovery action."""
        current_time = time.time()
        time_in_state = current_time - self.state_changed_time
        
        if self.state == self.STATE_INIT and time_in_state > 30.0:
            self.get_logger().warning(f'Initialization taking longer than expected ({time_in_state:.1f}s)')
            self.state_changed_time = current_time  # Reset timer to avoid spam
        
        elif self.state == self.STATE_TAKEOFF and time_in_state > 30.0:
            self.get_logger().warning(f'Takeoff taking longer than expected ({time_in_state:.1f}s), ' 
                                     f'waiting for offboard_control_px4 response...')
            # Reset the timer so we don't spam warnings
            self.state_changed_time = current_time
            
        elif self.state == self.STATE_NAVIGATE and time_in_state > 60.0:
            self.get_logger().warning(f'Navigation taking longer than expected ({time_in_state:.1f}s), '
                                     f'waiting for completion...')
            # Reset timer to avoid spam
            self.state_changed_time = current_time
            
        elif self.state == self.STATE_WAIT and time_in_state > self.wait_duration:
            self.get_logger().info(f'Waited at position for {time_in_state:.1f}s, mission complete')
            self.change_state(self.STATE_COMPLETE)
    
    def change_state(self, new_state):
        """Change to a new state with logging and timing."""
        old_state = self.state
        self.state = new_state
        self.state_changed_time = time.time()
        
        # Map state numbers to names for better logging
        state_names = {
            self.STATE_INIT: "INIT",
            self.STATE_TAKEOFF: "TAKEOFF",
            self.STATE_NAVIGATE: "NAVIGATE",
            self.STATE_WAIT: "WAIT",
            self.STATE_COMPLETE: "COMPLETE"
        }
        
        self.get_logger().info(f'State transition: {state_names[old_state]} -> {state_names[new_state]}')
    
    def handle_init_state(self):
        """Handle the INIT state behavior - wait for position estimate."""
        # Wait for position estimate before proceeding to takeoff
        if self.estimated_position:
            self.get_logger().info('Initial position estimate received, proceeding to takeoff')
            self.change_state(self.STATE_TAKEOFF)
        else:
            if not hasattr(self, '_init_wait_log_timer') or time.time() - self._init_wait_log_timer > 2.0:
                self.get_logger().info('Waiting for initial position estimate...')
                self._init_wait_log_timer = time.time()
    
    def handle_takeoff_state(self):
        """Handle the TAKEOFF state behavior."""
        if not self.action_in_progress:
            height = 10.0
            
            if self.estimated_position:
                self.set_offboard_parameters(offboard_mode='position', coordinate_system='local_NED')
                
                success = self.navigate_to(
                    self.estimated_position['x'],
                    self.estimated_position['y'],
                    -height,  # Negative is up in NED
                    0.0
                )
                
                if success:
                    self.get_logger().info(f'Takeoff command sent, climbing to {height}m')
                    self.action_in_progress = True
            else:
                if not hasattr(self, '_takeoff_wait_log_timer') or time.time() - self._takeoff_wait_log_timer > 2.0:
                    self.get_logger().info('Waiting for position estimate before takeoff...')
                    self._takeoff_wait_log_timer = time.time()
    
    def handle_navigate_state(self):
        """Handle the NAVIGATE state behavior."""
        if not self.action_in_progress:
            self.set_offboard_parameters(offboard_mode='position', coordinate_system='NED')
            
            time.sleep(0.1)
            
            x, y, z, yaw = self.target_position
            self.get_logger().info(f'Navigating to target position: ({x}, {y}, {z})')
            success = self.navigate_to(x, y, z, yaw)
            
            if success:
                self.action_in_progress = True
            else:
                self.get_logger().error('Failed to send navigation command')
    
    def handle_wait_state(self):
        """Handle the WAIT state behavior."""
        current_time = time.time()
        
        # Periodically report that we're waiting at position
        if not hasattr(self, '_wait_log_timer') or current_time - self._wait_log_timer > 30.0:
            time_in_state = current_time - self.state_changed_time
            self.get_logger().info(f'Waiting at position: x={self.target_position[0]}, y={self.target_position[1]}, z={self.target_position[2]} for {time_in_state:.1f}s')
            self._wait_log_timer = current_time
            
            # Send a status message
            self.send_message(f"Drone {self.drone_id} maintaining position at x={self.target_position[0]}, "
                             f"y={self.target_position[1]}, z={self.target_position[2]}.")
        
        # Periodically refresh position command to ensure we stay in place
        if not self.action_in_progress and current_time - self.last_position_refresh > self.position_refresh_interval:
            x, y, z, yaw = self.target_position
            self.get_logger().info('Refreshing position to maintain target coordinates')
            success = self.navigate_to(x, y, z, yaw)
            if success:
                self.last_position_refresh = current_time
        
        # Check for enemy drones and report
        self.check_for_enemies()
    
    def check_for_enemies(self):
        """Check for enemy drones and report their presence."""
        current_time = time.time()
        
        if not self.latest_detections or current_time - self.last_detection_time > 10.0:
            return
            
        # Count enemies and friends
        enemy_count = sum(1 for d in self.latest_detections if not d.is_friend)
        friend_count = len(self.latest_detections) - enemy_count
        
        # Only log if we have enemies and haven't recently
        if enemy_count > 0 and (not hasattr(self, '_enemy_log_time') or current_time - self._enemy_log_time > 30.0):
            # Find closest enemy
            closest_enemy = None
            min_distance = float('inf')
            
            for detection in self.latest_detections:
                if not detection.is_friend:
                    distance = math.sqrt(
                        detection.relative_position.x**2 + 
                        detection.relative_position.y**2 + 
                        detection.relative_position.z**2
                    )
                    if distance < min_distance:
                        min_distance = distance
                        closest_enemy = detection
            
            if closest_enemy:
                self.get_logger().info(
                    f'Detected enemy drone at distance {min_distance:.1f}m, '
                    f'relative position: F={closest_enemy.relative_position.x:.1f}, '
                    f'R={closest_enemy.relative_position.y:.1f}, '
                    f'D={closest_enemy.relative_position.z:.1f}'
                )
                
                # Send message about enemy if they're close
                if min_distance < 50.0:
                    self.send_message(
                        f"Drone {self.drone_id} reporting: Enemy drone detected at {min_distance:.1f}m distance. "
                        f"I am at the designated position."
                    )
            
            self._enemy_log_time = current_time
        
        # Also periodically report friendlies
        if friend_count > 0 and (not hasattr(self, '_friend_log_time') or current_time - self._friend_log_time > 60.0):
            self.get_logger().info(f'Detected {friend_count} friendly drones')
            self._friend_log_time = current_time
    
    def handle_complete_state(self):
        """Handle mission completion behavior."""
        # Initialize completion steps if needed
        if not hasattr(self, 'completion_phase'):
            self.completion_phase = 'initializing'
            self.completion_start_time = time.time()
            self.get_logger().info('Mission complete, determining next action')
        
        # Handle the different completion phases
        if self.completion_phase == 'initializing':
            # Choose next action based on configuration
            if self.return_to_base:
                self.get_logger().info('Returning to spawn position')
                self.completion_phase = 'returning_safe_altitude'
                self.set_offboard_parameters(offboard_mode='position', coordinate_system='NED')
            else:
                self.get_logger().info('Hovering at current position')
                self.completion_phase = 'hovering'
                self.set_offboard_parameters(offboard_mode='position', coordinate_system='NED')
        
        elif self.completion_phase == 'returning_safe_altitude':
            # First go to a safe altitude above spawn position
            if not self.action_in_progress:
                x, y = self.spawn_position
                safe_altitude = -10.0  # 10 meters above ground in NED
                
                # Check if we're already at safe altitude
                if self.estimated_position and abs(self.estimated_position['z'] - safe_altitude) < 2.0:
                    self.get_logger().info('Already at safe altitude, proceeding to final approach')
                    self.completion_phase = 'final_approach'
                else:
                    self.get_logger().info(f'Moving to safe altitude above spawn: x={x:.1f}, y={y:.1f}, z={safe_altitude}')
                    success = self.navigate_to(x, y, safe_altitude, 0.0)
                    
                    if success:
                        self.action_in_progress = True
                        self.completion_phase = 'final_approach'
                    else:
                        self.get_logger().error('Failed to navigate to safe altitude, switching to hover')
                        self.completion_phase = 'hovering'
        
        elif self.completion_phase == 'final_approach':
            # Now descend to landing position
            if not self.action_in_progress:
                x, y = self.spawn_position
                # Get ground level at spawn position (or use a safe default like -2.0)
                ground_altitude = -2.0  # 2 meters above ground
                
                self.get_logger().info(f'Final approach to landing site: x={x:.1f}, y={y:.1f}, z={ground_altitude}')
                success = self.navigate_to(x, y, ground_altitude, 0.0)
                
                if success:
                    self.action_in_progress = True
                    self.completion_phase = 'landing'
                else:
                    self.get_logger().error('Failed to initiate final approach, switching to hover')
                    self.completion_phase = 'hovering'
        
        elif self.completion_phase == 'landing':
            # If we've reached the spawn position, initiate landing
            if not self.action_in_progress:
                self.get_logger().info('Reached spawn position, initiating landing')
                
                # Change to local_NED coordinate system for landing
                self.set_offboard_parameters(offboard_mode='position', coordinate_system='local_NED')
                
                time.sleep(0.1)
                
                # Send landing command: maintain x,y and land (z=0)
                success = self.navigate_to(0.0, 0.0, 0.0, 0.0)
                
                if success:
                    self.get_logger().info('Landing command sent')
                    self.action_in_progress = True
                    self.completion_phase = 'landed'
                else:
                    self.get_logger().error('Failed to initiate landing, switching to hover')
                    self.completion_phase = 'hovering'
        
        elif self.completion_phase == 'landed':
            # Check if we've reached ground level
            if not self.action_in_progress:
                self.get_logger().info('Mission complete and landed successfully')
                self.completion_phase = 'shutdown'
        
        elif self.completion_phase == 'hovering':
            # Just continue hovering at current position
            if time.time() - self.completion_start_time > 10.0:
                if not hasattr(self, '_hover_log_timer') or time.time() - self._hover_log_timer > 30.0:
                    self.get_logger().info('Continuing to hover at current position')
                    self._hover_log_timer = time.time()
        
        elif self.completion_phase == 'shutdown':
            # Nothing to do, mission is complete
            pass
    
    #################################################
    # MOVEMENT AND NAVIGATION METHODS               #
    #################################################
    
    def set_offboard_parameters(self, offboard_mode=None, coordinate_system=None):
        """Set parameters for offboard_control_px4 node."""
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
        """Navigate to a specific position and orientation."""
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
        """Send a velocity command to the drone."""
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
    
    def fire_missile(self):
        """
        Request to fire a missile.
        
        Returns:
            bool: True if request was sent successfully
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
    
    def send_message(self, message):
        """Send a message to other team members."""
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
    # CALLBACKS                                     #
    #################################################
    
    def vehicle_local_position_callback(self, msg):
        """Handle vehicle local position updates for precise positioning."""
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
        """Handle vehicle attitude updates to track the drone's orientation."""
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
                
                # State transitions based on completed goals
                if self.state == self.STATE_TAKEOFF:
                    self.change_state(self.STATE_NAVIGATE)
                    
                elif self.state == self.STATE_NAVIGATE:
                    self.change_state(self.STATE_WAIT)
                    # Send a message indicating drone 8 is in position
                    self.send_message(f"Drone {self.drone_id} in position at x={self.target_position[0]}, "
                                     f"y={self.target_position[1]}, z={self.target_position[2]}.")
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
    
    def detection_callback(self, msg):
        """Process detection information from game master."""
        if not msg.detections:
            return
            
        self.last_detection_time = time.time()
        self.latest_detections = msg.detections
        
        # Process detections - count enemies and friends
        friend_count = sum(1 for d in msg.detections if d.is_friend)
        enemy_count = len(msg.detections) - friend_count
        
        # Only log occasionally or when count changes
        if not hasattr(self, 'previous_detection_counts'):
            self.previous_detection_counts = {'friend': 0, 'enemy': 0}
            
        if (friend_count != self.previous_detection_counts['friend'] or 
            enemy_count != self.previous_detection_counts['enemy']):
            self.get_logger().info(f'Detected: {friend_count} friends, {enemy_count} enemies')
            self.previous_detection_counts['friend'] = friend_count
            self.previous_detection_counts['enemy'] = enemy_count
    
    def health_callback(self, msg):
        """Handle health updates from game master."""
        old_health = self.health
        self.health = msg.data
        
        # Only log significant health changes
        if self.health < old_health:
            self.get_logger().warning(f'Health changed: {old_health} -> {self.health}')
            
            # If health drops below 50, send a message that we're damaged
            if self.health < 50 and old_health >= 50:
                self.send_message(f"Drone {self.drone_id} damaged! Health at {self.health}%")
                
            # If health drops below 30, consider mission complete to avoid destruction
            if self.health < 30:
                self.get_logger().warning('Health critically low, completing mission')
                self.change_state(self.STATE_COMPLETE)
                
            # If health drops to 0, we've been destroyed
            if self.health == 0:
                self.get_logger().error(f"Drone {self.drone_id} has been destroyed!")
    
    def message_callback(self, msg):
        """Handle incoming messages from game master."""
        self.latest_message = msg.data
        self.get_logger().info(f'Received message: {msg.data}')
        
        # If someone asks about our position, respond
        if f"drone {self.drone_id}" in msg.data.lower() and ("position" in msg.data.lower() or "where" in msg.data.lower()):
            if self.estimated_position:
                response = (
                    f"Drone {self.drone_id} reporting position: x={self.estimated_position['x']:.1f}, "
                    f"y={self.estimated_position['y']:.1f}, z={self.estimated_position['z']:.1f}. "
                    f"I am at the designated position."
                )
                self.send_message(response)
    
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
    """Main function to initialize and run the drone controller."""
    rclpy.init(args=args)
    
    try:
        # Create node with multithreaded executor
        controller = DroneController()
        executor = MultiThreadedExecutor()
        executor.add_node(controller)
        
        try:
            executor.spin()
        finally:
            executor.shutdown()
            controller.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()