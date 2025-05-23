#!/usr/bin/env python3
"""
Drone 2 Controller

This module implements a controller for drone 2 that:
1. Takes off to 3m altitude
2. Detects the friendly ship
3. Positions itself 2m above the ship and follows it
4. After 15 seconds, executes kamikaze

Usage:
    ros2 run offboard_control_py offboard_control_client_2 --ros-args -r __ns:=/px4_2
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

import yaml, os, math, time
from ament_index_python.packages import get_package_share_directory


class DroneController(Node):
    """
    Controller for drone 2 that follows the friendly ship and executes kamikaze.
    Uses improved state management and position tracking from offboard_control_client_0.
    """
    
    # State machine states
    STATE_INIT = 0
    STATE_TAKEOFF = 1
    STATE_FIND_SHIP = 2
    STATE_FOLLOW_SHIP = 3
    STATE_KAMIKAZE = 4
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
        self.friendly_ship_detection = None
        self.follow_start_time = None
        self.kamikaze_triggered = False
        self.last_position_update_time = 0
        self.position_update_interval = 2.0  # seconds between position updates
        
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
        
        elif self.state == self.STATE_FIND_SHIP:
            self.handle_find_ship_state()
            
        elif self.state == self.STATE_FOLLOW_SHIP:
            self.handle_follow_ship_state()
            
        elif self.state == self.STATE_KAMIKAZE:
            self.handle_kamikaze_state()
            
        elif self.state == self.STATE_COMPLETE:
            self.handle_complete_state()
    
    def check_state_timeout(self):
        """Check if we've been in the current state too long and take recovery action."""
        current_time = time.time()
        time_in_state = current_time - self.state_changed_time
        
        if self.state == self.STATE_TAKEOFF and time_in_state > 30.0:
            self.get_logger().warning(f'Takeoff taking longer than expected ({time_in_state:.1f}s), ' 
                                    f'waiting for offboard_control_px4 response...')
            # Reset the timer so we don't spam warnings
            self.state_changed_time = current_time
            
        elif self.state == self.STATE_FIND_SHIP and time_in_state > 60.0:
            self.get_logger().warning(f'Ship search taking longer than expected ({time_in_state:.1f}s), '
                                    f'continuing to search...')
            self.state_changed_time = current_time
        
        elif self.state == self.STATE_FOLLOW_SHIP and time_in_state > 30.0:
            # If we've been following for too long without executing kamikaze, check if detection is lost
            if self.follow_start_time and current_time - self.follow_start_time >= 15.0:
                if not self.friendly_ship_detection or current_time - self.last_detection_time > 5.0:
                    self.get_logger().warning('Ship detection lost after follow period, attempting kamikaze anyway')
                    self.change_state(self.STATE_KAMIKAZE)
    
    def change_state(self, new_state):
        """Change to a new state with logging and timing."""
        old_state = self.state
        self.state = new_state
        self.state_changed_time = time.time()
        
        # Map state numbers to names for better logging
        state_names = {
            self.STATE_INIT: "INIT",
            self.STATE_TAKEOFF: "TAKEOFF",
            self.STATE_FIND_SHIP: "FIND_SHIP",
            self.STATE_FOLLOW_SHIP: "FOLLOW_SHIP",
            self.STATE_KAMIKAZE: "KAMIKAZE",
            self.STATE_COMPLETE: "COMPLETE"
        }
        
        self.get_logger().info(f'State transition: {state_names[old_state]} -> {state_names[new_state]}')
    
    def handle_init_state(self):
        """Handle the INIT state behavior."""
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
            height = 3.0  # 3m takeoff height for drone 2
            
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
    
    def handle_find_ship_state(self):
        """Handle the FIND_SHIP state behavior."""
        # Wait for ship detection in detection_callback
        # If ship is detected, state will change to FOLLOW_SHIP
        if not self.friendly_ship_detection:
            if not hasattr(self, '_find_ship_log_timer') or time.time() - self._find_ship_log_timer > 5.0:
                self.get_logger().info('Scanning for friendly ship...')
                self._find_ship_log_timer = time.time()
                
                # Optional: We could implement a search pattern here if needed
        else:
            # Ship already detected, transition to follow state
            self.get_logger().info('Friendly ship detected, beginning follow operation')
            self.change_state(self.STATE_FOLLOW_SHIP)
            self.follow_start_time = time.time()
    
    def handle_follow_ship_state(self):
        """
        Handle the FOLLOW_SHIP state behavior.
        Position the drone 2m above the friendly ship and follow it.
        """
        # If ship detection is lost, try to reacquire
        if not self.friendly_ship_detection or time.time() - self.last_detection_time > 5.0:
            if not hasattr(self, '_lost_ship_log_timer') or time.time() - self._lost_ship_log_timer > 3.0:
                self.get_logger().warning('Lost ship detection, maintaining position and waiting for reacquisition')
                self._lost_ship_log_timer = time.time()
            return
                
        # Check if we've been following for 15 seconds and should execute kamikaze
        current_time = time.time()
        if self.follow_start_time and current_time - self.follow_start_time >= 15.0:
            self.get_logger().info('15 seconds elapsed following ship, transitioning to KAMIKAZE state')
            self.change_state(self.STATE_KAMIKAZE)
            return
            
        # Only update position periodically to avoid overloading the control system
        if not self.action_in_progress and current_time - self.last_position_update_time > self.position_update_interval:
            self.position_above_ship()
            self.last_position_update_time = current_time
    
    def handle_kamikaze_state(self):
        """Handle the KAMIKAZE state behavior."""
        # Only execute kamikaze once
        if not self.kamikaze_triggered:
            self.get_logger().info('Executing kamikaze attack!')
            success = self.execute_kamikaze()
            self.kamikaze_triggered = success
            
            # Even if kamikaze fails, we consider the mission complete
            if not success:
                self.get_logger().error('Failed to execute kamikaze, mission failed')
                
            self.change_state(self.STATE_COMPLETE)
    
    def handle_complete_state(self):
        """Handle mission completion behavior."""
        # Not much to do here - after kamikaze, the drone should be destroyed
        if not hasattr(self, '_complete_log_timer') or time.time() - self._complete_log_timer > 10.0:
            self.get_logger().info('Mission complete (kamikaze executed)')
            self._complete_log_timer = time.time()
    
    def position_above_ship(self):
        """
        Calculate and move to a position 2m above the friendly ship.
        Uses detection information to maintain position above the ship.
        """
        if not self.friendly_ship_detection or not self.estimated_position:
            return
            
        # Get ship's relative position (in FRD frame)
        ship_f = self.friendly_ship_detection.relative_position.x  # Forward
        ship_r = self.friendly_ship_detection.relative_position.y  # Right
        ship_d = self.friendly_ship_detection.relative_position.z  # Down
        
        # OPTION 1: Simpler approach using FRD coordinates directly
        # This is better for positioning relative to a moving target
        self.set_offboard_parameters(offboard_mode='position', coordinate_system='FRD')
        
        # We want to be directly above the ship (same F,R coordinates) but 2m higher (D-2)
        target_f = ship_f
        target_r = ship_r
        target_d = ship_d - 2.0  # 2m above ship in FRD (negative is up)
        
        self.get_logger().info(
            f'Positioning 2m above ship in FRD: '
            f'Ship at F={ship_f:.1f}, R={ship_r:.1f}, D={ship_d:.1f}, '
            f'Moving to F={target_f:.1f}, R={target_r:.1f}, D={target_d:.1f}'
        )
        
        # Move to the calculated position
        return self.navigate_to(target_f, target_r, target_d, 0.0)
    
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
    
    def execute_kamikaze(self):
        """
        Execute a kamikaze attack against the target.
        
        Returns:
            bool: True if request was sent successfully
        """
        if not self.kamikaze_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Kamikaze service not available')
            return False
            
        request = Kamikaze.Request()
        request.robot_name = self.namespace
        
        self.get_logger().info(f'Executing kamikaze from {self.namespace}!')
        
        future = self.kamikaze_client.call_async(request)
        # We don't need a callback as the drone will be destroyed
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
                    self.change_state(self.STATE_FIND_SHIP)
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
        
        # Find friendly ship (vehicle_type=SHIP, is_friend=True)
        ship_found = False
        for detection in msg.detections:
            if detection.vehicle_type == Detection.SHIP and detection.is_friend:
                self.friendly_ship_detection = detection
                ship_found = True
                
                # Note: relative_position is in FRD (Forward-Right-Down) coordinates 
                # from the drone's perspective
                if not hasattr(self, '_ship_position_log_timer') or time.time() - self._ship_position_log_timer > 5.0:
                    self.get_logger().info(
                        f'Friendly ship detected at relative position: '
                        f'F={detection.relative_position.x:.1f}, '
                        f'R={detection.relative_position.y:.1f}, '
                        f'D={detection.relative_position.z:.1f}'
                    )
                    self._ship_position_log_timer = time.time()
                
                # If we're in FIND_SHIP state, move to FOLLOW_SHIP state
                if self.state == self.STATE_FIND_SHIP:
                    self.change_state(self.STATE_FOLLOW_SHIP)
                    self.follow_start_time = time.time()
                    self.get_logger().info('Friendly ship found, positioning above it')
        
        # If we were tracking a ship and it's no longer detected, clear the reference
        if not ship_found and self.friendly_ship_detection is not None:
            self.get_logger().warning('Previously detected friendly ship is no longer visible')
            self.friendly_ship_detection = None
        
        # Count detections by type
        friend_count = sum(1 for d in msg.detections if d.is_friend)
        enemy_count = len(msg.detections) - friend_count
        
        # Log summary of detections (occasionally)
        if not hasattr(self, '_detection_log_timer') or time.time() - self._detection_log_timer > 10.0:
            self.get_logger().info(f'Detection summary: {friend_count} friends, {enemy_count} enemies')
            self._detection_log_timer = time.time()
    
    def health_callback(self, msg):
        """Handle health updates from game master."""
        old_health = getattr(self, 'health', 1)
        self.health = msg.data
        
        if self.health < old_health:
            self.get_logger().warning(f'Health changed: {old_health} -> {self.health}')
            
            # If health drops too low, consider executing kamikaze immediately
            if self.health < 30 and self.state == self.STATE_FOLLOW_SHIP:
                self.get_logger().warning('Health critically low, executing kamikaze immediately')
                self.change_state(self.STATE_KAMIKAZE)
    
    def message_callback(self, msg):
        """Handle incoming messages from game master."""
        message_text = msg.data
        self.latest_message = message_text
        self.get_logger().info(f'Received message: {message_text}')
        
        # If we receive a message from drone 0 that it has found an enemy, we could react
        # but for this mission we stay focused on the ship


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