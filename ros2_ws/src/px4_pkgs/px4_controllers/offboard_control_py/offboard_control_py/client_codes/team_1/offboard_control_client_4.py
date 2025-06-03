#!/usr/bin/env python3
"""
Drone 4 Controller - Following Mission

This module implements a controller for drone 4 that:
1. Takes off
2. Moves to a position behind the line of drones (-5,-2,-2), facing west (-90)
3. Finds the closest drone in front of it and follows 2m behind it
4. Maintains follow position, adjusting when target is >4m away

Usage:
    ros2 run offboard_control_py offboard_control_client_4 --ros-args -r __ns:=/px4_4
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
import threading


class DroneController(Node):
    """
    Controller for drone 4 that follows behind the closest drone in front of it.
    """
    
    # State machine states
    STATE_INIT = 0
    STATE_TAKEOFF = 1
    STATE_POST_TAKEOFF_WAIT = 2
    STATE_GOTO_START_POSITION = 3
    STATE_TRACK_DRONE = 4
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
        
        # ====== MISSION PARAMETERS ======
        self.takeoff_altitude = 2.0  # Meters
        self.follow_distance = 2.0   # Distance to maintain behind target (meters)
        self.max_distance = 4.0      # Maximum distance before repositioning
        self.position_update_interval = 2.0  # Seconds between position updates
        self.target_lost_timeout = 5.0  # Seconds before considering target lost
        self.west_heading = -90.0  # Degrees, west in local_NED coordinate system
        
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
        self.target_drone = None
        self.last_position_update_time = 0
        self.return_to_base = self.get_parameter('return_to_base').value
        
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
            
            # Search for YAML file in various locations
            try:
                package_dir = get_package_share_directory('offboard_control_py')
                config_path = os.path.join(package_dir, 'config', yaml_file)
                if os.path.exists(config_path):
                    yaml_paths.append(config_path)
            except Exception:
                pass
            
            # Look in ros2_ws directory structure
            current_dir = os.path.dirname(os.path.abspath(__file__))
            while current_dir != '/' and os.path.basename(current_dir) != 'ros2_ws':
                current_dir = os.path.dirname(current_dir)
                
            if os.path.basename(current_dir) == 'ros2_ws':
                src_dir = os.path.join(current_dir, 'src')
                for root, dirs, files in os.walk(src_dir):
                    if os.path.basename(root) == 'config' and os.path.exists(os.path.join(root, yaml_file)):
                        yaml_paths.append(os.path.join(root, yaml_file))
            
            # Also try absolute path and current directory
            if os.path.isabs(yaml_file):
                yaml_paths.append(yaml_file)
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
            f'{self.namespace}/out_going_messages',
            10,
            callback_group=self.action_group
        )
        
        self.message_subscription = self.create_subscription(
            String,
            f'{self.namespace}/incoming_messages',
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
            
        elif self.state == self.STATE_POST_TAKEOFF_WAIT:
            self.handle_post_takeoff_wait_state()
            
        elif self.state == self.STATE_GOTO_START_POSITION:
            self.handle_goto_start_position_state()
        
        elif self.state == self.STATE_TRACK_DRONE:
            self.handle_track_drone_state()
            
        elif self.state == self.STATE_COMPLETE:
            self.handle_complete_state()
    
    def check_state_timeout(self):
        """Check if we've been in the current state too long and take recovery action."""
        current_time = time.time()
        time_in_state = current_time - self.state_changed_time
        
        if self.state == self.STATE_TAKEOFF and time_in_state > 30.0:
            self.get_logger().warning(f'Takeoff taking longer than expected ({time_in_state:.1f}s), ' 
                                    f'waiting for offboard_control_px4 response...')
            self.state_changed_time = current_time
            
        elif self.state == self.STATE_GOTO_START_POSITION and time_in_state > 30.0:
            self.get_logger().warning(f'Moving to start position taking longer than expected ({time_in_state:.1f}s)')
            if time_in_state > 60.0:
                self.get_logger().warning('Timeout waiting for start position, proceeding to tracking anyway')
                self.action_in_progress = False
                self.change_state(self.STATE_TRACK_DRONE)
            
        elif self.state == self.STATE_TRACK_DRONE and time_in_state > 300.0:
            # After 5 minutes of tracking, consider mission complete
            self.get_logger().info(f'Tracked drones for {time_in_state:.1f}s, mission successful')
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
            self.STATE_POST_TAKEOFF_WAIT: "POST_TAKEOFF_WAIT", 
            self.STATE_GOTO_START_POSITION: "GOTO_START_POSITION",
            self.STATE_TRACK_DRONE: "TRACK_DRONE",
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
        """Handle the TAKEOFF state behavior - take off to 2m altitude."""
        if not self.action_in_progress:
            height = self.takeoff_altitude  # 2m takeoff height
            
            if self.estimated_position:
                self.set_offboard_parameters(offboard_mode='position', coordinate_system='local_NED')
                
                success = self.navigate_to(
                    0,  # Stay at current x position
                    0,  # Stay at current y position
                    -height,  # Negative is up in NED
                    0.0  # Keep current heading
                )
                
                if success:
                    self.get_logger().info(f'Takeoff command sent, climbing to {height}m')
                    self.action_in_progress = True
            else:
                if not hasattr(self, '_takeoff_wait_log_timer') or time.time() - self._takeoff_wait_log_timer > 2.0:
                    self.get_logger().info('Waiting for position estimate before takeoff...')
                    self._takeoff_wait_log_timer = time.time()
    
    def handle_post_takeoff_wait_state(self):
            """Wait for 15 seconds after takeoff before proceeding to start position."""
            current_time = time.time()
            wait_duration = 15.0  # seconds
            
            # Calculate time spent waiting
            time_elapsed = current_time - self.state_changed_time
            
            # Log the waiting progress periodically
            if not hasattr(self, '_wait_log_timer') or current_time - self._wait_log_timer > 3.0:
                self.get_logger().info(f'Post-takeoff wait: {time_elapsed:.1f}s / {wait_duration:.1f}s')
                self._wait_log_timer = current_time
            
            # Transition to start position state after waiting period
            if time_elapsed >= wait_duration:
                self.get_logger().info(f'Post-takeoff wait complete ({wait_duration} seconds)')
                self.change_state(self.STATE_GOTO_START_POSITION)

    def handle_goto_start_position_state(self):
        """
        Handle moving to start position behind the line of drones.
        Position: x=-5, y=-2, z=-2, heading=-90 (west) in local_NED coordinates.
        """
        if not self.action_in_progress:
            self.get_logger().info('Moving to start position behind drone line')
            
            # Use local_NED coordinates to position behind the line of drones
            self.set_offboard_parameters(offboard_mode='position', coordinate_system='local_NED')
            
            # Command to move to x=-5, y=-2, z=-2, looking west (heading -90)
            success = self.navigate_to(
                -5.0,  # 5m behind line of drones (in local NED, +X is north)
                -2.0,  # 2m to the left (in local NED, +Y is east)
                -2.0,  # 2m above ground (in local NED, -Z is up)
                self.west_heading  # -90 degrees (west)
            )
            
            if success:
                self.get_logger().info('Moving to start position command sent')
                self.action_in_progress = True
            else:
                self.get_logger().error('Failed to send start position command')
                time.sleep(1.0)  # Avoid rapid retries
    
    def handle_track_drone_state(self):
        """
        Handle tracking state - find the closest drone in front and follow it.
        If current distance > 4m, reposition to 2m behind the drone.
        """
        # Find the closest drone in front of us
        closest_drone = self.find_closest_drone_in_front()
        
        # If no drones detected, log status
        if not closest_drone:
            if not hasattr(self, '_no_drone_log_timer') or time.time() - self._no_drone_log_timer > 5.0:
                self.get_logger().info('No drones detected in front, continuing to scan')
                self._no_drone_log_timer = time.time()
            return
            
        # Store the current target drone
        self.target_drone = closest_drone
        
        # Calculate distance to target drone
        target_f = closest_drone.relative_position.x  # Forward
        target_r = closest_drone.relative_position.y  # Right
        distance = math.sqrt(target_f**2 + target_r**2)
        
        # If we're too far from the target (> 4m) or it's been a while since last update
        current_time = time.time()
        update_needed = (
            distance > self.max_distance or
            not self.action_in_progress and current_time - self.last_position_update_time > self.position_update_interval
        )
        
        if update_needed:
            # Track the closest drone (move to position 2m behind it)
            if not self.action_in_progress:
                success = self.track_closest_drone(closest_drone)
                
                if success:
                    self.action_in_progress = True
                    self.last_position_update_time = current_time
                    if not hasattr(self, '_tracking_log_timer') or current_time - self._tracking_log_timer > 10.0:
                        self.get_logger().info(f'Tracking drone at distance {distance:.1f}m')
                        self._tracking_log_timer = current_time
                else:
                    self.get_logger().error('Failed to send tracking command')
    
    def handle_complete_state(self):
        """Handle mission completion behavior."""
        # Initialize completion steps if needed
        if not hasattr(self, 'completion_phase'):
            self.completion_phase = 'initializing'
            self.completion_start_time = time.time()
            self.get_logger().info('Mission complete, determining next action')
        
        # Handle the different completion phases (landing, returning to base, etc.)
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
                safe_altitude = -3.0  # 3 meters above ground in NED
                
                # Check if we're already at safe altitude
                if self.estimated_position and abs(self.estimated_position['z'] - safe_altitude) < 1.0:
                    self.get_logger().info('Already at safe altitude, proceeding to final approach')
                    self.completion_phase = 'final_approach'
                else:
                    self.get_logger().info(f'Moving to safe altitude above spawn: x={x:.1f}, y={y:.1f}, z={safe_altitude}')
                    success = self.navigate_to(x, y, safe_altitude, 0.0)
                    
                    if success:
                        self.action_in_progress = True
                    else:
                        self.get_logger().error('Failed to navigate to safe altitude, switching to hover')
                        self.completion_phase = 'hovering'
        
        elif self.completion_phase == 'final_approach':
            # Now descend to landing position
            if not self.action_in_progress:
                x, y = self.spawn_position
                ground_altitude = -1.0  # 1 meter above ground
                
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
    
    def find_closest_drone_in_front(self):
        """
        Find the closest friendly drone that is in front of us.
        
        Returns:
            Detection: The closest drone in front or None if no drones detected
        """
        if not self.latest_detections:
            return None
            
        closest_drone = None
        min_distance = float('inf')
        
        for detection in self.latest_detections:
            if detection.vehicle_type == Detection.DRONE and detection.is_friend:
                # Get the drone's relative position
                rel_f = detection.relative_position.x  # Forward
                rel_r = detection.relative_position.y  # Right
                
                # Only consider drones in front of us (positive Forward value)
                if rel_f > 0.0:
                    # Calculate straight-line distance to this drone
                    distance = math.sqrt(rel_f**2 + rel_r**2)
                    
                    # If this is the closest drone so far, update our tracking
                    if distance < min_distance:
                        min_distance = distance
                        closest_drone = detection
        
        # Log info about the closest drone if we found one
        if closest_drone and (not hasattr(self, '_closest_log_timer') or 
                             time.time() - self._closest_log_timer > 5.0):
            rel_f = closest_drone.relative_position.x
            rel_r = closest_drone.relative_position.y
            rel_d = closest_drone.relative_position.z
            distance = math.sqrt(rel_f**2 + rel_r**2)
            
            self.get_logger().info(
                f'Closest drone detected at: F={rel_f:.1f}, R={rel_r:.1f}, D={rel_d:.1f}, '
                f'Distance: {distance:.1f}m'
            )
            self._closest_log_timer = time.time()
                
        return closest_drone
    
    def track_closest_drone(self, target_drone):
        """
        Move to position 2m behind the closest drone.
        
        Args:
            target_drone: The Detection object representing the target drone
            
        Returns:
            bool: True if command was sent successfully
        """
        if not target_drone:
            self.get_logger().warning('Cannot track - no target drone provided')
            return False
        
        # Get target drone's relative position (in FRD frame)
        target_f = target_drone.relative_position.x  # Forward
        target_r = target_drone.relative_position.y  # Right
        target_d = target_drone.relative_position.z  # Down
        
        # Calculate current distance
        current_distance = math.sqrt(target_f**2 + target_r**2)
        
        # Use FRD coordinates for simple following
        self.set_offboard_parameters(offboard_mode='position', coordinate_system='FRD')
        
        # Calculate how much to move
        # We want to position ourselves at the target drone's position, but 2m back (-2 in x/Forward)
        target_f_local = target_f - self.follow_distance
        target_r_local = target_r  # Match the target's right/left position
        target_d_local = target_d  # Match the target's altitude
        
        self.get_logger().info(
            f'Tracking drone at distance {current_distance:.1f}m: '
            f'Target at F={target_f:.1f}, R={target_r:.1f}, D={target_d:.1f}, '
            f'Moving to F={target_f_local:.1f}, R={target_r_local:.1f}, D={target_d_local:.1f}'
        )
        
        # Send movement command
        success = self.navigate_to(target_f_local, target_r_local, target_d_local, 0.0)
        
        if success:
            self.last_position_update_time = time.time()
        
        return success
    
    #################################################
    # MOVEMENT AND NAVIGATION METHODS               #
    #################################################
    
    def set_offboard_parameters(self, offboard_mode=None, coordinate_system=None):
        """Set parameters for offboard_control_px4 node and wait for confirmation."""
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
        
        self.get_logger().info(f'Setting offboard parameters: mode={offboard_mode}, coordinate_system={coordinate_system}')
        
        # Create a synchronization event for callback to signal
        if not hasattr(self, 'param_update_event'):
            self.param_update_event = threading.Event()
        self.param_update_event.clear()
        
        # Track parameter update status
        self.param_update_success = False
        self.param_update_values = None
        
        # Send parameter update request
        future = self.param_client.call_async(request)
        future.add_done_callback(self.parameter_sync_callback)
        
        # Wait for parameter update to complete (with timeout)
        if not self.param_update_event.wait(timeout=2.0):
            self.get_logger().error('Parameter update timed out after 2 seconds')
            return False
        
        # After event is set, check success status
        if not self.param_update_success:
            self.get_logger().error('Parameter update failed')
            return False
            
        # Add a small delay to ensure parameters are fully applied
        time.sleep(0.1)
        
        self.get_logger().info('Parameter update confirmed')
        return True

    def parameter_sync_callback(self, future):
        """Synchronous callback for parameter service."""
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
                self.param_update_success = False
            else:
                self.param_update_success = True
                self.param_update_values = {
                    'offboard_mode': response.results[0].successful, 
                    'coordinate_system': response.results[1].successful
                }
                
        except Exception as e:
            self.get_logger().error(f'Error in parameter callback: {str(e)}')
            self.param_update_success = False
            
        # Signal that the parameter update attempt is complete
        self.param_update_event.set()
    
    def navigate_to(self, x, y, z, yaw, hover_time=0.0):
        """Navigate to a specific position and orientation."""
        self.get_logger().info(f'Navigating to position: x={x}, y={y}, z={z}, yaw={yaw}')
        
        # Don't set action_in_progress yet - we need to ensure command is sent successfully
        
        # Create goal message
        goal_msg = GotoPosition.Goal()
        goal_msg.target.position.x = float(x)
        goal_msg.target.position.y = float(y)
        goal_msg.target.position.z = float(z)
        goal_msg.target.yaw = float(yaw)
        
        # Send goal
        if not self.action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error('Action server not available')
            return False
        
        # Now that everything is ready, mark action as in progress
        self.action_in_progress = True
        
        send_goal_future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)
        
        # Store hover time if provided
        if hover_time > 0.0:
            self._pending_hover_time = hover_time
            
        return True
    
    def return_to_spawn(self, altitude=-1.0, yaw=0.0):
        """
        Navigate back to the drone's spawn position.
        
        Args:
            altitude: Z coordinate for return height (negative is up)
            yaw: Yaw angle in degrees
            
        Returns:
            bool: True if command was sent successfully
        """
        x, y = self.spawn_position
        self.get_logger().info(f'Returning to spawn position: x={x}, y={y}, z={altitude}, yaw={yaw}')
        return self.navigate_to(x, y, altitude, yaw)
    
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
                    self.get_logger().info('Takeoff complete, beginning 15-second wait period')
                    self.change_state(self.STATE_POST_TAKEOFF_WAIT)
                
                elif self.state == self.STATE_GOTO_START_POSITION:
                    self.get_logger().info('Successfully reached start position, beginning drone tracking')
                    self.change_state(self.STATE_TRACK_DRONE)
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
        
        # Count detections by type
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
        old_health = getattr(self, 'health', 1)
        self.health = msg.data
        
        if self.health < old_health:
            self.get_logger().warning(f'Health changed: {old_health} -> {self.health}')
            
            # If health gets too low, consider mission completion to avoid destruction
            if self.health < 30 and self.state not in [self.STATE_COMPLETE]:
                self.get_logger().warning('Health critically low, completing mission')
                self.change_state(self.STATE_COMPLETE)
    
    def message_callback(self, msg):
        """Handle incoming messages from game master."""
        self.latest_message = msg.data
        self.get_logger().info(f'Received message: {msg.data}')
        
        # Optionally report our status if other drones ask
        if "drone 4" in msg.data.lower() and "status" in msg.data.lower():
            if self.state == self.STATE_TRACK_DRONE and self.target_drone:
                # Calculate current distance to target
                rel_f = self.target_drone.relative_position.x
                rel_r = self.target_drone.relative_position.y
                distance = math.sqrt(rel_f**2 + rel_r**2)
                
                self.send_message(
                    f"Drone 4 reporting status: Tracking drone at distance {distance:.1f}m. "
                    f"Current altitude: {-self.estimated_position['z']:.1f}m"
                )


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