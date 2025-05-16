#!/usr/bin/env python3
"""
Drone 0 Controller

This module implements a controller for drone 0 that:
1. Takes off to 10m altitude
2. Navigates to position x=130, y=240
3. Waits for an enemy drone detection
4. Positions itself 20 meters in front of the enemy using position-based navigation
5. Sends a message with the enemy drone position and own position
6. Waits 1 second
7. Fires a missile
8. Returns to spawn position after mission completion

Usage:
    ros2 run offboard_control_py offboard_control_client_0 --ros-args -r __ns:=/px4_0
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
    Controller for drone 0 that implements a combat reconnaissance mission
    with position-based target alignment.
    """
    
    # State machine states
    STATE_TAKEOFF = 0
    STATE_NAVIGATE = 1
    STATE_SCAN = 2
    STATE_ALIGN = 3
    STATE_ATTACK = 4
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
        
        # Mission-specific variables
        self.state = self.STATE_TAKEOFF
        self.state_changed_time = time.time()
        self.target_position = (130.0, 240.0, -10.0, 90.0)  # x, y, z, yaw
        self.return_to_base = self.get_parameter('return_to_base').value
        self.message_sent = False
        self.missile_fired = False
        self.alignment_attempts = 0
        self.target_enemy = None
        self.alignment_complete = False
        
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
        if self.state == self.STATE_TAKEOFF:
            self.handle_takeoff_state()
        
        elif self.state == self.STATE_NAVIGATE:
            self.handle_navigate_state()
            
        elif self.state == self.STATE_SCAN:
            self.handle_scan_state()
            
        elif self.state == self.STATE_ALIGN:
            self.handle_align_state()
            
        elif self.state == self.STATE_ATTACK:
            self.handle_attack_state()
            
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
            
        elif self.state == self.STATE_NAVIGATE and time_in_state > 60.0:
            self.get_logger().warning(f'Navigation taking longer than expected ({time_in_state:.1f}s), '
                                    f'waiting for completion...')
            # Don't force state changes, just provide warnings
            
        elif self.state == self.STATE_SCAN and time_in_state > 120.0:
            self.get_logger().warning(f'Been scanning for {time_in_state:.1f}s with no detections')
            if not self.latest_detections:
                self.get_logger().info('No detections after extended scan, proceeding to completion')
                self.change_state(self.STATE_COMPLETE)
                
        elif self.state == self.STATE_ALIGN and time_in_state > 45.0:
            self.get_logger().warning(f'Alignment taking longer than expected ({time_in_state:.1f}s)')
            # Let alignment complete naturally through goal_result_callback
    
    def change_state(self, new_state):
        """Change to a new state with logging and timing."""
        old_state = self.state
        self.state = new_state
        self.state_changed_time = time.time()
        
        # Map state numbers to names for better logging
        state_names = {
            self.STATE_TAKEOFF: "TAKEOFF",
            self.STATE_NAVIGATE: "NAVIGATE",
            self.STATE_SCAN: "SCAN",
            self.STATE_ALIGN: "ALIGN",
            self.STATE_ATTACK: "ATTACK",
            self.STATE_COMPLETE: "COMPLETE"
        }
        
        self.get_logger().info(f'State transition: {state_names[old_state]} -> {state_names[new_state]}')
    
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
                    self.get_logger().info('Waiting for initial position estimate to take off...')
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
    
    def handle_scan_state(self):
        """Handle the SCAN state behavior."""
        if not hasattr(self, '_scan_log_timer') or time.time() - self._scan_log_timer > 5.0:
            self.get_logger().info('Scanning for enemy drones...')
            self._scan_log_timer = time.time()
            
        # Check for enemies - detection callback will trigger state change when an enemy is found
    
    def handle_align_state(self):
        """
        Handle the ALIGN state - position the drone 20 meters in front of the enemy.
        Using the heading-aligned detection system for direct FRD positioning.
        """
        # If we're already navigating, wait for completion
        if self.action_in_progress:
            return
        
        # Check if enemy is already properly aligned (within 2m margin)
        for detection in self.latest_detections:
            if not detection.is_friend:
                rel_f = detection.relative_position.x
                rel_r = detection.relative_position.y
                rel_d = detection.relative_position.z
                
                # Check if enemy is at ideal position (20m in front with 2m margin)
                if (abs(rel_f - 20.0) <= 2.0 and 
                    abs(rel_r) <= 2.0 and 
                    abs(rel_d) <= 2.0):
                    self.get_logger().info('Enemy aligned at 20m in front (±2m), proceeding to attack')
                    self.alignment_complete = True
                    self.change_state(self.STATE_ATTACK)
                    return
        
        # If too many alignment attempts, just go to attack state
        if self.alignment_attempts >= 5:
            self.get_logger().info(f'Made {self.alignment_attempts} alignment attempts, proceeding to attack')
            self.alignment_complete = True
            self.change_state(self.STATE_ATTACK)
            return
                
        # Find enemies in latest detections
        enemy_found = False
        closest_enemy = None
        min_distance = float('inf')
        
        for detection in self.latest_detections:
            if not detection.is_friend:
                # Calculate distance
                distance = math.sqrt(
                    detection.relative_position.x**2 + 
                    detection.relative_position.y**2 + 
                    detection.relative_position.z**2
                )
                if distance < min_distance:
                    min_distance = distance
                    closest_enemy = detection
                    enemy_found = True
        
        if not enemy_found:
            self.get_logger().warning('No enemies detected for alignment, retrying...')
            return
        
        # Get detected enemy position in heading-aligned FRD
        rel_f = closest_enemy.relative_position.x
        rel_r = closest_enemy.relative_position.y
        rel_d = closest_enemy.relative_position.z
        
        # Log the detected enemy position
        self.get_logger().info(f'Alignment attempt {self.alignment_attempts+1}')
        self.get_logger().info(f'Enemy position (heading-aligned FRD): F={rel_f:.1f}, R={rel_r:.1f}, D={rel_d:.1f}')
        
        # SIMPLIFIED APPROACH: Use direct FRD positioning
        self.set_offboard_parameters(offboard_mode='position', coordinate_system='FRD')
        time.sleep(0.1)
        
        # Calculate our desired position - we want to be 20m in front of the target
        # If target is at F=50, R=10, we need to move to F=30, R=10 (50-20, 10)
        goal_f = max(0.0, rel_f - 20.0)  # Keep positive values only
        goal_r = rel_r                   # Keep same lateral position
        goal_d = rel_d                   # Keep same vertical position
        
        # Adjust for very close or very far targets
        if rel_f < 15.0:  # If too close, back up to give space
            self.get_logger().info(f'Target too close at {rel_f:.1f}m, backing away')
            goal_f = -5.0  # Back up 5 meters
        elif rel_f > 100.0:  # If very far, move halfway to avoid large movements
            self.get_logger().info(f'Target far away at {rel_f:.1f}m, moving halfway')
            goal_f = rel_f / 2
        
        self.get_logger().info(f'Positioning in FRD: F={goal_f:.1f}, R={goal_r:.1f}, D={goal_d:.1f}')
        success = self.navigate_to(goal_f, goal_r, goal_d, 0.0)
        
        if success:
            self.action_in_progress = True
            self.alignment_attempts += 1
            self.get_logger().info('Alignment navigation goal sent')
        else:
            self.get_logger().error('Failed to send navigation command for alignment')
    
    def calculate_alignment_position(self, target, standoff_distance=20.0):
        """
        Calculate a position that places the drone at the specified standoff distance
        in front of the target enemy, using the heading-aligned detection system.
        
        This simplified version takes advantage of the vertical axis always representing
        the true vertical direction regardless of drone attitude.
        
        Args:
            target: Detection message with enemy position
            standoff_distance: How far to position from the enemy (meters)
            
        Returns:
            tuple: (x, y, z, yaw) in NED coordinates or None if calculation fails
        """
        try:
            # Get current drone position in NED
            if not self.estimated_position:
                self.get_logger().warning('No position estimate available for alignment calculation')
                return None
                
            drone_n = self.estimated_position['x']
            drone_e = self.estimated_position['y']
            drone_d = self.estimated_position['z']
            drone_heading = self.estimated_position['heading']
            
            # Get enemy position in FRD (Forward-Right-Down) relative to drone's heading
            rel_f = target.relative_position.x  # Forward
            rel_r = target.relative_position.y  # Right
            rel_d = target.relative_position.z  # Down
            
            # Calculate horizontal distance to enemy
            horiz_distance = math.sqrt(rel_f**2 + rel_r**2)
            
            # Convert FRD to NED coordinates using drone's heading
            sin_heading = math.sin(drone_heading)
            cos_heading = math.cos(drone_heading)
            
            # Rotate the relative position from FRD to NED
            rel_n = rel_f * cos_heading - rel_r * sin_heading
            rel_e = rel_f * sin_heading + rel_r * cos_heading
            
            # Calculate enemy absolute position in NED
            enemy_n = drone_n + rel_n
            enemy_e = drone_e + rel_e
            enemy_d = drone_d + rel_d
            
            # Calculate direction vector from enemy to drone in horizontal plane
            dir_n = drone_n - enemy_n 
            dir_e = drone_e - enemy_e
            
            # Normalize direction vector for horizontal positioning
            dir_magnitude = math.sqrt(dir_n**2 + dir_e**2)
            if dir_magnitude < 0.001:  # Avoid division by zero
                # If directly above/below enemy, use drone's current heading
                unit_n = math.cos(drone_heading)
                unit_e = math.sin(drone_heading)
            else:
                unit_n = dir_n / dir_magnitude
                unit_e = dir_e / dir_magnitude
            
            # Calculate position at standoff distance from enemy
            target_n = enemy_n + unit_n * standoff_distance
            target_e = enemy_e + unit_e * standoff_distance
            
            # For altitude, match enemy's altitude but ensure minimum safe altitude
            min_altitude = -3.0  # Minimum 3m above ground (in NED, negative is up)
            target_d = min(enemy_d, min_altitude)
            
            # Calculate yaw to face the enemy (direction from target to enemy)
            target_yaw = math.atan2(enemy_e - target_e, enemy_n - target_n)
            target_yaw_deg = math.degrees(target_yaw)
            
            self.get_logger().info(f'Calculated alignment position using heading-aligned detections')
            self.get_logger().info(f'Enemy at: N={enemy_n:.1f}, E={enemy_e:.1f}, D={enemy_d:.1f}')
            self.get_logger().info(f'Target at: N={target_n:.1f}, E={target_e:.1f}, D={target_d:.1f}, Yaw={target_yaw_deg:.1f}')
            
            return (target_n, target_e, target_d, target_yaw_deg)
            
        except Exception as e:
            self.get_logger().error(f'Error calculating alignment position: {str(e)}')
            return None
    
    def handle_attack_state(self):
        """Execute the attack sequence."""
        # Check if we need to perform alignment first
        if not self.alignment_complete:
            self.get_logger().info('Alignment not complete, switching to ALIGN state')
            self.change_state(self.STATE_ALIGN)
            return
            
        # Check if enemy is still at the right position before firing
        enemy_ready_to_fire = False
        for detection in self.latest_detections:
            if not detection.is_friend:
                rel_f = detection.relative_position.x
                rel_r = detection.relative_position.y
                rel_d = detection.relative_position.z
                
                # Check if enemy is at ideal position (20m in front with 2m margin)
                if (abs(rel_f - 20.0) <= 2.0 and 
                    abs(rel_r) <= 2.0 and 
                    abs(rel_d) <= 2.0):
                    enemy_ready_to_fire = True
                    break
        
        if not enemy_ready_to_fire:
            self.get_logger().warning('Enemy no longer at optimal firing position, realigning')
            self.alignment_complete = False
            self.change_state(self.STATE_ALIGN)
            return
                
        # Send message with position information
        if not self.message_sent and self.estimated_position:
            # Find our target in the latest detections
            current_target = None
            for detection in self.latest_detections:
                if not detection.is_friend:
                    current_target = detection
                    break
                    
            if current_target:
                # Format enemy position in FRD coordinates
                enemy_f = current_target.relative_position.x
                enemy_r = current_target.relative_position.y
                enemy_d = current_target.relative_position.z
                
                # Format own position from precise position data
                own_x = self.estimated_position['x']
                own_y = self.estimated_position['y']
                own_z = self.estimated_position['z']
                
                # Send message with position information
                message = (
                    f"I see an enemy at (F={enemy_f:.1f}, R={enemy_r:.1f}, D={enemy_d:.1f}), "
                    f"and I am at (x={own_x:.1f}, y={own_y:.1f}, z={own_z:.1f})"
                )
                self.send_message(message)
                self.message_sent = True
                self.wait_start_time = time.time()
                self.get_logger().info('Message sent, waiting 1 second before firing missile')
                
        # Wait 1 second before firing missile
        if self.message_sent and not self.missile_fired:
            if self.wait_start_time and time.time() - self.wait_start_time >= 1.0:
                # Fire missile at enemy
                success = self.fire_missile()
                self.missile_fired = success
                
                # Move to COMPLETE state
                if success:
                    self.get_logger().info('Missile fired, mission complete')
                    self.change_state(self.STATE_COMPLETE)
    
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
            msg = PointYaw()
            msg.position.x = float(vx)
            msg.position.y = float(vy)
            msg.position.z = float(vz)
            msg.yaw = float(yaw_rate)
            
            self.target_pose_publisher.publish(msg)
            return True
        except Exception as e:
            self.get_logger().error(f'Error sending velocity command: {str(e)}')
            return False
    
    def cancel_current_goal(self):
        """Cancel the current navigation goal."""
        if not self.goal_handle or not self.action_in_progress:
            return False
            
        self.get_logger().info('Canceling current goal')
        future = self.goal_handle.cancel_goal_async()
        future.add_done_callback(self.cancel_goal_callback)
        return True
    
    #################################################
    # COMBAT AND WEAPON SYSTEMS                     #
    #################################################
    
    def fire_missile(self):
        """Fire a missile at the current target."""
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
            self.get_logger().info(f'Goal result: {success}')
            if success:
                self.get_logger().info('Goal succeeded')
                
                # State transitions based on completed goals
                if self.state == self.STATE_TAKEOFF:
                    self.change_state(self.STATE_NAVIGATE)
                    
                elif self.state == self.STATE_NAVIGATE:
                    self.change_state(self.STATE_SCAN)
                
                elif self.state == self.STATE_ALIGN:
                    self.action_in_progress = False
                    self.goal_handle = None
                    return
                
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
        pos = feedback_msg.feedback.current_position.position
        self.current_position = (pos.x, pos.y, pos.z)
    
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
        self.latest_detections = msg.detections
        
        # Count enemies
        enemy_count = sum(1 for d in msg.detections if not d.is_friend)
        
        # Log detections only if count has changed
        if not hasattr(self, 'previous_enemy_count') or enemy_count != self.previous_enemy_count:
            change_text = ""
            if hasattr(self, 'previous_enemy_count'):
                if enemy_count > self.previous_enemy_count:
                    change_text = f" (increased from {self.previous_enemy_count})"
                else:
                    change_text = f" (decreased from {self.previous_enemy_count})"
            
            if enemy_count > 0:
                self.get_logger().info(f'Detected {enemy_count} enemies{change_text}')
            elif hasattr(self, 'previous_enemy_count') and self.previous_enemy_count > 0:
                self.get_logger().info('No enemies detected')
        
        # Store the current count for next comparison
        self.previous_enemy_count = enemy_count
        
        # In SCAN state, transition to ALIGN state when enemy detected
        if self.state == self.STATE_SCAN and enemy_count > 0:
            for detection in msg.detections:
                if not detection.is_friend:
                    # Calculate distance
                    distance = math.sqrt(
                        detection.relative_position.x**2 + 
                        detection.relative_position.y**2 + 
                        detection.relative_position.z**2
                    )
                    
                    # Use a reasonable detection range
                    if distance < 80.0:  # 80m detection range
                        self.get_logger().info(
                            f'Enemy detected at ~{distance:.1f}m! '
                            f'Position: F={detection.relative_position.x:.1f}, '
                            f'R={detection.relative_position.y:.1f}, '
                            f'D={detection.relative_position.z:.1f}'
                        )
                        self.target_enemy = detection
                        self.change_state(self.STATE_ALIGN)
                        break
    
    def health_callback(self, msg):
        """Handle health updates from game master."""
        old_health = self.health
        self.health = msg.data
        
        if self.health < old_health:
            self.get_logger().warning(f'Health changed: {old_health} -> {self.health}')
    
    def message_callback(self, msg):
        """Handle incoming messages from game master."""
        self.get_logger().info(f'Received message: {msg.data}')
    
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