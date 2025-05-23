#!/usr/bin/env python3
"""
Drone 3 Escort Controller

This module implements a robust controller for drone 3 that:
1. Takes off to 3m altitude
2. Detects and locates the friendly ship
3. Positions itself 10m to the East of the ship at ship's altitude
4. Maintains that position as the ship moves
5. Returns to base when mission complete or on low health

Usage:
    ros2 run offboard_control_py offboard_control_client_3 --ros-args -r __ns:=/px4_3
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
    Controller for drone 3 that positions it as an escort 10m East of the friendly ship.
    """
    
    # State machine states
    STATE_INIT = 0
    STATE_TAKEOFF = 1
    STATE_SEARCH = 2
    STATE_POSITION = 3
    STATE_MAINTAIN = 4
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
        
        # ====== MISSION CONFIGURATION ======
        self.takeoff_altitude = 3.0  # Meters
        self.standoff_distance = 10.0  # Distance to maintain from ship (meters)
        self.ship_update_interval = 10.0  # Seconds between position updates
        self.ship_lost_timeout = 10.0  # Seconds before considering ship lost
        self.position_threshold = 2.0  # Distance threshold for position maintenance (meters)
        
        # ====== STATE TRACKING ======
        self.state = self.STATE_INIT
        self.state_changed_time = time.time()
        self.action_in_progress = False
        self.goal_handle = None
        self.estimated_position = None  # From VehicleLocalPosition
        self.current_attitude = None    # From VehicleAttitude
        self.health = 1
        self.team_id = 1 if self.drone_id <= 5 else 2
        
        # Ship tracking
        self.friendly_ship = None
        self.ship_last_seen_time = None
        self.last_position_update_time = 0
        self.latest_detections = []
        
        # ====== CLIENT INTERFACES ======
        self.setup_action_clients()
        self.setup_publishers()
        self.setup_subscribers()
        self.setup_service_clients()
        
        # ====== BEHAVIOR TIMER ======
        self.behavior_timer = self.create_timer(
            0.2,  # 5 Hz
            self.behavior_callback,
            callback_group=self.timer_group
        )
        
        # ====== INITIALIZATION ======
        self.default_offboard_mode = self.get_parameter('offboard_mode').value
        self.default_coordinate_system = self.get_parameter('coordinate_system').value
        
        # Apply default parameters
        self.set_offboard_parameters(
            offboard_mode=self.default_offboard_mode,
            coordinate_system=self.default_coordinate_system
        )
        
        self.get_logger().info(f'Drone {self.drone_id} escort controller initialized (Team: {self.team_id})')

    #################################################
    # INITIALIZATION AND SETUP                      #
    #################################################
    
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

    def setup_action_clients(self):
        """Initialize action clients."""
        self.action_client = ActionClient(
            self,
            GotoPosition,
            f'{self.namespace}/goto_position',
            callback_group=self.action_group
        )

    def setup_publishers(self):
        """Initialize publishers."""
        self.target_pose_publisher = self.create_publisher(
            PointYaw,
            f'{self.namespace}/target_pose',
            10,
            callback_group=self.action_group
        )
        
        self.message_publisher = self.create_publisher(
            String,
            f'{self.namespace}/incoming_messages',
            10,
            callback_group=self.action_group
        )

    def setup_subscribers(self):
        """Initialize subscribers."""
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
        
        # Detection subscription
        self.detections_subscription = self.create_subscription(
            Detections,
            f'{self.namespace}/detections',
            self.detection_callback,
            10,
            callback_group=self.subscriber_group
        )
        
        # Health subscription
        self.health_subscription = self.create_subscription(
            Int32,
            f'{self.namespace}/health',
            self.health_callback,
            10,
            callback_group=self.subscriber_group
        )
        
        # Message subscription
        self.message_subscription = self.create_subscription(
            String,
            f'{self.namespace}/out_going_messages',
            self.message_callback,
            10,
            callback_group=self.subscriber_group
        )

    def setup_service_clients(self):
        """Initialize service clients."""
        self.param_client = self.create_client(
            SetParameters,
            f'{self.namespace}/offboard_control_px4/set_parameters',
            callback_group=self.service_group
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
    # STATE MACHINE IMPLEMENTATION                  #
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
        
        elif self.state == self.STATE_SEARCH:
            self.handle_search_state()
            
        elif self.state == self.STATE_POSITION:
            self.handle_position_state()
            
        elif self.state == self.STATE_MAINTAIN:
            self.handle_maintain_state()
            
        elif self.state == self.STATE_COMPLETE:
            self.handle_complete_state()
    
    def check_state_timeout(self):
        """Check if we've been in the current state too long and take recovery action."""
        current_time = time.time()
        time_in_state = current_time - self.state_changed_time
        
        # State-specific timeout handling
        if self.state == self.STATE_TAKEOFF and time_in_state > 30.0:
            self.get_logger().warning(f'Takeoff taking longer than expected ({time_in_state:.1f}s), '
                                     f'checking altitude for takeoff detection')
            
            # Check if we're actually at altitude despite not getting a callback
            if self.estimated_position and self.estimated_position['z'] < -2.0:  # We're at least 2m in the air
                self.get_logger().info(f'Takeoff confirmed via altitude check ({-self.estimated_position["z"]:.1f}m)')
                self.action_in_progress = False
                self.change_state(self.STATE_SEARCH)
            else:
                # Reset the timer so we don't spam warnings
                self.state_changed_time = current_time
                
        elif self.state == self.STATE_SEARCH and time_in_state > 60.0:
            self.get_logger().warning(f'Ship search taking longer than expected ({time_in_state:.1f}s)')
            # Reset to avoid repeated warnings
            self.state_changed_time = current_time + 30.0  # Add 30s before next warning
            
        elif self.state == self.STATE_POSITION and time_in_state > 45.0:
            self.get_logger().warning(f'Initial positioning taking longer than expected ({time_in_state:.1f}s)')
            
            # If we're not making progress, cancel the current goal and try again
            if self.action_in_progress and time_in_state > 60.0:
                self.get_logger().warning('Positioning timeout, canceling current goal and retrying')
                if self.goal_handle:
                    self.goal_handle.cancel_goal_async()
                self.action_in_progress = False
                # Give a little delay before retry
                time.sleep(0.5)
            
            # Reset to avoid repeated warnings
            self.state_changed_time = current_time + 15.0
            
        elif self.state == self.STATE_MAINTAIN:
            # Check if ship is lost for too long
            if self.friendly_ship is None:
                if self.ship_last_seen_time is not None:
                    ship_lost_duration = current_time - self.ship_last_seen_time
                    if ship_lost_duration > self.ship_lost_timeout:
                        self.get_logger().warning(f'Ship lost for {ship_lost_duration:.1f}s, returning to search')
                        self.change_state(self.STATE_SEARCH)
            else:
                # Reset last seen time when ship is visible
                self.ship_last_seen_time = current_time
    
    def change_state(self, new_state):
        """Change to a new state with logging and timing."""
        old_state = self.state
        self.state = new_state
        self.state_changed_time = time.time()
        
        # Map state numbers to names for better logging
        state_names = {
            self.STATE_INIT: "INIT",
            self.STATE_TAKEOFF: "TAKEOFF",
            self.STATE_SEARCH: "SEARCH",
            self.STATE_POSITION: "POSITION",
            self.STATE_MAINTAIN: "MAINTAIN",
            self.STATE_COMPLETE: "COMPLETE"
        }
        
        self.get_logger().info(f'State transition: {state_names[old_state]} -> {state_names[new_state]}')
        
        # Reset action flags on state transition
        if old_state != new_state:
            # Only reset if current goal is not in progress
            if not self.action_in_progress:
                self.goal_handle = None
    
    def handle_init_state(self):
        """Handle initialization state - wait for all systems to be ready."""
        # Wait for position estimate before proceeding to takeoff
        if self.estimated_position:
            self.get_logger().info('Initial position estimate received, proceeding to takeoff')
            self.change_state(self.STATE_TAKEOFF)
        else:
            if not hasattr(self, '_init_wait_log_timer') or time.time() - self._init_wait_log_timer > 2.0:
                self.get_logger().info('Waiting for initial position estimate...')
                self._init_wait_log_timer = time.time()
    
    def handle_takeoff_state(self):
        """Handle takeoff state - send takeoff command and wait for completion."""
        if not self.action_in_progress:
            if self.estimated_position:
                # Initialize takeoff tracking attributes if not already set
                current_time = time.time()
                if not hasattr(self, '_takeoff_start_time'):
                    self._takeoff_start_time = current_time
                    self._takeoff_attempt_count = 0
                
                # Set position parameters for takeoff
                self.set_offboard_parameters(offboard_mode='position', coordinate_system='local_NED')
                
                # Send takeoff command - stay in current x,y but climb to takeoff altitude
                self.get_logger().info(f'Initiating takeoff to {self.takeoff_altitude}m altitude')
                success = self.navigate_to(
                    self.estimated_position['x'],
                    self.estimated_position['y'],
                    -self.takeoff_altitude,  # Negative is up in NED
                    0.0  # North-facing
                )
                
                if success:
                    self.get_logger().info('Takeoff command sent')
                    self.action_in_progress = True
                    self._takeoff_attempt_count += 1
                else:
                    self.get_logger().error('Failed to send takeoff command')
                    # Add a retry delay
                    time.sleep(1.0)
            else:
                self.get_logger().warning('Cannot takeoff - no position estimate available')
                time.sleep(0.5)  # Avoid rapid retry
    
    def handle_search_state(self):
        """Handle search state - scan for the friendly ship."""
        current_time = time.time()
        
        # If ship is detected, transition to POSITION state
        if self.friendly_ship:
            self.get_logger().info('Friendly ship detected, transitioning to positioning')
            self.ship_last_seen_time = current_time
            self.change_state(self.STATE_POSITION)
            return
        
        # If not running an action, consider search maneuver
        if not self.action_in_progress:
            # If we've been searching for a while without success, consider changing altitude
            time_in_state = current_time - self.state_changed_time
            
            # Optional: Implement search pattern here if needed
            if time_in_state > 30.0 and not hasattr(self, '_altitude_change_executed'):
                self._altitude_change_executed = True
                
                # Adjust altitude to improve visibility
                if self.estimated_position:
                    target_altitude = -5.0  # Higher altitude for better visibility
                    
                    self.get_logger().info(f'Extended search time, climbing to {-target_altitude}m for better visibility')
                    self.set_offboard_parameters(offboard_mode='position', coordinate_system='NED')
                    
                    success = self.navigate_to(
                        self.estimated_position['x'],
                        self.estimated_position['y'],
                        target_altitude,
                        self.estimated_position['heading']
                    )
                    
                    if success:
                        self.action_in_progress = True
                    else:
                        self.get_logger().error('Failed to adjust search altitude')
        
        # Periodically log search status
        if not hasattr(self, '_search_log_timer') or current_time - self._search_log_timer > 5.0:
            self.get_logger().info('Searching for friendly ship...')
            self._search_log_timer = current_time
    
    def handle_position_state(self):
        """Handle positioning state - move to 10m East of the ship."""
        current_time = time.time()
        
        # Check if ship is still visible
        if not self.friendly_ship:
            if self.ship_last_seen_time:
                # If ship recently lost, wait briefly before taking action
                ship_lost_duration = current_time - self.ship_last_seen_time
                if ship_lost_duration > 3.0:  # Give 3 seconds grace period
                    self.get_logger().warning(f'Lost sight of ship for {ship_lost_duration:.1f}s while positioning')
                    if ship_lost_duration > self.ship_lost_timeout:
                        self.get_logger().warning('Ship lost for too long, returning to search')
                        self.change_state(self.STATE_SEARCH)
            else:
                # No record of ever seeing ship, back to search
                self.get_logger().warning('Ship target lost, returning to search')
                self.change_state(self.STATE_SEARCH)
            return
        
        # Reset ship visibility timer since we can see it
        self.ship_last_seen_time = current_time
        
        # If not currently executing a movement command, initiate positioning
        if not self.action_in_progress:
            # Calculate desired position 10m to the East of the ship
            self.get_logger().info('Initiating positioning to East of ship')
            success = self.position_east_of_ship()
            
            if success:
                self.action_in_progress = True
                
                # After successful positioning, transition to maintenance will happen
                # in the goal_result_callback when action completes successfully
            else:
                self.get_logger().error('Failed to send positioning command')
                # Add delay to avoid rapid retries
                time.sleep(1.0)
    
    def handle_maintain_state(self):
        """Handle maintenance state - maintain position 10m in front of ship."""
        current_time = time.time()
        
        # Check if ship is still visible
        if not self.friendly_ship:
            if self.ship_last_seen_time:
                # If ship recently lost, wait briefly before taking action
                ship_lost_duration = current_time - self.ship_last_seen_time
                if ship_lost_duration > 2.0:  # Give 2 seconds grace period
                    self.get_logger().warning(f'Lost sight of ship for {ship_lost_duration:.1f}s')
                    if ship_lost_duration > self.ship_lost_timeout:
                        self.get_logger().warning('Ship lost for too long, returning to search')
                        self.change_state(self.STATE_SEARCH)
            else:
                # No record of ever seeing ship, set timer
                self.ship_last_seen_time = current_time
            return
        
        # Reset ship visibility timer since we can see it
        self.ship_last_seen_time = current_time
        
        # Get current ship angle and distance to check position
        rel_f = self.friendly_ship.relative_position.x
        rel_r = self.friendly_ship.relative_position.y
        rel_d = self.friendly_ship.relative_position.z
        ship_angle = math.degrees(math.atan2(rel_r, rel_f))
        ship_distance = math.sqrt(rel_f**2 + rel_r**2)
        
        # Add a stabilization period after each successful position command
        # This lets the drone settle and get better sensor readings before making new adjustments
        stabilization_period = 2.0  # seconds
        if (hasattr(self, '_last_position_command_time') and 
            current_time - self._last_position_command_time < stabilization_period):
            # Still in stabilization period, don't issue new commands yet
            return
        
        # Check if we need to update position 
        # Make more aggressive distance error checks to prevent the ship from getting away
        misalignment = abs(ship_angle) > 5.0  # Angle error threshold increased
        distance_error = abs(ship_distance - 10.0)  # Distance error
        significant_distance_error = distance_error > 3.0  # Ship getting too far away
        
        # Different response based on distance error severity
        if ship_distance > 15.0:  # Ship significantly far away
            distance_urgency = True
        else:
            distance_urgency = False
        
        # Check if we need to reposition
        needs_update = (
            not self.action_in_progress and (
                current_time - self.last_position_update_time > self.ship_update_interval or
                self.ship_moved_significantly() or
                misalignment or
                significant_distance_error
            )
        )
        
        if needs_update:
            reason = ""
            if current_time - self.last_position_update_time > self.ship_update_interval:
                reason = "10-second position check"
            elif self.ship_moved_significantly():
                reason = "significant ship movement"
            elif misalignment:
                reason = f"misalignment with ship (angle: {ship_angle:.1f}°)"
            elif significant_distance_error:
                reason = f"significant distance error (current: {ship_distance:.1f}m, target: 10.0m)"
            else:
                reason = f"distance error (current: {ship_distance:.1f}m, target: 10.0m)"
                
            self.get_logger().info(f'Updating position due to {reason}')
            
            # If the ship is really getting away, make this a priority update
            if distance_urgency:
                self.get_logger().warning(f'Ship distance ({ship_distance:.1f}m) exceeds safe range, urgent repositioning')
            
            success = self.position_east_of_ship()
            
            if success:
                self.action_in_progress = True
                self.last_position_update_time = current_time
            else:
                self.get_logger().error('Failed to update position')
        
        # Periodically report status
        if not hasattr(self, '_maintain_log_timer') or current_time - self._maintain_log_timer > 30.0:
            if self.friendly_ship:
                self.get_logger().info(
                    f'Maintaining position 10m in front of ship. '
                    f'Current distance: {ship_distance:.1f}m, alignment: {ship_angle:.1f}°'
                )
            else:
                self.get_logger().info('Waiting for ship visibility')
            self._maintain_log_timer = current_time
    
    def handle_complete_state(self):
        """Handle mission completion - return to base if configured."""
        # Initialize completion phase on first entry
        if not hasattr(self, 'completion_phase'):
            self.completion_phase = 'initializing'
            self.completion_start_time = time.time()
            self.get_logger().info('Mission complete, initiating return sequence')
        
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
                safe_altitude = -5.0  # 5 meters above ground in NED
                
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
    
    #################################################
    # SHIP POSITIONING FUNCTIONS                    #
    #################################################
    
    def position_east_of_ship(self):
        """Position drone 10m directly in front of the ship, facing the ship."""
        if not self.friendly_ship or not self.estimated_position:
            self.get_logger().warning('Cannot position relative to ship - missing ship data or position')
            return False
        
        # Get ship's position relative to drone in FRD coordinates
        rel_f = self.friendly_ship.relative_position.x  # Forward
        rel_r = self.friendly_ship.relative_position.y  # Right
        rel_d = self.friendly_ship.relative_position.z  # Down
        
        # Calculate ship distance and angle
        ship_distance = math.sqrt(rel_f**2 + rel_r**2)
        ship_angle = math.degrees(math.atan2(rel_r, rel_f))
        
        # STEP 1: First turn to face the ship
        if self.state == self.STATE_POSITION or not hasattr(self, '_faced_ship'):
            face_ship_yaw = ship_angle
            
            self.get_logger().info(
                f'First turning to face the ship at angle {face_ship_yaw:.1f}° before approaching. '
                f'Ship at range: {ship_distance:.1f}m'
            )
            
            self.set_offboard_parameters(offboard_mode='position', coordinate_system='FRD')
            time.sleep(0.1)
            
            # Send command to rotate in place to face the ship
            success = self.navigate_to(0.0, 0.0, 0.0, face_ship_yaw)
            
            if success:
                self._faced_ship = True
                self._last_ship_position = {
                    'f': rel_f,
                    'r': rel_r,
                    'd': rel_d,
                    'time': time.time()
                }
                
                if self.state == self.STATE_POSITION:
                    # In positioning state, wait for rotation to complete before approaching
                    return True
            else:
                self.get_logger().error('Failed to send facing command')
                return False
        
        # STEP 2: Calculate position 10m in front of the ship    
        # Calculate unit vector toward ship
        ship_distance = math.sqrt(rel_f**2 + rel_r**2)
        if ship_distance < 0.1:  # Avoid division by zero
            unit_f, unit_r = 1.0, 0.0
        else:
            unit_f = rel_f / ship_distance
            unit_r = rel_r / ship_distance
        
        # Calculate position 10m in front of ship (in direction of ship)
        target_distance = 10.0  # Standoff distance (m)
        
        # Calculate whether we're too far or too close
        distance_error = ship_distance - target_distance
        align_only = abs(distance_error) < 0.5
        
        # If we're too close, move back; if we're too far, move forward
        if align_only:
            # We're at the right distance, just align
            target_f = 0.0
            target_r = 0.0
        else:
            # Calculate position that puts us at exactly 10m distance
            # Move along the direction to/from ship
            # If we're way too far, make a more aggressive approach
            if distance_error > 5.0:
                # More aggressive approach when far away
                distance_to_move = distance_error * 0.8
            else:
                distance_to_move = distance_error * 0.6
                
            target_f = distance_to_move * unit_f
            target_r = distance_to_move * unit_r
        
        # Stay at ship's altitude
        target_d = rel_d
        
        # Yaw should be aligned with ship (we're already facing it)
        target_yaw = 0.0  # Keep facing forward (toward ship)
        
        # Log message depends on whether we're just aligning or moving
        if align_only:
            self.get_logger().info(
                f'Maintaining position 10m in front of ship. Distance: {ship_distance:.1f}m, angle: {ship_angle:.1f}°'
            )
        else:
            self.get_logger().info(
                f'Moving to position 10m in front of ship: '
                f'Ship at FRD ({rel_f:.1f}, {rel_r:.1f}, {rel_d:.1f}), '
                f'Ship distance: {ship_distance:.1f}m, angle: {ship_angle:.1f}°, '
                f'Moving to position ({target_f:.1f}, {target_r:.1f}, {target_d:.1f})'
            )
        
        # Use smaller rotation corrections to avoid overshooting
        # Only do a major realignment if significantly off-axis
        if abs(ship_angle) > 10.0:
            # Need to turn to face ship again - major misalignment
            self.get_logger().info(f'Realigning to face ship (current angle: {ship_angle:.1f}°)')
            success = self.navigate_to(0.0, 0.0, 0.0, ship_angle)
        elif abs(ship_angle) > 2.0:
            # Minor alignment correction - use a smaller proportional correction
            # to avoid overshooting
            correction_factor = 0.5  # 50% correction to avoid oscillation
            adjusted_angle = ship_angle * correction_factor
            self.get_logger().info(f'Fine-tuning alignment with small correction: {adjusted_angle:.1f}°')
            success = self.navigate_to(0.0, 0.0, 0.0, adjusted_angle)
        elif align_only:
            # Already at correct distance and alignment
            self.get_logger().info('Already at optimal position and alignment')
            
            if self.state == self.STATE_POSITION:
                self.change_state(self.STATE_MAINTAIN)
                
            return True
        else:
            # Move to target position
            success = self.navigate_to(target_f, target_r, target_d, target_yaw)
        
        if success:
            self._last_ship_position = {
                'f': rel_f,
                'r': rel_r,
                'd': rel_d,
                'time': time.time()
            }
            
            # Set a timestamp for when this position command was sent
            # This will be used to enforce a stabilization period
            self._last_position_command_time = time.time()
        
        return success
    
    def ship_moved_significantly(self):
        """
        Check if the ship has moved enough to justify repositioning.
        
        Returns:
            bool: True if ship movement exceeds thresholds
        """
        if not self.friendly_ship or not hasattr(self, '_last_ship_position'):
            return False
        
        # Get current ship position
        rel_f = self.friendly_ship.relative_position.x
        rel_r = self.friendly_ship.relative_position.y
        rel_d = self.friendly_ship.relative_position.z
        
        # Calculate change in position
        delta_f = rel_f - self._last_ship_position['f']
        delta_r = rel_r - self._last_ship_position['r']
        delta_d = rel_d - self._last_ship_position['d']
        
        # Calculate horizontal distance change
        delta_horiz = math.sqrt(delta_f**2 + delta_r**2)
        
        # Calculate angle change
        last_angle = math.atan2(self._last_ship_position['r'], self._last_ship_position['f'])
        current_angle = math.atan2(rel_r, rel_f)
        delta_angle = abs(math.degrees(current_angle - last_angle))
        while delta_angle > 180:
            delta_angle = 360 - delta_angle
        
        # Determine if movement is significant
        significant_movement = (delta_horiz > 5.0 or  # More than 5m horizontal distance
                              delta_angle > 2.0 or  # More than 15 degrees angle change
                              abs(delta_d) > 1.0)    # More than 1m altitude change
        
        # Log significant movement
        if significant_movement:
            self.get_logger().info(
                f'Ship moved significantly: '
                f'Distance change: {delta_horiz:.1f}m, '
                f'Angle change: {delta_angle:.1f}°, '
                f'Altitude change: {delta_d:.1f}m'
            )
        
        return significant_movement
    
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
        """
        Navigate to a specific position and orientation.
        
        Args:
            x, y, z: Position coordinates (in current coordinate system)
            yaw: Orientation in degrees
            hover_time: Optional hover time at destination
            
        Returns:
            bool: True if navigation command was accepted
        """
        self.get_logger().info(f'Navigating to position: x={x}, y={y}, z={z}, yaw={yaw}')
        
        # Create goal message
        goal_msg = GotoPosition.Goal()
        goal_msg.target.position.x = float(x)
        goal_msg.target.position.y = float(y)
        goal_msg.target.position.z = float(z)
        goal_msg.target.yaw = float(yaw)
        
        # Wait for action server
        if not self.action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error('Action server not available')
            return False
        
        # Send goal
        send_goal_future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)
        
        # Store hover time if provided
        if hover_time > 0.0:
            self._pending_hover_time = hover_time
            
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
        """Handle vehicle local position updates."""
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
        """Handle vehicle attitude updates."""
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
        """Handle the result of the navigation action."""
        try:
            # Properly extract result
            response = future.result()
            result = response.result
            success = result.success
            
            if success:
                self.get_logger().info('Goal succeeded')
                
                # State-specific success handling
                if self.state == self.STATE_TAKEOFF:
                    self.get_logger().info('Takeoff complete, transitioning to search')
                    self.change_state(self.STATE_SEARCH)
                    
                elif self.state == self.STATE_POSITION:
                    self.get_logger().info('Initial positioning complete, transitioning to maintenance')
                    self.change_state(self.STATE_MAINTAIN)
            else:
                self.get_logger().warning('Goal failed')
                
                # State-specific failure handling could go here
            
            # Clear action flags
            self.action_in_progress = False
            
        except Exception as e:
            self.get_logger().error(f'Error in goal result callback: {str(e)}')
            self.action_in_progress = False
            self.goal_handle = None
    
    def feedback_callback(self, feedback_msg):
        """Handle feedback from the action server."""
        # Process feedback from navigation action
        feedback = feedback_msg.feedback
        
        # Only log occasionally to avoid flooding
        if hasattr(self, '_feedback_count'):
            self._feedback_count += 1
            if self._feedback_count % 10 != 0:  # Log every 10th feedback
                return
        else:
            self._feedback_count = 0
        
        # Log distance to target
        self.get_logger().debug(f'Distance to target: {feedback.distance_to_target:.2f}m')
    
    def detection_callback(self, msg):
        """Process detection information from game master."""
        if not msg.detections:
            return
        
        # Store all detections
        self.latest_detections = msg.detections
        
        # Find friendly ship
        ship_found = False
        for detection in msg.detections:
            if detection.vehicle_type == Detection.SHIP and detection.is_friend:
                self.friendly_ship = detection
                ship_found = True
                
                # Set the last seen time
                self.ship_last_seen_time = time.time()
                break
        
        # If ship not found in this detection, don't immediately clear
        # Let the timeout logic handle temporary loss of detection
        
        # Count detections by type
        friend_count = sum(1 for d in msg.detections if d.is_friend)
        enemy_count = len(msg.detections) - friend_count
        
        # Log detection counts (periodically or when they change)
        if not hasattr(self, 'previous_counts'):
            self.previous_counts = {'friend': -1, 'enemy': -1}
            
        if (friend_count != self.previous_counts['friend'] or 
            enemy_count != self.previous_counts['enemy']):
            self.get_logger().info(f'Detected: {friend_count} friends, {enemy_count} enemies')
            self.previous_counts['friend'] = friend_count
            self.previous_counts['enemy'] = enemy_count
            
        # Periodically log ship position if detected
        if ship_found:
            if not hasattr(self, '_ship_pos_log_timer') or time.time() - self._ship_pos_log_timer > 10.0:
                self.get_logger().info(
                    f'Friendly ship detected at relative position: '
                    f'F={self.friendly_ship.relative_position.x:.1f}, '
                    f'R={self.friendly_ship.relative_position.y:.1f}, '
                    f'D={self.friendly_ship.relative_position.z:.1f}'
                )
                self._ship_pos_log_timer = time.time()
    
    def health_callback(self, msg):
        """Handle health updates."""
        old_health = self.health
        self.health = msg.data
        
        # Log health changes
        if self.health < old_health:
            self.get_logger().warning(f'Health changed: {old_health} -> {self.health}')
            
            # If health is 0, shut down the program
            if self.health == 0:
                self.get_logger().warning('Health is 0, shutting down drone controller')
                # Initiate clean shutdown
                self.destroy_node()
                rclpy.shutdown()
                # Use sys.exit to terminate the program
                import sys
                sys.exit(0)
            
            # Even if health is critically low but not zero, we'll stay with the ship
            # Just log a warning but don't change state
            elif hasattr(self, 'low_health_threshold') and self.health < self.low_health_threshold:
                self.get_logger().warning(f'Health critically low ({self.health}), continuing escort mission')
    
    def message_callback(self, msg):
        """Handle incoming messages."""
        self.get_logger().info(f'Received message: {msg.data}')
        
        # If someone asks about the ship position and we know it, respond
        if "ship" in msg.data.lower() and ("where" in msg.data.lower() or "position" in msg.data.lower()):
            if self.friendly_ship and self.estimated_position:
                self.send_message(
                    f"Ship escort reporting: Friendly ship at my relative position "
                    f"F={self.friendly_ship.relative_position.x:.1f}, "
                    f"R={self.friendly_ship.relative_position.y:.1f}, "
                    f"D={self.friendly_ship.relative_position.z:.1f}. "
                    f"I am at x={self.estimated_position['x']:.1f}, "
                    f"y={self.estimated_position['y']:.1f}, "
                    f"z={self.estimated_position['z']:.1f}"
                )


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