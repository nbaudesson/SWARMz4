#!/usr/bin/env python3
"""
Multi-Drone Mission Control Client for PX4 VTOL Aircraft

This node provides high-level mission control for multiple PX4-powered drones.
It loads mission files in YAML format and executes waypoint-based missions
with configurable hover times at each point.

Features:
---------
- Multi-drone mission coordination
- YAML-based mission configuration
- Automatic waypoint progression
- Configurable hover times
- Progress monitoring and logging
- Automatic mission completion detection

Requirements:
------------
- ROS 2 (Foxy/Humble)
- PX4 Autopilot
- offboard_control_py package
- px4_controllers_interfaces package
- YAML mission configuration file

Mission File Format:
------------------
drone_1:  # Drone identifier
  - x: 10.0    # X position in meters
    y: 0.0     # Y position in meters
    z: 2.0     # Z position in meters
    yaw: 0.0   # Yaw angle in radians
    time: 5.0  # Hover time in seconds
  # ... more waypoints ...
drone_2:
  # ... waypoints for second drone ...

Usage:
------
1. Create a mission file in the offboard_control_py/missions directory
2. Launch the node with:
   $ ros2 run offboard_control_py mission_control --ros-args -p mission_file:=my_mission.yaml

Parameters:
----------
- mission_file: Name of the YAML mission file (required)
- log_level: Logging verbosity ('debug' or 'info', default: 'info')

Publishers:
----------
- Various action clients for each drone (via GotoPosition action)

Subscribers:
-----------
- Feedback from each drone's action server
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from px4_controllers_interfaces.action import GotoPosition
import yaml
import time
from ament_index_python.packages import get_package_share_directory
import os.path

class DroneActionClient:
    """
    Handles individual drone mission execution and monitoring.
    
    This class manages:
    - Waypoint progression
    - Action client communication
    - Hover timing
    - Progress monitoring
    - Status logging
    """
    
    def __init__(self, node: Node, drone_id: str, waypoints: list):
        """
        Initialize drone control client.
        
        Args:
            node: Parent ROS node
            drone_id: Unique identifier for the drone
            waypoints: List of waypoint dictionaries
        """
        self.node = node
        self.drone_id = drone_id
        self.waypoints = waypoints
        self.current_waypoint = 0
        self.hover_start_time = None
        self.is_hovering = False
        self.goal_handle = None  # Add this to track current goal handle
        self.goal_in_progress = False  # Add this to track goal status
        
        # Create action client
        self._action_client = ActionClient(
            node,
            GotoPosition,
            f'/px4_{drone_id}/goto_position'
        )
        
        # Improved logging controls
        self.last_status_time = 0.0
        self.status_interval = 2.0  # Increased to 2 seconds
        self.last_result = None
        self.distance_log_threshold = 1.0  # Only log distance changes > 1m
        self.last_logged_distance = None
        self.log_counter = 0
        self.log_modulo = 5  # Only log every 5th message

        # Add retry configuration
        self.max_retries = 3  # Maximum number of retry attempts
        self.retry_delay = 2.0  # Seconds to wait between retries
        self.current_retry = 0  # Current retry attempt
        self.failure_reason = None  # Store the reason for failure
        self.last_goal_status = None  # Track goal status
        self.retry_timer = None  # Add timer storage

    def send_waypoint(self):
        """
        Send next waypoint as an action goal.
        
        Returns:
            bool: True if waypoint was sent, False if no more waypoints
                 or action server unavailable
        """
        if self.current_waypoint >= len(self.waypoints):
            return False
            
        # Don't send new goal if one is in progress
        if self.goal_in_progress:
            return False

        # Reset retry counter when sending new waypoint
        self.current_retry = 0
        self.failure_reason = None
        
        waypoint = self.waypoints[self.current_waypoint]
        goal_msg = GotoPosition.Goal()
        goal_msg.target.position.x = float(waypoint['x'])
        goal_msg.target.position.y = float(waypoint['y'])
        goal_msg.target.position.z = float(waypoint['z'])
        goal_msg.target.yaw = float(waypoint['yaw'])

        # Wait for action server with improved error reporting
        if not self._action_client.wait_for_server(timeout_sec=1.0):
            self.failure_reason = "Action server not available"
            self.node.get_logger().error(f'Drone {self.drone_id}: {self.failure_reason}')
            return False

        # Send goal and setup callbacks
        self.goal_in_progress = True
        self.node.get_logger().info(
            f'Drone {self.drone_id}: Sending waypoint {self.current_waypoint + 1} '
            f'(Attempt {self.current_retry + 1}/{self.max_retries})'
        )
        
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)
        return True

    def goal_response_callback(self, future):
        """
        Handle goal acceptance/rejection from action server.
        
        Args:
            future: Future object containing goal response
        """
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.failure_reason = "Goal rejected by action server"
                self.node.get_logger().warn(
                    f'Drone {self.drone_id}: {self.failure_reason}. '
                    f'Attempt {self.current_retry + 1}/{self.max_retries}'
                )
                self.handle_failure()
                return

            self.goal_handle = goal_handle
            self.last_goal_status = "ACCEPTED"
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self.get_result_callback)
            
        except Exception as e:
            self.failure_reason = f"Error in goal response: {str(e)}"
            self.node.get_logger().error(f'Drone {self.drone_id}: {self.failure_reason}')
            self.handle_failure()

    def get_result_callback(self, future):
        """Handle action result and manage hover time"""
        try:
            result = future.result().result
            waypoint = self.waypoints[self.current_waypoint]
            status = "succeeded" if result.success else "failed"
            
            if not result.success:
                self.failure_reason = "Goal execution failed"
                self.last_goal_status = "FAILED"
                self.node.get_logger().warn(
                    f'Drone {self.drone_id}: Waypoint {self.current_waypoint + 1} failed. '
                    f'Position: [{waypoint["x"]}, {waypoint["y"]}, {waypoint["z"]}], '
                    f'Yaw: {waypoint["yaw"]}. Attempt {self.current_retry + 1}/{self.max_retries}'
                )
                self.handle_failure()
                return
                
            # Success case
            self.node.get_logger().info(
                f'Drone {self.drone_id}: Waypoint {self.current_waypoint + 1} succeeded'
            )
            self.last_goal_status = "SUCCEEDED"
            self.current_retry = 0  # Reset retry counter
            self.hover_start_time = self.node.get_clock().now().seconds_nanoseconds()[0]
            self.is_hovering = True
            self.goal_in_progress = False
            self.goal_handle = None
            
        except Exception as e:
            self.failure_reason = f"Error in result callback: {str(e)}"
            self.node.get_logger().error(f'Drone {self.drone_id}: {self.failure_reason}')
            self.handle_failure()

    def handle_failure(self):
        """Handle failures with retry logic."""
        self.goal_in_progress = False
        self.goal_handle = None
        
        if self.current_retry < self.max_retries:
            self.current_retry += 1
            self.node.get_logger().info(
                f'Drone {self.drone_id}: Retrying waypoint {self.current_waypoint + 1} '
                f'in {self.retry_delay} seconds (Attempt {self.current_retry}/{self.max_retries})'
            )
            # Create and store timer for delayed retry
            if self.retry_timer:
                self.retry_timer.cancel()
            self.retry_timer = self.node.create_timer(
                self.retry_delay,
                self.retry_timer_callback
            )
        else:
            self.node.get_logger().error(
                f'Drone {self.drone_id}: Maximum retries ({self.max_retries}) reached. '
                f'Last failure reason: {self.failure_reason}. '
                f'Last goal status: {self.last_goal_status}'
            )
            # Move to next waypoint after max retries
            self.current_waypoint += 1
            self.current_retry = 0
            self.goal_in_progress = False

    def retry_timer_callback(self):
        """Handle retry timer callback."""
        if self.retry_timer:
            self.retry_timer.cancel()
            self.retry_timer = None
        self.send_waypoint()

    def feedback_callback(self, feedback_msg):
        """Handle action feedback with reduced logging"""
        # Only log significant distance changes at debug level
        current_distance = feedback_msg.feedback.distance_to_target
        current_time = time.time()
        
        # Increment counter and check if we should log
        self.log_counter += 1
        should_log = (
            self.log_counter % self.log_modulo == 0 and  # Every nth message
            current_time - self.last_status_time >= self.status_interval and  # Time threshold
            (
                self.last_logged_distance is None or  # First message
                abs(current_distance - self.last_logged_distance) >= self.distance_log_threshold  # Significant change
            )
        )
        
        if should_log:
            self.node.get_logger().debug(
                f'Drone {self.drone_id}: {current_distance:.1f}m to target'
            )
            self.last_status_time = current_time
            self.last_logged_distance = current_distance

    def check_hover_complete(self):
        """Check if hover time completed and move to next waypoint"""
        if not self.is_hovering:
            return False

        try:
            waypoint = self.waypoints[self.current_waypoint]
            if time.time() - self.hover_start_time >= waypoint['time']:
                self.current_waypoint += 1
                self.is_hovering = False
                self.hover_start_time = None
                return True
        except Exception as e:
            self.node.get_logger().error(f'Error in hover check: {str(e)}')
            self.is_hovering = False
            self.hover_start_time = None
            
        return False

class MultiDroneMissionClient(Node):
    """
    Main mission control node managing multiple drones.
    
    This node:
    - Loads mission configurations
    - Creates and manages drone clients
    - Monitors overall mission progress
    - Handles mission completion
    """
    
    def __init__(self):
        """Initialize the mission control node."""
        super().__init__('offboard_control_client')
        
        # Add logging control with more restrictive defaults
        self.declare_parameter('log_level', 'info')
        log_level = self.get_parameter('log_level').value
        
        # Set more restrictive default logging
        if log_level == 'debug':
            self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
        else:
            # Default to INFO level with reduced output
            self.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)
            
        # Add progress tracking
        self.mission_start_time = time.time()
        self.last_progress_time = 0
        self.progress_interval = 5.0  # Show progress every 5 seconds
        
        # Initialize drone clients dictionary
        self.drone_clients = {}
        
        # Load mission file
        self.declare_parameter('mission_file', '')
        mission_name = self.get_parameter('mission_file').value
        
        if not mission_name:
            self.get_logger().error('No mission file specified')
            return
            
        # Load mission configuration
        try:
            package_dir = get_package_share_directory('offboard_control_py')
            mission_path = os.path.join(
                package_dir, 'missions',
                mission_name if mission_name.endswith('.yaml') else f'{mission_name}.yaml'
            )
            
            if not os.path.exists(mission_path):
                self.get_logger().error(f'Mission file not found: {mission_path}')
                return
                
            self.get_logger().info(f'Loading mission from: {mission_path}')
            
            with open(mission_path, 'r') as f:
                mission_config = yaml.safe_load(f)
                
            # Create drone clients
            for drone_id, waypoints in mission_config.items():
                self.drone_clients[drone_id] = DroneActionClient(self, drone_id, waypoints)
            
            # Create timer for mission execution
            self.create_timer(0.1, self.mission_timer_callback)
            
        except Exception as e:
            self.get_logger().error(f'Failed to initialize: {str(e)}')

    def mission_timer_callback(self):
        """
        Periodic callback to manage mission execution.
        
        - Calculates mission progress
        - Updates status logging
        - Manages waypoint progression
        - Detects mission completion
        """
        try:
            all_complete = True
            completed_waypoints = 0
            total_waypoints = 0
            
            # Calculate progress
            for client in self.drone_clients.values():
                total_waypoints += len(client.waypoints)
                completed_waypoints += min(client.current_waypoint, len(client.waypoints))
                
                if client.current_waypoint < len(client.waypoints):
                    all_complete = False
            
            # Show periodic progress updates
            current_time = time.time()
            if current_time - self.last_progress_time >= self.progress_interval:
                progress = (completed_waypoints / total_waypoints) * 100 if total_waypoints > 0 else 0
                elapsed_time = current_time - self.mission_start_time
                self.get_logger().info(
                    f'Mission progress: {progress:.1f}% ({completed_waypoints}/{total_waypoints} waypoints), '
                    f'Time: {elapsed_time:.1f}s'
                )
                self.last_progress_time = current_time
            
            # Process waypoints
            for client in self.drone_clients.values():
                if client.current_waypoint >= len(client.waypoints):
                    continue
                    
                if client.check_hover_complete():
                    if client.current_waypoint < len(client.waypoints):
                        client.send_waypoint()
                elif not client.is_hovering and not client.goal_in_progress:
                    client.send_waypoint()
            
            if all_complete:
                final_time = time.time() - self.mission_start_time
                self.get_logger().info(
                    f'All missions completed in {final_time:.1f} seconds'
                )
                rclpy.shutdown()
                
        except Exception as e:
            self.get_logger().error(f'Error in mission timer: {str(e)}')

def main(args=None):
    """
    Main function initializing and running the mission control node.
    
    Args:
        args: Command line arguments passed to ROS
    """
    rclpy.init(args=args)
    
    try:
        node = MultiDroneMissionClient()
        
        # Set default ROS2 logging format to be more concise
        logging_format = '[{severity}] [{name}]: {message}'
        rclpy.logging.set_logger_level('offboard_control_client', 10)  # INFO level
        
        rclpy.spin(node)
    except Exception as e:
        print(f'Error in main: {str(e)}')
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()