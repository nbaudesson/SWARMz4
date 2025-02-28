#!/usr/bin/env python3
"""Multi-Drone Mission Client"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from px4_controllers_interfaces.action import GotoPosition
from px4_controllers_interfaces.msg import PointYaw
import yaml
import time
from ament_index_python.packages import get_package_share_directory
import os.path

class DroneActionClient:
    def __init__(self, node: Node, drone_id: str, waypoints: list):
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

    def send_waypoint(self):
        """Send next waypoint as an action goal"""
        if self.current_waypoint >= len(self.waypoints):
            return False
            
        # Don't send new goal if one is in progress
        if self.goal_in_progress:
            return False

        waypoint = self.waypoints[self.current_waypoint]
        goal_msg = GotoPosition.Goal()
        goal_msg.target.position.x = float(waypoint['x'])
        goal_msg.target.position.y = float(waypoint['y'])
        goal_msg.target.position.z = float(waypoint['z'])
        goal_msg.target.yaw = float(waypoint['yaw'])

        # Wait for action server
        if not self._action_client.wait_for_server(timeout_sec=1.0):
            self.node.get_logger().warn(f'Action server not available for drone {self.drone_id}')
            return False

        # Send goal and setup callbacks
        self.goal_in_progress = True
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)
        return True

    def goal_response_callback(self, future):
        """Handle goal acceptance/rejection"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().error(f'Goal rejected for drone {self.drone_id}')
            self.goal_in_progress = False
            return

        self.goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """Handle action result and manage hover time"""
        try:
            result = future.result().result
            waypoint = self.waypoints[self.current_waypoint]
            status = "succeeded" if result.success else "failed"
            
            # Always log waypoint completion, but only once
            current_time = time.time()
            result_key = (self.current_waypoint, status)
            
            if result_key != self.last_result:
                self.node.get_logger().info(
                    f'Drone {self.drone_id}: Waypoint {self.current_waypoint + 1}/{len(self.waypoints)} {status}'
                )
                self.last_status_time = current_time
                self.last_result = result_key

            # Start hover timer
            self.hover_start_time = current_time
            self.is_hovering = True
            self.goal_in_progress = False
            self.goal_handle = None
            
        except Exception as e:
            self.node.get_logger().error(f'Error in result callback: {str(e)}')
            self.goal_in_progress = False
            self.goal_handle = None

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
    def __init__(self):
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
        """Handle mission execution with progress updates"""
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