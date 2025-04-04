"""
Kamikaze Service Server Node
===========================

This module implements a ROS2 service server that handles kamikaze (self-destruct) actions
for robots in a swarm simulation. When a robot triggers the kamikaze action, it damages
itself and any other robots within a configurable explosion range.

Key Features:
------------
- Handles kamikaze service requests from robots
- Tracks robot positions using Gazebo
- Applies configurable explosion damage to robots in range
- Integrates with health system through update_health service

Configuration Parameters:
-----------------------
- explosion_damage (int): Amount of damage dealt to robots (default: 3)
- explosion_range (float): Radius in meters of explosion effect (default: 5.0)
- world_name (string): Gazebo world name (default: "swarmz_world_2")
- update_health_service_name (string): Name of the update_health service (default: "/update_health")
- enable_gazebo_debug (bool): Enable Gazebo debug mode (default: False)
- gazebo_debug_interval (float): Interval in seconds for Gazebo debug logging (default: 60.0)

Usage:
------
1. Start the node:
   $ ros2 run game_master kamikaze_server

2. Call the service:
   $ ros2 service call /kamikaze swarmz_interfaces/srv/Kamikaze "{robot_name: 'robot1'}"

Dependencies:
------------
- swarmz_interfaces: Custom interface package with service definitions
- utils.tools: Namespace and distance calculation utilities
- utils.gazebo_subscriber: Robot position tracking
- rclpy: ROS2 Python client library

Implementation Details:
---------------------
The server maintains a list of all robot namespaces and their positions.
When a kamikaze request is received:
1. Validates the requesting robot's existence
2. Gets the robot's current position
3. Applies damage to the kamikaze robot
4. Finds all robots within explosion_range
5. Applies damage to affected robots through update_health service

Execution Flow:
-------------
1. Server Initialization
   - Load parameters
   - Setup tracking systems
   - Initialize services

2. Request Processing
   - Validate kamikaze robot
   - Get position data
   - Apply self-damage

3. Explosion Processing
   - Find robots in range
   - Calculate damage
   - Update health values

4. Cleanup
   - Log results
   - Update robot states
"""

# -----------------
# Imports Section
# -----------------
from swarmz_interfaces.srv import Kamikaze, UpdateHealth
import rclpy
from rclpy.node import Node
from utils.tools import get_all_namespaces, get_distance, get_stable_namespaces
from utils.gazebo_subscriber import GazeboPosesTracker
import time  # Still needed for sleep

# ----------------------
# Main Service Class
# ----------------------
class KamikazeServiceServer(Node):

    def __init__(self):
        super().__init__('kamikaze_service_server')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])

        # Add parameter for service name to ensure consistency
        self.declare_parameter('update_health_service_name', '/update_health')

        # Declare parameters with default values - using correct integer type for explosion_damage
        self.declare_parameter('explosion_damage', 3)
        self.declare_parameter('explosion_range', 5.0)
        self.declare_parameter('world_name', 'swarmz_world_2')

        # Get parameter values - accessing explosion_damage as an integer
        self.explosion_damage = self.get_parameter('explosion_damage').get_parameter_value().integer_value
        self.explosion_range = self.get_parameter('explosion_range').get_parameter_value().double_value
        self.world_name = self.get_parameter('world_name').get_parameter_value().string_value
        self.update_health_service_name = self.get_parameter('update_health_service_name').get_parameter_value().string_value

        # Get list of all namespaces and initialize GazeboPosesTracker in one step  
        self.get_logger().info("Detecting robot namespaces...")
        self.namespaces = get_stable_namespaces(self, max_attempts=10, wait_time=1.0)
        
        # Create GazeboPosesTracker directly without redundant variable declarations
        self.gz = GazeboPosesTracker(
            self.namespaces, 
            world_name=self.world_name,
        )

        # Create the kamikaze service first to ensure it's available
        self.srv = self.create_service(Kamikaze, 'kamikaze', self.kamikaze_callback)
        self.get_logger().info("Created kamikaze service")
        
        # Create the UpdateHealth client to communicate with the GameMasterNode
        self.get_logger().info(f"Creating client for '{self.update_health_service_name}' service")
        self.update_health_client = self.create_client(UpdateHealth, self.update_health_service_name)
        
        # Add service discovery checking but without flooding output
        try:
            from utils.gazebo_subscriber import check_ros2_services
            services = check_ros2_services(verbose=False)
            service_names = [s[0] for s in services]
            
            # Only log important service status, not the full list
            has_update_health = self.update_health_service_name in service_names
            self.get_logger().info(f"UpdateHealth service available: {has_update_health}")
        except Exception as e:
            self.get_logger().error(f"Error checking ROS2 services: {e}")
        
        # Initialize service connection status
        self.service_ready = False
        self.last_service_check_time = self.get_clock().now()
        
        # Create a timer to periodically check for the update_health service
        self.service_check_timer = self.create_timer(2.0, self.check_service_availability)
        self.get_logger().info("Service check timer initialized")

        self.get_logger().info('KamikazeServiceServer initialized with parameters: '
                        f'explosion_damage={self.explosion_damage}, '
                        f'explosion_range={self.explosion_range}, '
                        f'world_name={self.world_name}')
    
    def check_service_availability(self):
        """Periodically check if the update_health service is available"""
        if self.service_ready:
            # Already connected, no need to check
            return
            
        current_time = self.get_clock().now()
        # Don't spam logs, only log every 10 seconds
        time_diff_seconds = (current_time - self.last_service_check_time).nanoseconds / 1e9
        should_log = time_diff_seconds >= 10.0
        
        if self.update_health_client.service_is_ready():
            self.service_ready = True
            self.get_logger().info('Successfully connected to update_health service')
            # We can reduce the timer frequency once connected
            self.service_check_timer.timer_period_ns = 30 * 1000000000  # 30 seconds
        elif should_log:
            self.get_logger().info('Waiting for update_health service to become available...')
            self.last_service_check_time = current_time

    def update_health_request(self, robot_name, damage):
        """
        Create a request to update health of a robot.
        :param robot_name: The name of the robot to update health.
        :param damage: The amount of damage to apply.
        :return: The request object.
        """
        # Check if service is available now if it wasn't earlier
        if not self.service_ready:
            if self.update_health_client.service_is_ready():
                self.service_ready = True
                self.get_logger().info('Successfully connected to update_health service')
            else:
                self.get_logger().warn(f'Update_health service not available, health update for {robot_name} failed')
                return None
                
        request = UpdateHealth.Request()
        request.robot_name = robot_name
        request.damage = damage
        future = self.update_health_client.call_async(request)
        return future

    def kamikaze_callback(self, request, response):
        # Move the logging line here (was out of order in the original)
        self.get_logger().info(f'Received kamikaze request from {request.robot_name}')
        
        """
        Handle the kamikaze service request.
        :param request: The service request containing the robot name.
        :param response: The service response.
        :return: The updated response.
        """
        # Get the namespace of the robot requesting the kamikaze action
        kamikaze_ns = request.robot_name
        if not kamikaze_ns:
            self.get_logger().warn('Robot name is None. Cannot detonate.')
            return response
        if not kamikaze_ns.startswith('/'):
            kamikaze_ns = '/' + kamikaze_ns

        if kamikaze_ns not in self.namespaces:
            self.get_logger().warn(f'{kamikaze_ns} not found in robot list. Cannot detonate.')
            return response

        # Get the position of the kamikaze robot
        kamikaze_pose = self.gz.get_pose(kamikaze_ns)
        kamikaze_position = (kamikaze_pose['position']['x'], kamikaze_pose['position']['y'], kamikaze_pose['position']['z'])
        self.get_logger().info(f'💥 KAMIKAZE: {kamikaze_ns} detonating at position {kamikaze_position}')

        # Call the update_health service of the GameMasterNode to apply damage to the kamikaze robot
        try:
            self.update_health_request(kamikaze_ns, self.explosion_damage)
        except Exception as e:
            self.get_logger().error(f'Exception occurred while calling update_health service: {e}')
            return response

        # Track affected robots to log a summary instead of individual messages
        affected_robots = []
        
        # Check all other robots within the explosion range and apply damage
        for target_ns in self.namespaces:
            if target_ns != kamikaze_ns:
                # Reduce logging - avoid logging for every robot checked
                self.get_logger().debug(f'Checking {target_ns}')
                try:
                    robot_position = self.gz.get_robot_position(target_ns)
                except Exception as e:
                    self.get_logger().error(f'Exception occurred while getting pose of {target_ns}: {e}')
                    continue
                    
                distance = get_distance(kamikaze_position, robot_position)
                # Use debug level for distance calculations to avoid flooding
                self.get_logger().debug(f'{target_ns} at distance {distance}')
                
                if distance <= self.explosion_range:
                    affected_robots.append(target_ns)
                    # Still log affected robots, but more concisely
                    self.get_logger().info(f'DAMAGE: {target_ns} caught in explosion')
                    try:
                        self.update_health_request(target_ns, self.explosion_damage)
                    except Exception as e:
                        self.get_logger().error(f'Exception occurred while calling update_health service: {e}')
                        return response
        
        # Add a summary log at the end
        self.get_logger().info(f'Kamikaze explosion from {kamikaze_ns} affected {len(affected_robots)} robots')
        
        return response

def main(args=None):
    rclpy.init(args=args)
    kamikaze_service_server = KamikazeServiceServer()
    try:
        rclpy.spin(kamikaze_service_server)
    except Exception as e:
        kamikaze_service_server.get_logger().error(f'Exception occurred in kamikaze node {e}')
    finally:
        kamikaze_service_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()