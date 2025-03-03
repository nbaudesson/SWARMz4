"""
Missile Service Server for SWARMz4 Game System

This node manages the missile firing system for both drones and ships in the game.
It handles missile firing requests, checks if targets are in range and aligned,
and manages ammunition and cooldown timers for each robot.

Requirements:
- ROS2 running with the following:
    - swarmz_interfaces package with Missile.srv and UpdateHealth.srv
    - Gazebo simulation running with robots
    - GameMasterNode running for health updates
    - Valid robot namespaces configured

Parameters:
    - drone_missile_range: Maximum range for drone missiles (default: 100)
    - ship_missile_range: Maximum range for ship missiles (default: 200)
    - drone_missile_damage: Damage dealt by drone missiles (default: 10)
    - ship_missile_damage: Damage dealt by ship missiles (default: 20)
    - drone_cooldown: Cooldown time between drone shots (default: 1.0)
    - ship_cooldown: Cooldown time between ship shots (default: 2.0)
    - drone_magazine: Number of missiles per drone (default: 5)
    - ship_magazine: Number of missiles per ship (default: 10)
    - laser_width: Width of targeting laser (default: 0.1)
    - laser_length: Length of targeting laser (default: 1.0)
    - padding parameters: Hit box dimensions for drones and ships

Usage:
    1. Start the ROS2 system and Gazebo simulation
    2. Launch this node as part of the game_master package
    3. Robots can request missile firing through the 'fire_missile' service

Manual Service Call:
    To test the missile service manually, use the following command:
    ```bash
    ros2 service call /fire_missile swarmz_interfaces/srv/Missile "{robot_name: '/px4_1'}"
    ```
    Replace '/px4_1' with the namespace of the robot you want to fire from.
"""

from swarmz_interfaces.srv import Missile, UpdateHealth
import rclpy
from rclpy.node import Node
from utils.tools import get_all_namespaces, get_distance, is_aligned, is_aligned_HB
from utils.gazebo_subscriber import GazeboPosesTracker
import time
from std_msgs.msg import Int32

class MissileServiceServer(Node):

    def __init__(self):
        super().__init__('missile_service_server')
        self.get_logger().info('Initializing MissileServiceServer')
        
        # Enable simulation time
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])

        # Initialize all parameters with default values
        self._init_parameters()
        
        # Initialize tracking systems
        self._init_tracking_systems()
        
        # Set up service and subscribers
        self._setup_communications()

    def _init_parameters(self):
        """Initialize and load all parameters for the missile system"""
        self.declare_parameter('drone_missile_range', 100)
        self.declare_parameter('ship_missile_range', 200)
        self.declare_parameter('drone_missile_damage', 10)
        self.declare_parameter('ship_missile_damage', 20)
        self.declare_parameter('drone_cooldown', 1.0)
        self.declare_parameter('ship_cooldown', 2.0)
        self.declare_parameter('drone_magazine', 5)
        self.declare_parameter('ship_magazine', 10)
        self.declare_parameter('laser_width', 0.1)
        self.declare_parameter('laser_length', 1.0)
        
        self.declare_parameter('drone_padding_x', 0.5)
        self.declare_parameter('drone_padding_y', 0.5)
        self.declare_parameter('drone_padding_z', 0.5)
        self.declare_parameter('ship_padding_x', 6.0)
        self.declare_parameter('ship_padding_y', 1.0)
        self.declare_parameter('ship_padding_z', 1.0)

        # Collect parameters into instance variables
        self.drone_missile_range = self.get_parameter('drone_missile_range').get_parameter_value().integer_value
        self.ship_missile_range = self.get_parameter('ship_missile_range').get_parameter_value().integer_value
        self.drone_missile_damage = self.get_parameter('drone_missile_damage').get_parameter_value().integer_value
        self.ship_missile_damage = self.get_parameter('ship_missile_damage').get_parameter_value().integer_value
        self.drone_cooldown = self.get_parameter('drone_cooldown').get_parameter_value().double_value
        self.ship_cooldown = self.get_parameter('ship_cooldown').get_parameter_value().double_value
        self.drone_magazine = self.get_parameter('drone_magazine').get_parameter_value().integer_value
        self.ship_magazine = self.get_parameter('ship_magazine').get_parameter_value().integer_value
        self.laser_width = self.get_parameter('laser_width').get_parameter_value().double_value
        self.laser_length = self.get_parameter('laser_length').get_parameter_value().double_value

        self.drone_padding_x = self.get_parameter('drone_padding_x').get_parameter_value().double_value
        self.drone_padding_y = self.get_parameter('drone_padding_y').get_parameter_value().double_value
        self.drone_padding_z = self.get_parameter('drone_padding_z').get_parameter_value().double_value
        self.ship_padding_x = self.get_parameter('ship_padding_x').get_parameter_value().double_value
        self.ship_padding_y = self.get_parameter('ship_padding_y').get_parameter_value().double_value
        self.ship_padding_z = self.get_parameter('ship_padding_z').get_parameter_value().double_value

    def _init_tracking_systems(self):
        """Initialize systems for tracking robots and their states"""
        # Get list of all robot namespaces
        self.namespaces = get_all_namespaces(self)
        while not self.namespaces:
            self.get_logger().warn("No valid namespaces detected. Waiting...")
            time.sleep(1)
            self.namespaces = get_all_namespaces(self)

        # Initialize tracking dictionaries
        self.magazines = {ns: self.drone_magazine if 'drone' in ns else self.ship_magazine 
                         for ns in self.namespaces}
        self.last_fire_time = {ns: 0 for ns in self.namespaces}
        
        # Initialize Gazebo pose tracker
        self.gz = GazeboPosesTracker(self.namespaces)
        self.robots_poses = self.gz.poses

    def _setup_communications(self):
        """Set up all ROS2 communications (services, subscribers, clients)"""
        # Subscribe to health topics for all robots
        self.health_subscribers = {
            ns: self.create_subscription(
                Int32, 
                f'{ns}/health',
                lambda msg, ns=ns: self.health_callback(msg, ns),
                10
            ) for ns in self.namespaces
        }

        # Create UpdateHealth client
        self.update_health_client = self.create_client(UpdateHealth, 'update_health')
        while not self.update_health_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Waiting for update_health service...')

        # Create missile service
        self.srv = self.create_service(Missile, 'fire_missile', self.fire_missile_callback)

    def health_callback(self, msg, ns):
        """
        Callback function to handle health updates.
        :param msg: The health message.
        :param ns: The namespace of the robot.
        """
        if msg.data == 0:
            self.get_logger().info(f'{ns} has been destroyed. Removing from missile_server list of namespaces.')
            if ns in self.namespaces:
                self.namespaces.remove(ns)
                self.gz = GazeboPosesTracker(self.namespaces)
                # Instead of destroying immediately, add to destroy_list
                # self.destroy_list.append(ns)

    def update_health_request(self, robot_name, damage):
        """
        Create a request to update health of a robot.
        :param robot_name: The name of the robot to update health.
        :param damage: The amount of damage to apply.
        :return: The request object.
        """
        request = UpdateHealth.Request()
        request.robot_name = robot_name
        request.damage = damage
        future = self.update_health_client.call_async(request)
        return future

    def fire_missile_callback(self, request, response):
        """
        Main callback for handling missile firing requests.
        
        Process:
        1. Validate shooter and get their position
        2. Check ammunition and cooldown
        3. Find targets within range
        4. Check target alignment
        5. Apply damage to closest aligned target
        
        Args:
            request: Contains robot_name of the shooter
            response: Will contain has_fired status and remaining ammo
        """
        self.get_logger().info(f'Received missile fire request from {request.robot_name}')
        
        # 1. Get shooter namespace
        shooter_ns = request.robot_name
        if shooter_ns not in self.namespaces:
            self.get_logger().warn(f'{shooter_ns} not found in robot list. Cannot fire missile.')
            response.has_fired = False
            response.ammo = 0
            return response

        shooter_pose = self.gz.get_pose(shooter_ns)
        if shooter_pose is None:
            self.get_logger().warn(f"No pose found for '{shooter_ns}' in Gazebo. Cannot fire missile.")
            self.get_logger().warn(f"Current robot poses: {shooter_pose}") 
            response.has_fired = False
            response.ammo = self.magazines[shooter_ns]
            return response

        # 2. Check if corresponding namespace in the magazine dictionary has a value > 0
        current_time = time.time()
        cooldown = self.drone_cooldown if 'drone' in shooter_ns else self.ship_cooldown
        if self.magazines[shooter_ns] > 0 and (current_time - self.last_fire_time[shooter_ns]) >= cooldown:
            self.get_logger().info(f'{shooter_ns} fired a missile. Remaining ammo: {self.magazines[shooter_ns] - 1}')
            # Decrease magazine count
            self.magazines[shooter_ns] -= 1
            response.has_fired = True
            response.ammo = self.magazines[shooter_ns]
            # Update last fire time
            self.last_fire_time[shooter_ns] = current_time
        else:
            self.get_logger().warn(f'{shooter_ns} cannot fire. Either out of ammo or still in cooldown.')
            response.has_fired = False
            response.ammo = self.magazines[shooter_ns]
            return response

        # 3. Get shooter position and orientation
        shooter_position = (shooter_pose['position']['x'], shooter_pose['position']['y'], shooter_pose['position']['z'])
        shooter_orientation = (shooter_pose['orientation']['x'], shooter_pose['orientation']['y'], shooter_pose['orientation']['z'], shooter_pose['orientation']['w'])
        self.get_logger().info(f'{shooter_ns} position: {shooter_position}, orientation: {shooter_orientation}')

        # 4. Out of all robots in gazebo get list of ID's, distance and positions of those that are within radius of missile range
        missile_range = self.drone_missile_range if '/px4_' in shooter_ns else self.ship_missile_range
        targets_in_range = []
        for robot in self.namespaces:
            if robot == shooter_ns:
                continue
            robot_pose = self.gz.get_pose(robot)
            robot_position = (robot_pose['position']['x'], robot_pose['position']['y'], robot_pose['position']['z'])
            distance = get_distance(shooter_position, robot_position)
            if distance <= missile_range:
                targets_in_range.append((robot, distance, robot_position))
        self.get_logger().info(f'Targets in range: {targets_in_range}')

        # 5. Out of those robots remove those that are not aligned with shooter's orientation, using laser_width as threshold
        aligned_targets = []
        for target in targets_in_range:
            target_id = target[0]
            target_position = target[2]
            if '/px4_' in target_id:
                target_padding = (self.drone_padding_x, self.drone_padding_y, self.drone_padding_z)
            else:
                target_padding = (self.ship_padding_x, self.ship_padding_y, self.ship_padding_z)
            self.get_logger().info(f'Target {target_id} position: {target_position}')
            # aligned_result = is_aligned(
            aligned_result = is_aligned_HB(
                self,
                shooter_position,
                shooter_orientation,
                target_position,
                target_padding,
                missile_range, # base_length
                self.laser_width, # base_radius
                verbose=False
            )
            if aligned_result:
                aligned_targets.append(target)
        self.get_logger().info(f'Aligned targets: {aligned_targets}')

        # 6. For the element of the list with the smallest distance, send ID and damage to game_master
        if aligned_targets:
            target = min(aligned_targets, key=lambda x: x[1])
            target_id = target[0]
            damage = self.drone_missile_damage if '/px4_' in shooter_ns else self.ship_missile_damage
            self.get_logger().info(f'Target {target_id} selected for attack with damage {damage}')
            
            try:
                self.update_health_request(target_id, damage)
            except Exception as e:
                self.get_logger().error(f'Exception occurred while calling update_health service: {e}')
                return response

        # 8. return response has_fired = true and ammo = namespace.magazine
        self.get_logger().info(f'Missile service response: has_fired={response.has_fired}, ammo={response.ammo}')
        return response

def main(args=None):
    """Initialize and run the missile service node"""
    rclpy.init(args=args)
    missile_service_server = MissileServiceServer()
    try:
        rclpy.spin(missile_service_server)
    except Exception as e:
        missile_service_server.get_logger().error(f'Exception occurred in missile node {e}')
    finally:
        missile_service_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()