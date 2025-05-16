"""
Game Master Node for Multi-Robot Combat Simulation

This module implements a ROS2 node that manages a multi-robot combat simulation game.
It handles team formation, health tracking, robot detection, communication, and scoring
for both drones (PX4) and ships in a Gazebo-based simulation environment.

Features:
- Automatic team formation
- Health and damage management
- Detection range simulation
- Inter-robot communication
- Score tracking
- Game state management
- Automated game results logging
- Missile firing system
- Kamikaze/self-destruct functionality

Usage:
1. Ensure ROS2 and Gazebo are properly installed and configured.
2. Launch the Gazebo simulation with the appropriate world file.
3. Start the Game Master node using the following command:
   ```bash
   ros2 run game_master new_game_master_node
   ```
4. Robots in the simulation will automatically be detected and managed by the Game Master node.
5. Use the provided services (`fire_missile`, `kamikaze`) to interact with the game.
6. Monitor the game state and results through the `/game_master/game_state` topic and the generated results files.

Manual Service Calls:
- To fire a missile:
  ```bash
  ros2 service call /fire_missile swarmz_interfaces/srv/Missile "{robot_name: '/px4_1'}"
  ```
  Replace `/px4_1` with the namespace of the robot you want to fire from.

- To trigger a kamikaze:
  ```bash
  ros2 service call /kamikaze swarmz_interfaces/srv/Kamikaze "{robot_name: '/px4_1'}"
  ```
  Replace `/px4_1` with the namespace of the robot you want to self-destruct.

Results:
- Game results are saved in the `results` directory under the `SWARMz4` folder in the user's home directory.
- Individual game results are stored in timestamped files within the `individual_games` subdirectory.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
from swarmz_interfaces.msg import Detections, Detection, GameState, RobotState
from utils.tools import get_distance, get_relative_position_with_orientation, get_relative_position_with_heading, get_stable_namespaces, is_aligned_HB
from utils.kill_drone import get_model_id, kill_drone_from_game_master
from utils.gazebo_subscriber import GazeboPosesTracker
from swarmz_interfaces.srv import Kamikaze, Missile
import time  # Still imported for sleep function only
import threading
import os
from datetime import datetime
from gz.transport13 import Node as GzNode
from geometry_msgs.msg import Pose
from concurrent.futures import ThreadPoolExecutor

class GameMasterNode(Node):
    """
    Main node for managing the combat simulation game.

    This node handles:
    - Team formation and management
    - Robot health tracking
    - Detection and communication simulation
    - Score keeping
    - Game state management
    - Results logging
    - Missile firing system (integrated from missile_server)
    - Kamikaze functionality (integrated from kamikaze_server)

    Parameters (via ROS2 parameters):
        drone_detection_range (float): Maximum range at which drones can detect others
        ship_detection_range (float): Maximum range at which ships can detect others
        drone_communication_range (float): Maximum range for drone communications
        ship_communication_range (float): Maximum range for ship communications
        drone_health (int): Initial health points for drones
        ship_health (int): Initial health points for ships
        drone_points (int): Points awarded for destroying a drone
        ship_points (int): Points awarded for destroying a ship
        game_duration (int): Game duration in seconds
        gazebo_world_name (str): Name of the Gazebo world
        drone_model_base_name (str): Base name pattern for drone models
        
        # Missile system parameters
        drone_missile_range (float): Maximum range for drone missiles
        ship_missile_range (float): Maximum range for ship missiles
        drone_missile_damage (int): Damage dealt by drone missiles
        ship_missile_damage (int): Damage dealt by ship missiles
        drone_cooldown (float): Cooldown time between drone shots
        ship_cooldown (float): Cooldown time between ship shots
        drone_magazine (int): Number of missiles per drone
        ship_magazine (int): Number of missiles per ship
        laser_width (float): Width of targeting laser
        drone_padding_x (float): Hit box X dimension for drones
        drone_padding_y (float): Hit box Y dimension for drones
        drone_padding_z (float): Hit box Z dimension for drones
        ship_padding_x (float): Hit box X dimension for ships
        ship_padding_y (float): Hit box Y dimension for ships
        ship_padding_z (float): Hit box Z dimension for ships
        
        # Kamikaze system parameters
        explosion_damage (int): Amount of damage dealt by kamikaze explosions
        explosion_range (float): Radius of kamikaze explosion effect
    """

    ###########################################
    # INITIALIZATION AND SETUP
    ###########################################

    def __init__(self):
        """Initialize the Game Master node and set up all necessary components."""
        super().__init__('game_master_node')
        # Enable simulation time
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])

        # Create a single Gazebo node instance for reuse
        self.gz_node = GzNode()

        # Initialize locks for thread safety
        self.robot_poses_lock = threading.Lock()
        self.distance_cache_lock = threading.Lock()
        self.health_points_lock = threading.Lock()

        # Add a shutdown flag to safely handle thread termination
        self.is_shutting_down = False
        self.detection_threads = []

        # Add tracking for dead robots
        self.dead_robots = {}  # Store data for robots that have been destroyed
        self.all_robot_namespaces = []  # Track all robots that have existed

        # Initialize game parameters
        self._init_game_parameters()
        self._init_missile_parameters()
        self._init_kamikaze_parameters()

        # Detect robot namespaces
        self.get_logger().info("Detecting robot namespaces...")
        self.namespaces = get_stable_namespaces(self, max_attempts=10, wait_time=1.0)

        # Get Gazebo model IDs for drones
        self.drone_models = self._get_model_ids(self.namespaces, self.gazebo_world_name)

        # Log detected robots
        self.get_logger().info(f"Detected robots: {self.namespaces}")

        # Initialize teams, health, and missile systems
        self._initialize_teams()
        self._initialize_health_system()
        self._initialize_missile_system()

        # Set up publishers, subscribers, and services
        self._setup_communications()

        # Initialize robot positions and timers
        self.robot_poses = {}
        self.robot_velocities = {}
        self.last_update_time = {}
        self.distance_cache = {}
        self.gz = GazeboPosesTracker(self.namespaces, logger=self.get_logger(), world_name=self.gazebo_world_name)
        self.get_logger().info("Gazebo poses tracker initialized")
        time.sleep(1)  # Allow time for Gazebo to initialize

        # Populate self.robot_poses with initial positions
        for ns in self.namespaces:
            pose = self.gz.get_pose(ns)
            self._update_poses(pose, ns)
        self.last_update_time[ns] = self.get_clock().now()
        self.get_logger().info(self._format_initial_positions(self.robot_poses))

        # Call _update_positions_and_distance to initialize velocities and distances
        self._update_positions_and_distance()

        # Timer for periodic detections
        self.timer = self.create_timer(0.5, self._detections_callback)

        # Initialize game timer
        current_time = self.get_clock().now()
        self.start_time = current_time
        self.game_timer = self.create_timer(1.0, self._game_timer_callback)

        # Combine position updates and distance cache updates into a single timer
        self.update_positions_timer = self.create_timer(0.1, self._update_positions_and_distance)  # Update every 100ms

    ##############################
    # INITIALIZE GAME PARAMETERS #
    ##############################

    def _init_game_parameters(self):
        """Initialize the core game parameters."""
        self.declare_parameter('drone_detection_range', 137.0)
        self.declare_parameter('ship2ship_detection_range', 500.0)
        self.declare_parameter('ship2drone_detection_range', 162.0)
        self.declare_parameter('drone_communication_range', 144.0)
        self.declare_parameter('ship_communication_range', 525.0)
        self.declare_parameter('drone_health', 1)
        self.declare_parameter('ship_health', 6)
        self.declare_parameter('game_duration', 300)  # 5 minutes
        self.declare_parameter('gazebo_world_name', 'game_world_water')

        self.drone_detection_range = self.get_parameter('drone_detection_range').get_parameter_value().double_value
        self.ship2ship_detection_range = self.get_parameter('ship2ship_detection_range').get_parameter_value().double_value
        self.ship2drone_detection_range = self.get_parameter('ship2drone_detection_range').get_parameter_value().double_value
        self.drone_communication_range = self.get_parameter('drone_communication_range').get_parameter_value().double_value
        self.ship_communication_range = self.get_parameter('ship_communication_range').get_parameter_value().double_value
        self.drone_health = self.get_parameter('drone_health').get_parameter_value().integer_value
        self.ship_health = self.get_parameter('ship_health').get_parameter_value().integer_value
        self.game_duration = self.get_parameter('game_duration').get_parameter_value().integer_value
        self.remaining_time = self.game_duration
        self.gazebo_world_name = self.get_parameter('gazebo_world_name').get_parameter_value().string_value

    def _init_missile_parameters(self):
        """Initialize missile system parameters."""
        # Missile system parameters
        self.declare_parameter('drone_missile_range', 69.0)
        self.declare_parameter('ship_missile_range', 81.0)
        self.declare_parameter('drone_missile_damage', 1)
        self.declare_parameter('ship_missile_damage', 1)
        self.declare_parameter('drone_cooldown', 8.0)
        self.declare_parameter('ship_cooldown', 6.0)
        self.declare_parameter('drone_magazine', 2)
        self.declare_parameter('ship_magazine', 4)
        self.declare_parameter('laser_width', 5.0)
        
        self.declare_parameter('drone_padding_x', 0.5)
        self.declare_parameter('drone_padding_y', 0.5)
        self.declare_parameter('drone_padding_z', 0.5)
        self.declare_parameter('ship_padding_x', 6.0)
        self.declare_parameter('ship_padding_y', 1.0)
        self.declare_parameter('ship_padding_z', 1.0)

        # Collect parameters into instance variables
        self.drone_missile_range = self.get_parameter('drone_missile_range').get_parameter_value().double_value
        self.ship_missile_range = self.get_parameter('ship_missile_range').get_parameter_value().double_value
        self.drone_missile_damage = self.get_parameter('drone_missile_damage').get_parameter_value().integer_value
        self.ship_missile_damage = self.get_parameter('ship_missile_damage').get_parameter_value().integer_value
        self.drone_cooldown = self.get_parameter('drone_cooldown').get_parameter_value().double_value
        self.ship_cooldown = self.get_parameter('ship_cooldown').get_parameter_value().double_value
        self.drone_magazine = self.get_parameter('drone_magazine').get_parameter_value().integer_value
        self.ship_magazine = self.get_parameter('ship_magazine').get_parameter_value().integer_value
        self.laser_width = self.get_parameter('laser_width').get_parameter_value().double_value

        self.drone_padding_x = self.get_parameter('drone_padding_x').get_parameter_value().double_value
        self.drone_padding_y = self.get_parameter('drone_padding_y').get_parameter_value().double_value
        self.drone_padding_z = self.get_parameter('drone_padding_z').get_parameter_value().double_value
        self.ship_padding_x = self.get_parameter('ship_padding_x').get_parameter_value().double_value
        self.ship_padding_y = self.get_parameter('ship_padding_y').get_parameter_value().double_value
        self.ship_padding_z = self.get_parameter('ship_padding_z').get_parameter_value().double_value

    def _init_kamikaze_parameters(self):
        """Initialize kamikaze system parameters."""
        self.declare_parameter('explosion_damage', 3)
        self.declare_parameter('explosion_range', 5.0)
        
        self.explosion_damage = self.get_parameter('explosion_damage').get_parameter_value().integer_value
        self.explosion_range = self.get_parameter('explosion_range').get_parameter_value().double_value

    ########################################
    # TEAM FORMATION AND HEALTH MANAGEMENT #
    ########################################

    def _initialize_teams(self):
        """Organize robots into two teams based on namespaces."""
        drones = [ns for ns in self.namespaces if '/px4_' in ns]
        ships = [ns for ns in self.namespaces if '/flag_ship_' in ns]

        # Sort drones and ships by numeric index
        drones.sort(key=lambda x: int(x.split('_')[-1]))
        ships.sort(key=lambda x: int(x.split('_')[-1]))

        # Split drones and ships into two teams
        drones_team_1 = (len(drones) + 1) // 2
        ships_team_1 = (len(ships) + 1) // 2
        self.team_1 = drones[:drones_team_1] + ships[:ships_team_1]
        self.team_2 = drones[drones_team_1:] + ships[ships_team_1:]

        # Log warning if teams are uneven
        if len(drones) % 2 != 0 or len(ships) % 2 != 0:
            self.get_logger().warn("Uneven team composition detected.")

        self.get_logger().info(f"Team 1: {self.team_1}")
        self.get_logger().info(f"Team 2: {self.team_2}")

    def _initialize_health_system(self):
        """Initialize health points for all robots."""
        # Ensure namespace format consistency by adding leading slash if missing
        self.namespaces = [ns if ns.startswith('/') else f'/{ns}' for ns in self.namespaces]
        
        # Assign health points based on robot type
        self.health_points = {
            ns: self.drone_health if 'px4_' in ns else self.ship_health
            for ns in self.namespaces if 'px4_' in ns or 'flag_ship' in ns
        }
        self.get_logger().info(f"Initialized health points: {self.health_points}")

    def _initialize_missile_system(self):
        """Initialize missile system tracking."""
        # Initialize magazines for all robots
        self.magazines = {ns: self.drone_magazine if 'px4' in ns else self.ship_magazine 
                         for ns in self.namespaces}
        
        # Store timestamps as ROS Time objects for cooldown tracking
        self.last_fire_time = {ns: self.get_clock().now() for ns in self.namespaces}
    
    def _setup_communications(self):
        """Set up all publishers, subscribers and services."""
        # Publishers for health points, detections, and communications
        self.health_publishers = {ns: self.create_publisher(Int32, f'{ns}/health', 10) for ns in self.namespaces}
        self.detection_publishers = {ns: self.create_publisher(Detections, f'{ns}/detections', 10) for ns in self.namespaces}
        
        # Publishers for boat positions
        ships = [ns for ns in self.namespaces if '/flag_ship_' in ns]
        self.boat_position_publishers = {ns: self.create_publisher(Pose, f'{ns}/localization', 10) for ns in ships}

        # Subscribers for communications
        self.communication_publishers = {ns: self.create_publisher(String, f'{ns}/out_going_messages', 10) for ns in self.namespaces}
        self.communication_subscribers = {ns: self.create_subscription(String, f'{ns}/incoming_messages', lambda msg, ns=ns: self._communication_callback(msg, ns), 10) for ns in self.namespaces}
        
        # Add time publisher (do this before initializing the game timer)
        self.time_publisher = self.create_publisher(Int32, '/game_master/time', 10)
        
        # Timer to periodically publish health status
        self.health_timer = self.create_timer(20.0, self._publish_health_status)
        
        # Create missile and kamikaze services
        self.missile_srv = self.create_service(Missile, '/game_master/fire_missile', self._fire_missile_callback)
        self.get_logger().info("Created fire_missile service")
        
        self.kamikaze_srv = self.create_service(Kamikaze, '/game_master/kamikaze', self._kamikaze_callback)
        self.get_logger().info("Created kamikaze service")

        # Add GameState publisher
        self.game_state_publisher = self.create_publisher(GameState, '/game_master/game_state', 10)
        self.get_logger().info("Created GameState publisher")

    def _get_model_ids(self, namespaces, world_name="game_world_water"):
        """
        Get Gazebo model IDs and names for the list of robot namespaces by determining the drone model automatically.
        The drone model is determined by checking if the `/px4_<px4_number>/laser/scan` topic exists.

        Args:
            namespaces (list): List of robot namespaces
            world_name (str): Name of the Gazebo world

        Returns:
            dict: Mapping of namespace to a dictionary containing model ID and model name
        """
        model_data = {}
        for ns in namespaces:
            if '/px4_' in ns:
                instance_number = ns.split('_')[-1]

                # Check if the laser scan topic exists to determine the model
                laser_scan_topic = f"/px4_{instance_number}/laser/scan"
                topic_list = self.get_topic_names_and_types()
                if any(laser_scan_topic in topic for topic in topic_list):
                    model_name = f"x500_lidar_front_{instance_number}"
                else:
                    model_name = f"x500_{instance_number}"

                # Get the model ID using the determined model name
                model_id = get_model_id(model_name, world_name=world_name)
                if model_id:
                    model_data[ns] = {
                        'id': model_id,
                        'model_name': model_name
                    }
                    self.get_logger().info(f"Found model ID {model_id} for {ns} (model name: {model_name})")
                else:
                    self.get_logger().error(f"Could not find model ID for {ns} (model name: {model_name})")
        return model_data
    
    def _format_initial_positions(self, robot_poses):
        """Format robot positions in a readable way."""
        result = "\nInitial Positions:\n"
        result += "=" * 40 + "\n"
        
        # Group robots by type
        flagships = {}
        drones_team1 = {}
        drones_team2 = {}
        
        for ns, pose in robot_poses.items():
            pos = pose['position']
            if '/flag_ship_' in ns:
                flagships[ns] = pos
            elif '/px4_' in ns:
                if ns in self.team_1:
                    drones_team1[ns] = pos
                else:
                    drones_team2[ns] = pos
        
        # Format flagships
        result += "Flagships:\n"
        for ns, pos in sorted(flagships.items()):
            team = "Team 1" if ns in self.team_1 else "Team 2"
            # Use defensive formatting to handle potential None values
            x = pos[0] if pos[0] is not None else 0.0
            y = pos[1] if pos[1] is not None else 0.0
            z = pos[2] if pos[2] is not None else 0.0
            result += f"  {ns} ({team}): x={x:.1f}, y={y:.1f}, z={z:.1f}\n"
        
        # Format drones - Team 1
        result += "\nTeam 1 Drones:\n"
        for ns, pos in sorted(drones_team1.items(), key=lambda x: x[0]):
            # Use defensive formatting again
            x = pos[0] if pos[0] is not None else 0.0
            y = pos[1] if pos[1] is not None else 0.0
            z = pos[2] if pos[2] is not None else 0.0
            result += f"  {ns}: x={x:.1f}, y={y:.1f}, z={z:.1f}\n"
        
        # Format drones - Team 2
        result += "\nTeam 2 Drones:\n"
        for ns, pos in sorted(drones_team2.items(), key=lambda x: x[0]):
            # And again for team 2
            x = pos[0] if pos[0] is not None else 0.0
            y = pos[1] if pos[1] is not None else 0.0
            z = pos[2] if pos[2] is not None else 0.0
            result += f"  {ns}: x={x:.1f}, y={y:.1f}, z={z:.1f}\n"
        
        return result

    ####################################
    # POSITION AND GAME STATE TRACKING #
    ####################################

    def _update_poses(self, pose, name):
        if pose and all(pose['position'][k] is not None for k in ['x', 'y', 'z']):
            self.robot_poses[name] = {
                'position': (
                    pose['position']['x'],
                    pose['position']['y'],
                    pose['position']['z']
                ),
                'orientation': (
                    pose['orientation']['x'],
                    pose['orientation']['y'],
                    pose['orientation']['z'],
                    pose['orientation']['w']
                )
            }

    def _update_positions_and_distance(self):
        """
        Update the position and orientation of all robots in the simulation.
        Also update the distance cache for all robots to avoid redundant distance calculations.
        """
        try:
            if not self.namespaces:
                self.get_logger().warn("No robot namespaces detected, can't update positions or distances")
                return

            current_time = self.get_clock().now()
            valid_poses = 0

            # Use ThreadPoolExecutor to parallelize position updates
            with ThreadPoolExecutor() as executor:
                futures = {
                    executor.submit(self._update_position_for_namespace, ns, current_time): ns
                    for ns in self.namespaces
                }
                for future in futures:
                    try:
                        valid_poses += future.result()
                    except Exception as e:
                        self.get_logger().error(f"Error updating pose for {futures[future]}: {e}")

            # Use ThreadPoolExecutor to parallelize distance cache updates
            with self.distance_cache_lock:
                self.distance_cache = {}
                with ThreadPoolExecutor() as executor:
                    futures = {
                        executor.submit(self._update_distance_for_namespace, ns1): ns1
                        for ns1 in self.namespaces
                    }
                    for future in futures:
                        try:
                            ns1, distances = future.result()
                            self.distance_cache[ns1] = distances
                        except Exception as e:
                            self.get_logger().error(f"Error updating distances for {futures[future]}: {e}")

            # Publish GameState message if we have valid pose data
            if valid_poses > 0:
                self._publish_game_state()
            else:
                self.get_logger().warn("No valid poses retrieved from any robot")
                return
        except Exception as e:
            self.get_logger().error(f"Error in _update_positions_and_distance: {e}")

    def _update_position_for_namespace(self, ns, current_time):
        """
        Update the position, orientation, and velocity for a single namespace.

        Args:
            ns (str): Namespace of the robot.
            current_time (Time): Current ROS2 time.

        Returns:
            int: 1 if the position was successfully updated, 0 otherwise.
        """
        try:
            pose = self.gz.get_pose(ns)
            # Check if pose has valid values
            if pose['position']['x'] is None:
                self.get_logger().warn(f"Invalid pose data for {ns}")
                return 0

            # Store current position and orientation
            current_position = (pose['position']['x'], 
                                pose['position']['y'], 
                                pose['position']['z'])
            current_orientation = (pose['orientation']['x'],
                                   pose['orientation']['y'],
                                   pose['orientation']['z'],
                                   pose['orientation']['w'])

            # Calculate velocity if we have previous position data
            velocity = (0.0, 0.0, 0.0)  # Default velocity
            if ns in self.robot_poses and ns in self.last_update_time:
                prev_position = self.robot_poses[ns]['position']
                time_diff = (current_time - self.last_update_time[ns]).nanoseconds / 1e9

                if time_diff > 0:
                    # Calculate velocity components (m/s)
                    vx = (current_position[0] - prev_position[0]) / time_diff
                    vy = (current_position[1] - prev_position[1]) / time_diff
                    vz = (current_position[2] - prev_position[2]) / time_diff
                    velocity = (vx, vy, vz)

            # Store updated position, orientation, and velocity
            with self.robot_poses_lock:
                self.robot_poses[ns] = {
                    'position': current_position,
                    'orientation': current_orientation
                }
                self.robot_velocities[ns] = velocity
                self.last_update_time[ns] = current_time

            # Check if a boat and publish its pose as the false flagship odometry system
            if ns.startswith("/flag_ship"):
                suffix = ns.split('_')[2]
                boat_pose = Pose()
                boat_pose.position.x = pose['position']['x']
                boat_pose.position.y = pose["position"]["y"]
                boat_pose.position.z = pose["position"]["z"]
                boat_pose.orientation.x = pose["orientation"]["x"]
                boat_pose.orientation.y = pose["orientation"]["y"]
                boat_pose.orientation.z = pose["orientation"]["z"]
                boat_pose.orientation.w = pose["orientation"]["w"]
                self.boat_position_publishers[f'/flag_ship_{suffix}'].publish(boat_pose)

            return 1
        except Exception as e:
            self.get_logger().error(f"Error updating position for {ns}: {e}")
            return 0

    def _update_distance_for_namespace(self, ns1):
        """
        Update the distance cache for a single namespace.

        Args:
            ns1 (str): Namespace of the robot.

        Returns:
            tuple: (ns1, dict) where dict contains distances to other namespaces.
        """
        distances = {}
        if ns1 not in self.robot_poses or None in self.robot_poses[ns1]['position']:
            return ns1, distances

        for ns2 in self.namespaces:
            if ns1 == ns2 or ns2 not in self.robot_poses or None in self.robot_poses[ns2]['position']:
                continue
            distance = get_distance(self.robot_poses[ns1]['position'], self.robot_poses[ns2]['position'])
            distances[ns2] = distance

        return ns1, distances

    def _publish_game_state(self):
        """
        Publish a GameState message containing the state of all robots.
        Includes position, velocity, health, ammo, and remaining game time.
        Also includes data for destroyed robots at their last known positions.
        """
        try:
            game_state = GameState()
            game_state.remaining_time = float(self.remaining_time)
            robot_states = []

            # Add states for all active robots
            for ns in self.namespaces:
                robot_state = RobotState()
                robot_state.id = int(ns.split('_')[-1])  # Extract ID from namespace
                robot_state.type = "drone" if '/px4_' in ns else "ship"

                # Get regular pose for drones and ships
                if ns in self.robot_poses:
                    pose = self.robot_poses[ns]
                    robot_state.pose.position.x = pose['position'][0]
                    robot_state.pose.position.y = pose['position'][1]
                    robot_state.pose.position.z = pose['position'][2]
                    robot_state.pose.orientation.x = pose['orientation'][0]
                    robot_state.pose.orientation.y = pose['orientation'][1]
                    robot_state.pose.orientation.z = pose['orientation'][2]
                    robot_state.pose.orientation.w = pose['orientation'][3]

                # Add cannon orientation for ships
                if '/flag_ship_' in ns:
                    cannon_key = f"/relative_cannon_{ns.split('_')[-1]}"
                    try:
                        cannon_pose = self.gz.get_pose(cannon_key)
                        if cannon_pose and cannon_pose['orientation']['x'] is not None:
                            robot_state.canon_orientation.x = cannon_pose['orientation']['x']
                            robot_state.canon_orientation.y = cannon_pose['orientation']['y']
                            robot_state.canon_orientation.z = cannon_pose['orientation']['z']
                            robot_state.canon_orientation.w = cannon_pose['orientation']['w']
                    except Exception as e:
                        self.get_logger().warn(f"Could not get cannon orientation for {cannon_key}: {e}")

                # Add velocity, health, and ammo
                if ns in self.robot_velocities:
                    velocity = self.robot_velocities[ns]
                    robot_state.velocity.x = velocity[0]
                    robot_state.velocity.y = velocity[1]
                    robot_state.velocity.z = velocity[2]
                robot_state.health = self.health_points.get(ns, 0)
                robot_state.ammo = self.magazines.get(ns, 0)

                robot_states.append(robot_state)

            # Add states for dead robots
            for ns, data in self.dead_robots.items():
                robot_state = RobotState()
                robot_state.id = int(ns.split('_')[-1])  # Extract ID from namespace
                robot_state.type = "drone"  # Dead robots are always drones (ships end the game)
                
                # Add last known position and orientation
                robot_state.pose.position.x = data['position'][0]
                robot_state.pose.position.y = data['position'][1]
                robot_state.pose.position.z = data['position'][2]
                robot_state.pose.orientation.x = data['orientation'][0]
                robot_state.pose.orientation.y = data['orientation'][1]
                robot_state.pose.orientation.z = data['orientation'][2]
                robot_state.pose.orientation.w = data['orientation'][3]
                
                # Add zero velocity, zero health, and remaining ammo
                robot_state.velocity.x = 0.0
                robot_state.velocity.y = 0.0
                robot_state.velocity.z = 0.0
                robot_state.health = 0
                robot_state.ammo = data['ammo']
                
                robot_states.append(robot_state)

            game_state.robots = robot_states
            self.game_state_publisher.publish(game_state)

        except Exception as e:
            self.get_logger().error(f"Error publishing game state: {e}")

    ###############################
    # DETECTION AND COMMUNICATION #
    ###############################

    def _detections_callback(self):
        """
        Manage the periodic publication of detection information.
        Uses multi-threading to efficiently process and publish detection data for all robots simultaneously.
        """
        # Skip if we're shutting down or don't have valid pose data
        if self.is_shutting_down:
            return
            
        if not self.robot_poses or None in self.robot_poses[next(iter(self.robot_poses))]['position']:
            return

        # Clear previous threads list
        self.detection_threads = []
        
        # Create and start threads
        for ns in self.namespaces:
            thread = threading.Thread(target=self._publish_detections, args=(ns,))
            self.detection_threads.append(thread)
            thread.start()

        # Only wait for threads if we're not shutting down (to avoid deadlock)
        if not self.is_shutting_down:
            for thread in self.detection_threads:
                thread.join(timeout=0.5)  # Set timeout to avoid blocking

    def _publish_detections(self, ns):
        """
        Publish detections for a specific robot.
        :param ns: The namespace of the robot.
        """
        # Skip if we're shutting down
        if self.is_shutting_down:
            return
            
        try:
            detections_msg = Detections()
            detections_msg.header.stamp = self.get_clock().now().to_msg()
            detections_msg.header.frame_id = ns
            detections_msg.detections = self._get_detections(ns)
            
            # Check again before publishing
            if not self.is_shutting_down and ns in self.detection_publishers:
                self.detection_publishers[ns].publish(detections_msg)
        except Exception as e:
            if not self.is_shutting_down:  # Only log if not shutting down
                self.get_logger().error(f"Error publishing detections for {ns}: {e}")

    def _get_detections(self, namespace):
        """
        Calculate which robots are within detection range of the specified robot.

        Args:
            namespace (str): The namespace of the detecting robot

        Returns:
            list[Detection]: List of Detection messages containing:
                - Vehicle type (drone/ship)
                - Friend/foe status
                - Relative position in FRD (Forward-Right-Down) coordinates
                - Uses heading-only orientation for more accurate height representation
        """
        detections = []
        if namespace not in self.robot_poses:
            return detections

        transmitter_pose = self.robot_poses[namespace]
        # Validate transmitter pose
        if None in transmitter_pose['position'] or None in transmitter_pose['orientation']:
            return detections

        # Determine detection range based on the type of the detecting robot
        if '/px4_' in namespace:
            detection_range = self.drone_detection_range
        else:
            detection_range_ship2ship = self.get_parameter('ship2ship_detection_range').get_parameter_value().double_value
            detection_range_ship2drone = self.get_parameter('ship2drone_detection_range').get_parameter_value().double_value

        for robot, receiver_pose in self.robot_poses.items():
            if robot == namespace:
                continue

            # Validate receiver pose
            if None in receiver_pose['position'] or None in receiver_pose['orientation']:
                continue

            try:
                distance = get_distance(transmitter_pose['position'], receiver_pose['position'])

                # Determine detection range based on the type of the detected robot
                if '/px4_' in robot:  # Detected robot is a drone
                    if '/px4_' in namespace:  # Detecting robot is a drone
                        if distance > detection_range:
                            continue
                    else:  # Detecting robot is a ship
                        if distance > detection_range_ship2drone:
                            continue
                else:  # Detected robot is a ship
                    if '/px4_' in namespace:  # Detecting robot is a drone
                        if distance > detection_range:
                            continue
                    else:  # Detecting robot is a ship
                        if distance > detection_range_ship2ship:
                            continue

                detection = Detection()
                detection.vehicle_type = Detection.DRONE if '/px4_' in robot else Detection.SHIP
                detection.is_friend = (namespace in self.team_1 and robot in self.team_1) or (namespace in self.team_2 and robot in self.team_2)

                # Get relative position in FRD frame using heading-only orientation
                # This provides more accurate height representation when the transmitter is tilted
                relative_position = get_relative_position_with_heading(
                    transmitter_pose['position'],
                    transmitter_pose['orientation'],
                    receiver_pose['position']
                )
                detection.relative_position.x = relative_position[0]   # Forward
                detection.relative_position.y = relative_position[1]   # Right 
                detection.relative_position.z = relative_position[2]   # Down

                detections.append(detection)
            except (TypeError, ValueError) as e:
                self.get_logger().warn(f'Error calculating detection between {namespace} and {robot}: {e}')
                continue

        return detections

    def _communication_callback(self, msg, sender_ns):
        """
        Handle incoming communication messages.
        :param msg: The communication message.
        :param sender_ns: The namespace of the sender robot.
        """
        # Check if sender position exists
        if sender_ns not in self.robot_poses:
            self.get_logger().warn(f'No pose data for sender {sender_ns}')
            return
            
        sender_pose = self.robot_poses[sender_ns]
        if None in sender_pose['position']:
            self.get_logger().warn(f'Invalid position for sender {sender_ns}')
            return

        communication_range = self.drone_communication_range if '/px4_' in sender_ns else self.ship_communication_range
        
        for ns in self.namespaces:
            if ns == sender_ns:
                continue

            if ns not in self.robot_poses:
                continue

            receiver_pose = self.robot_poses[ns]
            if None in receiver_pose['position']:
                continue

            distance = get_distance(sender_pose['position'], receiver_pose['position'])
            
            if distance <= communication_range:
                string_msg = String()
                string_msg.data = msg.data
                self.communication_publishers[ns].publish(string_msg)

    ##########################################
    # WEAPON SYSTEMS (MISSILES AND KAMIKAZE) #
    ##########################################

    def _find_targets_in_range(self, source_ns, source_position, range_limit):
        """
        Find all targets within a specified range of the source using the distance cache.

        Args:
            source_ns (str): Namespace of the source (e.g., shooter or kamikaze robot).
            source_position (tuple): Position of the source (x, y, z).
            range_limit (float): Maximum range to consider.

        Returns:
            list: List of (robot, distance, position) tuples for targets in range.
        """
        targets_in_range = []
        if source_ns not in self.distance_cache:
            return targets_in_range

        for target_ns, distance in self.distance_cache[source_ns].items():
            if distance <= range_limit:
                target_pose = self.robot_poses[target_ns]
                targets_in_range.append((target_ns, distance, target_pose['position']))
        return targets_in_range

    def _apply_damage_to_targets(self, targets, damage):
        """
        Apply damage to one or more targets.

        Args:
            targets (list): List of (robot, distance, position) tuples for targets.
            damage (int): Amount of damage to apply.
        """
        for target in targets:
            target_id = target[0]
            self.get_logger().info(f'Applying {damage} damage to {target_id}')
            self._update_health(target_id, damage=damage)

    def _validate_shooter(self, shooter_ns, response):
        """
        Validate shooter exists and has valid pose data.

        Args:
            shooter_ns: Namespace of the shooter.
            response: Service response object to populate if validation fails.

        Returns:
            bool: True if shooter is valid, False otherwise.
        """
        if shooter_ns not in self.namespaces:
            self.get_logger().warn(f'{shooter_ns} not found in robot list. Cannot fire missile.')
            response.has_fired = False
            response.ammo = 0
            return False

        if shooter_ns not in self.robot_poses:
            self.get_logger().warn(f"No pose found for '{shooter_ns}'. Cannot fire missile.")
            response.has_fired = False
            response.ammo = self.magazines[shooter_ns]
            return False

        shooter_pose = self.robot_poses[shooter_ns]
        if None in shooter_pose['position'] or None in shooter_pose['orientation']:
            self.get_logger().warn(f"Invalid pose for '{shooter_ns}'. Cannot fire missile.")
            response.has_fired = False
            response.ammo = self.magazines[shooter_ns]
            return False

        return True

    def _check_ammunition_and_cooldown(self, shooter_ns, response):
        """
        Check if shooter has ammunition and is not in cooldown.

        Args:
            shooter_ns: Namespace of the shooter.
            response: Service response object to populate.

        Returns:
            bool: True if shooter can fire, False otherwise.
        """
        current_time = self.get_clock().now()
        cooldown = self.drone_cooldown if '/px4_' in shooter_ns else self.ship_cooldown

        # Calculate time difference in seconds
        time_since_last_fire = (current_time - self.last_fire_time[shooter_ns]).nanoseconds / 1e9

        # Check ammunition first
        if self.magazines[shooter_ns] <= 0:
            self.get_logger().warn(f'{shooter_ns} cannot fire: Out of ammunition (0 missiles remaining).')
            response.has_fired = False
            response.ammo = self.magazines[shooter_ns]
            return False
        
        # Then check cooldown
        if time_since_last_fire < cooldown:
            time_remaining = cooldown - time_since_last_fire
            self.get_logger().warn(f'{shooter_ns} cannot fire: Cooldown period active (ready in {time_remaining:.1f} seconds).')
            response.has_fired = False
            response.ammo = self.magazines[shooter_ns]
            return False
        
        # If both checks pass, fire the missile
        self.get_logger().info(f'{shooter_ns} fired a missile. Remaining ammo: {self.magazines[shooter_ns] - 1}')
        # Decrease magazine count
        self.magazines[shooter_ns] -= 1
        response.has_fired = True
        response.ammo = self.magazines[shooter_ns]
        # Update last fire time
        self.last_fire_time[shooter_ns] = current_time
        return True

    def _fire_missile_callback(self, request, response):
        """
        Handle missile firing requests.

        Process:
        1. Validate shooter and get their position.
        2. Check ammunition and cooldown.
        3. Find targets within range and alignment.
        4. Apply damage to the closest aligned target.

        Args:
            request: Contains robot_name of the shooter.
            response: Will contain has_fired status and remaining ammo.
        """
        self.get_logger().info(f'Received missile fire request from {request.robot_name}')

        # Validate shooter and check ammunition/cooldown
        shooter_ns = request.robot_name
        if not self._validate_shooter(shooter_ns, response):
            return response
        if not self._check_ammunition_and_cooldown(shooter_ns, response):
            return response

        # Get shooter position and orientation
        if '/flag_ship_' in shooter_ns:
            # Use the global cannon pose for alignment checks
            cannon_pose = self.gz.get_global_cannon_pose(shooter_ns)
            shooter_position = (
                cannon_pose["position"]["x"],
                cannon_pose["position"]["y"],
                cannon_pose["position"]["z"]
            )
            shooter_orientation = (
                cannon_pose["orientation"]["x"],
                cannon_pose["orientation"]["y"],
                cannon_pose["orientation"]["z"],
                cannon_pose["orientation"]["w"]
            )
        else:
            # For drones, use their regular pose
            shooter_pose = self.robot_poses[shooter_ns]
            shooter_position = shooter_pose['position']
            shooter_orientation = shooter_pose['orientation']

        # Find targets within missile range using the distance cache
        missile_range = self.drone_missile_range if '/px4_' in shooter_ns else self.ship_missile_range
        targets_in_range = self._find_targets_in_range(shooter_ns, shooter_position, missile_range)

        # Check alignment and apply damage to the closest target
        aligned_targets = self._find_aligned_targets(shooter_position, shooter_orientation, targets_in_range, missile_range)
        if aligned_targets:
            closest_target = min(aligned_targets, key=lambda x: x[1])
            self._apply_damage_to_targets([closest_target], self.drone_missile_damage if '/px4_' in shooter_ns else self.ship_missile_damage)
        else:
            self.get_logger().info(f'MISSILE MISSED: No aligned targets for {shooter_ns}')

        return response

    def _kamikaze_callback(self, request, response):
        """
        Handle the kamikaze service request.

        Process:
        1. Validate kamikaze robot and get position.
        2. Apply damage to the kamikaze robot itself.
        3. Find all robots within explosion range.
        4. Apply explosion damage to all affected robots.

        Args:
            request: Contains robot_name of the kamikaze robot.
            response: Empty response.
        """
        kamikaze_ns = request.robot_name
        self.get_logger().info(f'Received kamikaze request from {kamikaze_ns}')

        # Ensure only drones can use the kamikaze service
        if '/flag_ship_' in kamikaze_ns:
            self.get_logger().warn(f"Kamikaze service is not available for flagships: {kamikaze_ns}")
            return response

        if not kamikaze_ns or kamikaze_ns not in self.namespaces:
            self.get_logger().warn(f'{kamikaze_ns} not found in robot list. Cannot detonate.')
            return response

        # Check if robot has a valid pose
        if kamikaze_ns not in self.robot_poses:
            self.get_logger().warn(f"No pose data for {kamikaze_ns}")
            return response

        kamikaze_pose = self.robot_poses[kamikaze_ns]
        kamikaze_position = kamikaze_pose['position']

        if None in kamikaze_position:
            self.get_logger().warn(f"Invalid position data for {kamikaze_ns}")
            return response

        self.get_logger().info(f'💥 KAMIKAZE: {kamikaze_ns} detonating at position {kamikaze_position}')

        # 1. Apply damage to the kamikaze robot itself
        self._update_health(kamikaze_ns, damage=self.explosion_damage, from_kamikaze=True)

        # 2. Find all robots within explosion range using the distance cache
        targets_in_range = self._find_targets_in_range(kamikaze_ns, kamikaze_position, self.explosion_range)

        # 3. Apply explosion damage to all affected robots (pass from_kamikaze=True)
        for target in targets_in_range:
            target_id = target[0]
            self.get_logger().info(f'Applying {self.explosion_damage} damage to {target_id}')
            self._update_health(target_id, damage=self.explosion_damage, from_kamikaze=True)

        self.get_logger().info(f'Kamikaze explosion from {kamikaze_ns} affected {len(targets_in_range)} robots')
        return response

    def _find_aligned_targets(self, shooter_position, shooter_orientation, targets_in_range, missile_range):
        """
        Filter targets that are aligned with the shooter's orientation.

        Args:
            shooter_position (tuple): Position of the shooter (x, y, z).
            shooter_orientation (tuple): Orientation of the shooter (x, y, z, w).
            targets_in_range (list): List of targets in range, each as (robot, distance, position).
            missile_range (float): Maximum range of the missile.

        Returns:
            list: List of aligned targets, each as (robot, distance, position).
        """
        aligned_targets = []
        for target in targets_in_range:
            target_id = target[0]
            target_position = target[2]

            # Determine target padding based on type
            if '/px4_' in target_id:
                target_padding = (self.drone_padding_x, self.drone_padding_y, self.drone_padding_z)
            else:
                target_padding = (self.ship_padding_x, self.ship_padding_y, self.ship_padding_z)

            # Check alignment using is_aligned_HB
            aligned_result = is_aligned_HB(
                self,
                shooter_position,
                shooter_orientation,
                target_position,
                target_padding,
                missile_range,  # base_length
                self.laser_width,  # base_radius
                verbose=1
            )
            if aligned_result:
                aligned_targets.append(target)

        return aligned_targets


    ################################
    # HEALTH AND DAMAGE MANAGEMENT #
    ################################

    def _update_health(self, ns, health=None, damage=0, from_kamikaze=False):
        """
        Update the health of the specified robot by setting a new health value or reducing it by a given damage amount.
        :param ns: The namespace of the robot.
        :param health: The health value to set (optional).
        :param damage: The amount of damage to apply (optional).
        :param from_kamikaze: Flag to prevent recursive kamikaze calls (optional).
        """
        # Validate namespace input
        if not ns or not isinstance(ns, str):
            self.get_logger().warn(f'Invalid namespace: {ns}. Cannot update health.')
            return
            
        if ns not in self.namespaces and ns not in self.dead_robots:
            self.get_logger().warn(f'{ns} not found in robot list : {self.namespaces}. Cannot update health.')
            return
            
        if damage > 0:
            # Calculate damage
            current_health = self.health_points.get(ns, 0)
            health = max(0, current_health - damage)
            
        if health is not None:
            self.health_points[ns] = health
            self.get_logger().debug(f'{ns} health is now {health}')
            # send health data to the health topic for those that are interested
            health_msg = Int32()
            health_msg.data = health
            self.health_publishers[ns].publish(health_msg)

            if health == 0:
                # If a drone is killed, remove it from the simulation
                if 'px4_' in ns:
                    # Only log DESTROYED for initial destruction, not for kamikaze self-damage
                    if not from_kamikaze:
                        self.get_logger().info(f'💥 DESTROYED: {ns} is eliminated')
                        # Create a request for the kamikaze service
                        request = Kamikaze.Request()
                        request.robot_name = ns
                        # Process it directly rather than using a client
                        self._kamikaze_callback(request, Kamikaze.Response())
                    
                    # Store the final state regardless of destruction source
                    if ns in self.namespaces and ns in self.robot_poses:
                        self.dead_robots[ns] = {
                            'position': self.robot_poses[ns]['position'],
                            'orientation': self.robot_poses[ns]['orientation'],
                            'velocity': (0.0, 0.0, 0.0),
                            'health': 0,
                            'ammo': self.magazines.get(ns, 0)
                        }
                        self.get_logger().info(f'Stored final state for {ns}')
                    
                    # Safely remove the namespace if it exists
                    if ns in self.namespaces:
                        self.get_logger().info(f'Removing {ns} from namespaces list')
                        self.namespaces.remove(ns)
                        # Update the GazeboPosesTracker with the new list of namespaces
                        self.gz = GazeboPosesTracker(self.namespaces, logger=self.get_logger(), world_name=self.gazebo_world_name)
                        # If the new GazeboPosesTracker is not fully initialized and the model of a drone is removed,
                        # it will not be able to find it and gazebo_subscriber.py will crash
                        time.sleep(0.5)
                        # Now kill the drone and remove it using imported functions
                        self._kill_drone(ns)
                    else:
                        self.get_logger().warn(f'{ns} already removed from namespaces list')

                # If a flagship is destroyed, end the game and determine the winner
                # Then stop the simulation
                elif 'flag_ship_' in ns:
                    self.get_logger().info(f'💥 FLAGSHIP DESTROYED: {ns}')              
                    self._end_game()
        return

    def _publish_health_status(self):
        """
        Publish health status for all robots every 2 seconds.
        """
        health_status = {ns: self.health_points[ns] for ns in self.namespaces}
        for ns in self.namespaces:
            health_msg = Int32()
            health_msg.data = self.health_points[ns]
            self.health_publishers[ns].publish(health_msg)
        if self.namespaces:
            self.get_logger().info(f'Robot health: {health_status}')

    def _kill_drone(self, namespace):
        """
        Remove a drone from the simulation when it's destroyed.
        Uses kill_drone_from_game_master in non-blocking mode to avoid holding up the game master.
        
        Args:
            namespace (str): The namespace of the drone to remove
        """
        start_time = self.get_clock().now()
        
        if namespace in self.drone_models:
            # Get model data
            model_data = self.drone_models[namespace]
            
            # Extract the base name pattern (remove instance number)
            model_name = model_data['model_name']
            base_name_parts = model_name.rsplit('_', 1)[0]
            
            # Call the kill_drone_from_game_master function with non-blocking mode
            success = kill_drone_from_game_master(
                namespace=namespace,
                drone_model_base_name=base_name_parts,
                drone_models={namespace: model_data['id']},
                logger=self.get_logger(),
                world_name=self.gazebo_world_name,
                gz_node=self.gz_node,
                non_blocking=True
            )
            
            # Calculate and log duration
            end_time = self.get_clock().now()
            duration = (end_time - start_time).nanoseconds / 1e9
            self.get_logger().info(f"⏱️ Drone kill operation for {namespace} initiated in {duration:.4f} seconds")
            
            if not success:
                self.get_logger().error(f"Failed to initiate kill process for {namespace}")
        else:
            self.get_logger().warn(f"No model data found for {namespace}, cannot kill drone")

    ###############################
    # GAME TIMING AND TERMINATION #
    ###############################

    def _game_timer_callback(self):
        """
        Monitor game progress and check for end conditions.
        Tracks:
        - Remaining game time
        - Publishes the remaining time to /game_master/time topic.
        """
        if self.start_time.seconds_nanoseconds()[0] == 0:
            self.start_time = self.get_clock().now()
            self.get_logger().info("Waiting for clock synchronization...")
            time.sleep(1)
            return

        current_time = self.get_clock().now()
        # Calculate elapsed time using proper ROS2 time methods
        elapsed_duration = current_time - self.start_time
        elapsed_seconds = elapsed_duration.nanoseconds / 1e9      
        self.remaining_time = max(0, int(self.game_duration - elapsed_seconds))
        
        # Publish remaining time
        time_msg = Int32()
        time_msg.data = self.remaining_time
        self.time_publisher.publish(time_msg)
        
        # Safety check - ensure we only end if elapsed time is reasonable (prevent false end on startup)
        if elapsed_seconds > 1.0 and elapsed_seconds >= self.game_duration:
            self.get_logger().info(f"Game duration reached: {elapsed_seconds:.2f} >= {self.game_duration}")
            self._end_game()
        
        # Add periodic progress updates with flagship health
        if int(elapsed_seconds) % 60 == 0 and int(elapsed_seconds) > 0:
            self.get_logger().info(f"GAME STATUS: {elapsed_seconds:.0f}/{self.game_duration} seconds elapsed, {self.remaining_time} seconds remaining")
            
            # Calculate current flagship health (works if there more than 1 flsh_ship too)
            team1_ship_health = sum(self.health_points.get(ns, 0) for ns in self.team_1 if '/flag_ship_' in ns)
            team2_ship_health = sum(self.health_points.get(ns, 0) for ns in self.team_2 if '/flag_ship_' in ns)
            
            # Count surviving drones
            team1_surviving_drones = len([ns for ns in self.team_1 if '/px4_' in ns and self.health_points.get(ns, 0) > 0])
            team2_surviving_drones = len([ns for ns in self.team_2 if '/px4_' in ns and self.health_points.get(ns, 0) > 0])
            
            self.get_logger().info(f"STATUS: Team 1 ship health: {team1_ship_health}, drones: {team1_surviving_drones}")
            self.get_logger().info(f"STATUS: Team 2 ship health: {team2_ship_health}, drones: {team2_surviving_drones}")

    def _end_game(self):
        """
        Handle game termination and results logging based on new win conditions:
        1. Team that destroyed enemy flagship wins
        2. Otherwise, team with healthiest flagship wins
        3. If flagship health is tied, team with most surviving drones wins
        4. If everything is tied, it's a draw
        """
        self.get_logger().info('Game Over')
        # Calculate current flagship health (works if there more than 1 flsh_ship too)
        team1_ship_health = sum(self.health_points.get(ns, 0) for ns in self.team_1 if '/flag_ship_' in ns)
        team2_ship_health = sum(self.health_points.get(ns, 0) for ns in self.team_2 if '/flag_ship_' in ns)   
        # Count surviving drones
        team1_surviving_drones = len([ns for ns in self.team_1 if '/px4_' in ns and self.health_points.get(ns, 0) > 0])
        team2_surviving_drones = len([ns for ns in self.team_2 if '/px4_' in ns and self.health_points.get(ns, 0) > 0])
        
        # Determine the winner based on new rules
        result = "Game Over\n"
        
        if team2_ship_health == 0:
            result += 'Team 1 wins! (Destroyed enemy flagship)\n'
            winner = "team_1"
        elif team1_ship_health == 0:
            result += 'Team 2 wins! (Destroyed enemy flagship)\n'
            winner = "team_2"
        elif team1_ship_health > team2_ship_health:
            result += 'Team 1 wins! (Flagship has more health)\n'
            winner = "team_1"
        elif team2_ship_health > team1_ship_health:
            result += 'Team 2 wins! (Flagship has more health)\n'
            winner = "team_2"
        elif team1_surviving_drones > team2_surviving_drones:
            result += 'Team 1 wins! (Has more surviving drones)\n'
            winner = "team_1"
        elif team2_surviving_drones > team1_surviving_drones:
            result += 'Team 2 wins! (Has more surviving drones)\n'
            winner = "team_2"
        else:
            result += 'The game is a draw! (Equal flagship health and surviving drones)\n'
            winner = "draw"
        
        # Get surviving and destroyed drones for Team 1
        team1_surviving_drones_names = [ns for ns in self.team_1 if '/px4_' in ns and self.health_points.get(ns, 0) > 0]
        team1_destroyed_drones_names = [ns for ns in self.team_1 if '/px4_' in ns and self.health_points.get(ns, 0) == 0]

        # Get surviving and destroyed drones for Team 2
        team2_surviving_drones_names = [ns for ns in self.team_2 if '/px4_' in ns and self.health_points.get(ns, 0) > 0]
        team2_destroyed_drones_names = [ns for ns in self.team_2 if '/px4_' in ns and self.health_points.get(ns, 0) == 0]

        # Add detailed statistics
        result += f"Team 1 flagship health: {team1_ship_health}/{len([ns for ns in self.team_1 if '/flag_ship_' in ns]) * self.ship_health}\n"
        result += f"Team 1 surviving drones: {', '.join(team1_surviving_drones_names) if team1_surviving_drones_names else 'None'}\n"
        result += f"Team 1 destroyed drones: {', '.join(team1_destroyed_drones_names) if team1_destroyed_drones_names else 'None'}\n"
        result += f"Team 2 flagship health: {team2_ship_health}/{len([ns for ns in self.team_2 if '/flag_ship_' in ns]) * self.ship_health}\n"
        result += f"Team 2 surviving drones: {', '.join(team2_surviving_drones_names) if team2_surviving_drones_names else 'None'}\n"
        result += f"Team 2 destroyed drones: {', '.join(team2_destroyed_drones_names) if team2_destroyed_drones_names else 'None'}\n"
        result += f"Winner: {winner}\n"

        self._save_game_results(result)
        
        self.get_logger().info("Results written successfully. Shutting down.")
        # Create a one-shot timer that will trigger shutdown after a short delay
        self.shutdown_timer = self.create_timer(1.0, self._delayed_shutdown, callback_group=rclpy.callback_groups.MutuallyExclusiveCallbackGroup())
        
    def _save_game_results(self, result):
        """
        Save game results to files.
        
        Args:
            result (str): Formatted game results
        """
        # Locate the SWARMz4 directory
        home_dir = os.path.expanduser("~")
        swarmz4_path = None
        for root, dirs, files in os.walk(home_dir):
            if 'SWARMz4' in dirs:
                swarmz4_path = os.path.join(root, 'SWARMz4')
                break

        if not swarmz4_path:
            self.get_logger().info("SWARMz4 directory not found. Logging results instead.")
            return

        # Create results directory if it doesn't exist
        result_dir = os.path.join(swarmz4_path, 'results')
        if not os.path.exists(result_dir):
            self.get_logger().info(f"Creating results directory at: {result_dir}")
            os.makedirs(result_dir)
        else:
            self.get_logger().info(f"Results directory already exists at: {result_dir}")

        # Write to game_results.txt
        result_file = os.path.join(result_dir, 'game_results.txt')
        
        # Initialize game_number to 1 (for first game) before checking if file exists
        game_number = 1
        
        if os.path.exists(result_file):
            self.get_logger().info(f"Result file exists. Reading current content.")
            with open(result_file, 'r') as file:
                content = file.read()
                game_number = content.count("--- Game") + 1
            self.get_logger().info(f"Appending results for Game {game_number}.")
            with open(result_file, 'a') as file:
                file.write(f"\n--- Game {game_number} ---\n")
                file.write(result)
        else:
            self.get_logger().info(f"Result file does not exist. Creating new file.")
            with open(result_file, 'w') as file:
                file.write(f"--- Game {game_number} ---\n")
                file.write(result)

        # Write to individual_games
        individual_games_dir = os.path.join(result_dir, 'individual_games')
        # Ensure the directory exists
        if not os.path.exists(individual_games_dir):
            self.get_logger().info(f"Creating individual games directory at: {individual_games_dir}")
            os.makedirs(individual_games_dir)
        else:
            self.get_logger().info(f"Individual games directory already exists at: {individual_games_dir}")

        # Create a unique filename with a timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        individual_result_file = os.path.join(individual_games_dir, f'game_results_{timestamp}.txt')
        with open(individual_result_file, 'w') as file:
            file.write(result)

    def _delayed_shutdown(self):
        """Perform delayed shutdown after game ends"""
        self.get_logger().info("Shutting down...")
        
        # Set the shutdown flag to prevent new thread creation
        self.is_shutting_down = True
        
        # Wait for any existing detection threads to finish (with timeout)
        for thread in self.detection_threads:
            if thread.is_alive():
                thread.join(timeout=0.5)
        
        # Cancel all timers
        try:
            self.timer.cancel()
            self.update_positions_timer.cancel()
            self.game_timer.cancel()
            self.health_timer.cancel()
            self.shutdown_timer.cancel()
        except Exception as e:
            self.get_logger().warn(f"Error canceling timers: {e}")

        # Create a thread to actually shut down ROS after a short delay
        # This is necessary because we can't call rclpy.shutdown() directly from a callback
        threading.Thread(target=self._actual_shutdown).start()

    def _actual_shutdown(self):
        """Actually shut down ROS after a small delay"""
        time.sleep(0.5)  # Small delay to allow final messages to be logged
        self.get_logger().info("Terminating ROS...")
        # Force shutdown
        rclpy.shutdown()
        
def main(args=None):
    rclpy.init(args=args)
    game_master_node = GameMasterNode()
    
    # Set up a clean destruction process
    def cleanup_and_shutdown():
        if rclpy.ok():
            rclpy.shutdown()
    
    # Use atexit to ensure proper cleanup
    import atexit
    atexit.register(cleanup_and_shutdown)
    
    executor = rclpy.executors.MultiThreadedExecutor()
    try:
        executor.add_node(game_master_node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        game_master_node.destroy_node()
        # Only call shutdown if ROS context is still initialized
        cleanup_and_shutdown()


if __name__ == '__main__':
    main()