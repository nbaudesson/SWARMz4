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
- Game state management
- Automated game results logging
- Missile firing system
- Kamikaze/self-destruct functionality
"""

import rclpy
from builtin_interfaces.msg import Time
import time
from rclpy.node import Node
from std_msgs.msg import String, Int32
from utils.gazebo_subscriber import GazeboPosesTracker
from swarmz_interfaces.msg import Detections, Detection, GameState, RobotState
from utils.tools import get_distance, get_relative_position_with_orientation, get_relative_position_with_heading, get_stable_namespaces, is_aligned_HB
from utils.kill_drone import get_model_id, kill_drone_from_game_master
from swarmz_interfaces.srv import Kamikaze, Missile
import time
import threading
import os
from datetime import datetime
from gz.transport13 import Node as GzNode
from geometry_msgs.msg import Pose
from concurrent.futures import ThreadPoolExecutor

class GameMasterNode(Node):
    """
    Main node for managing the combat simulation game with a centralized robot data structure.
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

        # Initialize the centralized robot dictionary and locks
        self.robots = {}  # Main dictionary to store all robot data
        self.robot_lock = threading.Lock()  # Lock for thread safety
        self.is_shutting_down = False
        self.detection_threads = []

        # Team tracking sets
        self.team_1 = set()
        self.team_2 = set()

        # Initialize game parameters
        self._init_game_parameters()
        self._init_missile_parameters()
        self._init_kamikaze_parameters()

        # Detect robot namespaces
        self.get_logger().info("Detecting robot namespaces...")
        namespaces = get_stable_namespaces(self, max_attempts=10, wait_time=1.0)

        # Initialize robot dictionary with detected robots
        if not self._initialize_robots(namespaces):
            self.get_logger().error("Failed to initialize robots due to unbalanced teams. Exiting.")
            return

        # Set up publishers, subscribers, and services
        self._setup_communications()
        
        # Initialize Gazebo pose tracker
        self.gz = GazeboPosesTracker(list(self.robots.keys()), logger=self.get_logger(), world_name=self.gazebo_world_name)
        self.get_logger().info("Gazebo poses tracker initialized")
        time.sleep(1)  # Allow time for Gazebo to initialize

        # Update initial positions
        self._update_positions_and_distance()
        
        # Setup game timers
        self.timer = self.create_timer(0.5, self._detections_callback)
        self.update_positions_timer = self.create_timer(0.1, self._update_positions_and_distance)
        
        # Initialize game timer
        current_time = self.get_clock().now()
        self.start_time = current_time
        self.game_timer = self.create_timer(1.0, self._game_timer_callback)
        
        self.get_logger().info("Game Master Node initialized successfully")
    
    ###########################################
    # FORMAT HELPERS FOR LOGGING
    ###########################################
    
    def _format_float(self, value):
        """Format a float value to 2 decimal places."""
        if isinstance(value, float):
            return round(value, 2)
        return value
    
    def _format_position(self, pos):
        """Format position dictionary to show only 2 decimal places for readability."""
        if not pos:
            return pos
        return {
            'x': round(pos['x'], 2) if isinstance(pos['x'], (float, int)) and pos['x'] is not None else pos['x'],
            'y': round(pos['y'], 2) if isinstance(pos['y'], (float, int)) and pos['y'] is not None else pos['y'],
            'z': round(pos['z'], 2) if isinstance(pos['z'], (float, int)) and pos['z'] is not None else pos['z']
        }
    
    def _format_orientation(self, ori):
        """Format orientation dictionary to show only 2 decimal places for readability."""
        if not ori:
            return ori
        return {
            'x': round(ori['x'], 2) if isinstance(ori['x'], (float, int)) and ori['x'] is not None else ori['x'],
            'y': round(ori['y'], 2) if isinstance(ori['y'], (float, int)) and ori['y'] is not None else ori['y'],
            'z': round(ori['z'], 2) if isinstance(ori['z'], (float, int)) and ori['z'] is not None else ori['z'],
            'w': round(ori['w'], 2) if isinstance(ori['w'], (float, int)) and ori['w'] is not None else ori['w']
        }
    
    def _format_dict_with_floats(self, d):
        """Format all float values in a dictionary to 2 decimal places."""
        if not isinstance(d, dict):
            return d
        
        result = {}
        for k, v in d.items():
            if isinstance(v, float):
                result[k] = round(v, 2)
            elif isinstance(v, dict):
                result[k] = self._format_dict_with_floats(v)
            elif isinstance(v, list):
                result[k] = [round(x, 2) if isinstance(x, float) else x for x in v]
            else:
                result[k] = v
        return result

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
        self.declare_parameter('laser_width', 4.0)
        
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

    def _initialize_robots(self, namespaces):
        """Initialize the robot dictionary with detected namespaces."""
        # Make sure namespaces are properly formatted
        namespaces = [ns if ns.startswith('/') else f'/{ns}' for ns in namespaces]
        
        # Sort robots by type and index
        drones = sorted([ns for ns in namespaces if '/px4_' in ns], 
                    key=lambda x: int(x.split('_')[-1]))
        ships = sorted([ns for ns in namespaces if '/flag_ship_' in ns], 
                key=lambda x: int(x.split('_')[-1]))
        
        # Check if we have enough ships for balanced teams
        if len(ships) < 2:
            self.get_logger().error("Not enough ships detected for a balanced game (minimum 2 required).")
            self._safe_shutdown_unbalanced()
            return False
        
        # Team assignment
        drones_team_1 = (len(drones) + 1) // 2
        ships_team_1 = (len(ships) + 1) // 2
        
        team1_drones = drones[:drones_team_1]
        team2_drones = drones[drones_team_1:]
        team1_ships = ships[:ships_team_1]
        team2_ships = ships[ships_team_1:]
        
        # Check for team balance
        # Both teams should have at least one ship
        if not team1_ships or not team2_ships:
            self.get_logger().error(f"Unbalanced teams: Each team must have at least one ship.")
            self._safe_shutdown_unbalanced()
            return False
        
        # Check drone balance - allow a maximum difference of 1 drone
        drone_diff = abs(len(team1_drones) - len(team2_drones))
        if drone_diff > 1:
            self.get_logger().error(f"Unbalanced teams: Drone count difference too large ({len(team1_drones)} vs {len(team2_drones)}).")
            self._safe_shutdown_unbalanced()
            return False
        
        # Update team sets
        self.team_1 = set(team1_drones + team1_ships)
        self.team_2 = set(team2_drones + team2_ships)
        
        # Log team assignments
        self.get_logger().info(f"Team 1: {self.team_1}")
        self.get_logger().info(f"Team 2: {self.team_2}")
        
        # Get Gazebo model IDs for drones
        drone_model_data = self._get_model_ids(drones, self.gazebo_world_name)
        
        # Initialize robot dictionary
        for ns in namespaces:
            is_drone = '/px4_' in ns
            instance = int(ns.split('_')[-1])
            team = 1 if ns in self.team_1 else 2
            
            # Base robot data
            self.robots[ns] = {
                "type": "drone" if is_drone else "ship",
                "instance": instance,
                "team": team,
                "model_id": None,
                "model_name": None,
                "position": {"x": None, "y": None, "z": None},
                "orientation": {"x": None, "y": None, "z": None, "w": None},
                "distances": {},
                "health": self.drone_health if is_drone else self.ship_health,
                "ammo": self.drone_magazine if is_drone else self.ship_magazine,
                "last_fire_time": self.get_clock().now()
            }
            
            # Add model data if available for drones
            if is_drone and ns in drone_model_data:
                self.robots[ns]["model_id"] = drone_model_data[ns]['id']
                self.robots[ns]["model_name"] = drone_model_data[ns]['model_name']
        
        self.get_logger().info(f"Initialized {len(self.robots)} robots")
        return True

    def _safe_shutdown_unbalanced(self):
        """Safely shut down the node when teams are unbalanced."""
        self.get_logger().error("Shutting down due to unbalanced teams.")
        self.get_logger().error("For a fair game, ensure both teams have at least one ship and similar numbers of drones.")
        
        # Set the shutdown flag to prevent new thread creation
        self.is_shutting_down = True
        
        # Create a thread to actually shut down ROS after a short delay
        threading.Thread(target=self._actual_shutdown).start()

    def _setup_communications(self):
        """Set up all publishers, subscribers and services."""
        # Publishers for health points, detections, and communications
        self.health_publishers = {ns: self.create_publisher(Int32, f'{ns}/health', 10) for ns in self.robots.keys()}
        self.detection_publishers = {ns: self.create_publisher(Detections, f'{ns}/detections', 10) for ns in self.robots.keys()}
        
        # Publishers for boat positions
        ships = [ns for ns in self.robots.keys() if '/flag_ship_' in ns]
        self.boat_position_publishers = {ns: self.create_publisher(Pose, f'{ns}/localization', 10) for ns in ships}

        # Subscribers for communications
        self.communication_publishers = {ns: self.create_publisher(String, f'{ns}/out_going_messages', 10) for ns in self.robots.keys()}
        self.communication_subscribers = {ns: self.create_subscription(String, f'{ns}/incoming_messages', 
                                          lambda msg, ns=ns: self._communication_callback(msg, ns), 10) 
                                         for ns in self.robots.keys()}
        
        # Add time publisher
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
        Get Gazebo model IDs and names for the list of robot namespaces.
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
                model_id = get_model_id(model_name, logger=self.get_logger(), world_name=world_name)
                if model_id:
                    model_data[ns] = {
                        'id': model_id,
                        'model_name': model_name
                    }
                    self.get_logger().info(f"Found model ID {model_id} for {ns} (model name: {model_name})")
                else:
                    self.get_logger().error(f"Could not find model ID for {ns} (model name: {model_name})")
        return model_data

    ####################################
    # POSITION AND DISTANCE TRACKING   #
    ####################################

    def _update_positions_and_distance(self):
        """
        Update the position and orientation of all alive robots in the simulation.
        Also update distances between robots.
        """
        try:
            if not self.robots:
                self.get_logger().warn("No robots detected, can't update positions or distances")
                return

            valid_poses = 0
            
            # Update positions for alive robots only (health > 0)
            alive_robots = [ns for ns, robot in self.robots.items() if robot["health"] > 0]
            
            with ThreadPoolExecutor() as executor:
                futures = {
                    executor.submit(self._update_position_for_namespace, ns): ns
                    for ns in alive_robots
                }
                for future in futures:
                    try:
                        valid_poses += future.result()
                    except Exception as e:
                        self.get_logger().error(f"Error updating pose for {futures[future]}: {e}")

            # Update distances between alive robots
            with ThreadPoolExecutor() as executor:
                futures = {
                    executor.submit(self._update_distance_for_namespace, ns): ns
                    for ns in alive_robots
                }
                for future in futures:
                    try:
                        future.result()
                    except Exception as e:
                        self.get_logger().error(f"Error updating distances for {futures[future]}: {e}")

            # Publish GameState message if we have valid pose data
            if valid_poses > 0:
                self._publish_game_state()
            else:
                self.get_logger().warn("No valid poses retrieved from any robot")
        
        except Exception as e:
            self.get_logger().error(f"Error in _update_positions_and_distance: {e}")

    def _update_position_for_namespace(self, ns):
        """
        Update the position and orientation for a single namespace.
        
        Args:
            ns (str): Namespace of the robot
            
        Returns:
            int: 1 if the position was successfully updated, 0 otherwise
        """
        try:
            if ns not in self.robots or self.robots[ns]["health"] <= 0:
                return 0
                
            pose = self.gz.get_pose(ns)
            
            # Check if pose has valid values
            if pose['position']['x'] is None:
                self.get_logger().warn(f"Invalid pose data for {ns}")
                return 0

            with self.robot_lock:
                # Update position and orientation
                self.robots[ns]["position"]["x"] = pose['position']['x']
                self.robots[ns]["position"]["y"] = pose['position']['y']
                self.robots[ns]["position"]["z"] = pose['position']['z']
                self.robots[ns]["orientation"]["x"] = pose['orientation']['x']
                self.robots[ns]["orientation"]["y"] = pose['orientation']['y']
                self.robots[ns]["orientation"]["z"] = pose['orientation']['z']
                self.robots[ns]["orientation"]["w"] = pose['orientation']['w']

            # Check if a ship and publish its pose
            if self.robots[ns]["type"] == "ship":
                boat_pose = Pose()
                boat_pose.position.x = pose['position']['x']
                boat_pose.position.y = pose['position']['y']
                boat_pose.position.z = pose['position']['z']
                boat_pose.orientation.x = pose['orientation']['x']
                boat_pose.orientation.y = pose['orientation']['y']
                boat_pose.orientation.z = pose['orientation']['z']
                boat_pose.orientation.w = pose['orientation']['w']
                
                if ns in self.boat_position_publishers:
                    self.boat_position_publishers[ns].publish(boat_pose)

            return 1
        except Exception as e:
            self.get_logger().error(f"Error updating position for {ns}: {e}")
            return 0

    def _update_distance_for_namespace(self, ns1):
        """
        Update the distance cache for a single namespace.
        
        Args:
            ns1 (str): Namespace of the robot
        """
        if ns1 not in self.robots or self.robots[ns1]["health"] <= 0:
            return
            
        robot1 = self.robots[ns1]
        pos1 = robot1["position"]
        
        if pos1["x"] is None or pos1["y"] is None or pos1["z"] is None:
            return
        
        # Convert to tuple format for the get_distance function
        position1 = (pos1["x"], pos1["y"], pos1["z"])
        
        # Calculate distances to all other alive robots
        with self.robot_lock:
            robot1["distances"] = {}
            for ns2, robot2 in self.robots.items():
                if ns1 == ns2 or robot2["health"] <= 0:
                    continue
                    
                pos2 = robot2["position"]
                
                if pos2["x"] is None or pos2["y"] is None or pos2["z"] is None:
                    continue
                    
                position2 = (pos2["x"], pos2["y"], pos2["z"])
                
                # Calculate distance
                distance = get_distance(position1, position2)
                robot1["distances"][ns2] = distance

    def _publish_game_state(self):
        """
        Publish a GameState message containing the state of all robots.
        """
        try:
            game_state = GameState()
            game_state.remaining_time = float(self.remaining_time)
            
            # Add simulation time (from ROS clock)
            game_state.sim_time = self.get_clock().now().to_msg()
            
            # Get current wall time and convert to ROS Time message
            current_time = time.time_ns()
            seconds = current_time // 1_000_000_000
            nanoseconds = current_time % 1_000_000_000
            ros_time = Time()
            ros_time.sec = int(seconds)
            ros_time.nanosec = int(nanoseconds)
            game_state.real_time = ros_time
            
            robot_states = []

            # Process all robots
            for ns, robot in self.robots.items():
                robot_state = RobotState()
                robot_state.id = robot["instance"]
                robot_state.type = robot["type"]
                
                # Add position and orientation
                pos = robot["position"]
                ori = robot["orientation"]
                
                robot_state.pose.position.x = pos["x"] if pos["x"] is not None else 0.0
                robot_state.pose.position.y = pos["y"] if pos["y"] is not None else 0.0
                robot_state.pose.position.z = pos["z"] if pos["z"] is not None else 0.0
                robot_state.pose.orientation.x = ori["x"] if ori["x"] is not None else 0.0
                robot_state.pose.orientation.y = ori["y"] if ori["y"] is not None else 0.0
                robot_state.pose.orientation.z = ori["z"] if ori["z"] is not None else 0.0
                robot_state.pose.orientation.w = ori["w"] if ori["w"] is not None else 1.0
                
                # Add health and ammo
                robot_state.health = robot["health"]
                robot_state.ammo = robot["ammo"]
                
                # For ships, add cannon orientation only if alive
                if robot["type"] == "ship" and robot["health"] > 0:
                    cannon_key = f"/relative_cannon_{robot['instance']}"
                    try:
                        cannon_pose = self.gz.get_pose(cannon_key)
                        if cannon_pose and cannon_pose['orientation']['x'] is not None:
                            robot_state.canon_orientation.x = cannon_pose['orientation']['x']
                            robot_state.canon_orientation.y = cannon_pose['orientation']['y']
                            robot_state.canon_orientation.z = cannon_pose['orientation']['z']
                            robot_state.canon_orientation.w = cannon_pose['orientation']['w']
                    except Exception as e:
                        self.get_logger().warn(f"Could not get cannon orientation for {cannon_key}: {e}")
                
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
        """
        # Skip if we're shutting down
        if self.is_shutting_down:
            return
            
        # Clear previous threads list
        self.detection_threads = []
        
        # Create and start threads for all alive robots
        alive_robots = [ns for ns, robot in self.robots.items() if robot["health"] > 0]
        for ns in alive_robots:
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
        """
        detections = []
        if namespace not in self.robots or self.robots[namespace]["health"] <= 0:
            return detections

        # Get the robot information
        robot = self.robots[namespace]
        position = robot["position"]
        orientation = robot["orientation"]
        
        # Validate position and orientation
        if (position["x"] is None or position["y"] is None or position["z"] is None or
            orientation["x"] is None or orientation["y"] is None or 
            orientation["z"] is None or orientation["w"] is None):
            return detections

        # Convert to tuple format for get_relative_position_with_heading
        position_tuple = (position["x"], position["y"], position["z"])
        orientation_tuple = (orientation["x"], orientation["y"], orientation["z"], orientation["w"])

        # Determine detection range based on the type of the detecting robot
        is_drone = robot["type"] == "drone"
        if is_drone:
            detection_range = self.drone_detection_range
        else:
            detection_range_ship2ship = self.ship2ship_detection_range
            detection_range_ship2drone = self.ship2drone_detection_range

        # Check all other robots
        for other_ns, distance in robot["distances"].items():
            if other_ns not in self.robots or self.robots[other_ns]["health"] <= 0:
                continue
                
            other_robot = self.robots[other_ns]
            other_position = other_robot["position"]
            
            # Validate other robot's position
            if other_position["x"] is None or other_position["y"] is None or other_position["z"] is None:
                continue

            other_is_drone = other_robot["type"] == "drone"
            
            # Determine detection range based on both robots' types
            if is_drone:  # Detecting robot is a drone
                if distance > detection_range:
                    continue
            else:  # Detecting robot is a ship
                if other_is_drone:  # Target is a drone
                    if distance > detection_range_ship2drone:
                        continue
                else:  # Target is a ship
                    if distance > detection_range_ship2ship:
                        continue

            # Create detection
            detection = Detection()
            detection.vehicle_type = Detection.DRONE if other_is_drone else Detection.SHIP
            detection.is_friend = ((namespace in self.team_1 and other_ns in self.team_1) or 
                                  (namespace in self.team_2 and other_ns in self.team_2))

            # Get other robot's position as tuple
            other_position_tuple = (other_position["x"], other_position["y"], other_position["z"])
            
            # Get relative position in FRD frame
            relative_position = get_relative_position_with_heading(
                position_tuple,
                orientation_tuple,
                other_position_tuple
            )
            detection.relative_position.x = relative_position[0]   # Forward
            detection.relative_position.y = relative_position[1]   # Right 
            detection.relative_position.z = relative_position[2]   # Down

            detections.append(detection)

        return detections

    def _communication_callback(self, msg, sender_ns):
        """
        Handle incoming communication messages.
        """
        # Check if sender exists and is alive
        if sender_ns not in self.robots or self.robots[sender_ns]["health"] <= 0:
            self.get_logger().warn(f'No data for sender {sender_ns} or sender is not alive')
            return
            
        sender = self.robots[sender_ns]
        pos = sender["position"]
        
        # Validate sender position
        if pos["x"] is None or pos["y"] is None or pos["z"] is None:
            self.get_logger().warn(f'Invalid position for sender {sender_ns}')
            return

        # Determine communication range based on sender type
        communication_range = self.drone_communication_range if sender["type"] == "drone" else self.ship_communication_range
        
        # Forward the message to alive robots within range
        for ns, robot in self.robots.items():
            if ns == sender_ns or robot["health"] <= 0:
                continue
                
            # Use the pre-calculated distances
            if ns in sender["distances"]:
                distance = sender["distances"][ns]
                
                if distance <= communication_range:
                    string_msg = String()
                    string_msg.data = msg.data
                    self.communication_publishers[ns].publish(string_msg)

    ##########################################
    # WEAPON SYSTEMS (MISSILES AND KAMIKAZE) #
    ##########################################

    def _fire_missile_callback(self, request, response):
        """Handle missile firing requests."""
        shooter_ns = request.robot_name
        self.get_logger().info(f'Received missile fire request from {shooter_ns}')

        # Validate shooter exists and is alive
        if shooter_ns not in self.robots or self.robots[shooter_ns]["health"] <= 0:
            self.get_logger().warn(f'{shooter_ns} not found or not alive. Cannot fire missile.')
            response.has_fired = False
            response.ammo = 0
            return response
        
        shooter = self.robots[shooter_ns]
        is_drone = shooter["type"] == "drone"
        
        # Check ammunition
        if shooter["ammo"] <= 0:
            self.get_logger().warn(f'{shooter_ns} cannot fire: Out of ammunition.')
            response.has_fired = False
            response.ammo = 0
            return response
        
        # Check valid position
        pos = shooter["position"]
        ori = shooter["orientation"]
        if (pos["x"] is None or pos["y"] is None or pos["z"] is None or
            ori["x"] is None or ori["y"] is None or ori["z"] is None or ori["w"] is None):
            self.get_logger().warn(f"Invalid pose for '{shooter_ns}'. Cannot fire missile.")
            response.has_fired = False
            response.ammo = shooter["ammo"]
            return response
        
        # Check cooldown
        current_time = self.get_clock().now()
        cooldown = self.drone_cooldown if is_drone else self.ship_cooldown
        time_since_last_fire = (current_time - shooter["last_fire_time"]).nanoseconds / 1e9
        
        if time_since_last_fire < cooldown:
            time_remaining = cooldown - time_since_last_fire
            self.get_logger().warn(f'{shooter_ns} cannot fire: Cooldown ({time_remaining:.2f}s remaining).')
            response.has_fired = False
            response.ammo = shooter["ammo"]
            return response
        
        # Get shooter position and orientation
        if shooter["type"] == "ship":
            # Use the global cannon pose for ships
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
            shooter_position = (pos["x"], pos["y"], pos["z"])
            shooter_orientation = (ori["x"], ori["y"], ori["z"], ori["w"])
        
        # Find targets within missile range
        missile_range = self.drone_missile_range if is_drone else self.ship_missile_range
        targets_in_range = []
        
        for target_ns, distance in shooter["distances"].items():
            if distance <= missile_range and target_ns in self.robots:
                target = self.robots[target_ns]
                if target["health"] <= 0:
                    continue
                    
                target_position = (
                    target["position"]["x"],
                    target["position"]["y"],
                    target["position"]["z"]
                )
                if None not in target_position:
                    targets_in_range.append((target_ns, distance, target_position))
        
        # Update ammunition and fire time
        with self.robot_lock:
            shooter["ammo"] -= 1
            shooter["last_fire_time"] = current_time
        
        response.has_fired = True
        response.ammo = shooter["ammo"]
        
        # Check alignment and apply damage
        aligned_targets = self._find_aligned_targets(
            shooter_position, 
            shooter_orientation, 
            targets_in_range, 
            missile_range
        )
        
        if aligned_targets:
            closest_target = min(aligned_targets, key=lambda x: x[1])
            target_ns = closest_target[0]
            distance = closest_target[1]
            damage = self.drone_missile_damage if is_drone else self.ship_missile_damage
            self._update_health(target_ns, damage=damage)
            self.get_logger().info(f'Missile hit: {target_ns} at distance {self._format_float(distance)}m')
        else:
            self.get_logger().info(f'MISSILE MISSED: No aligned targets for {shooter_ns}')
        
        return response

    def _kamikaze_callback(self, request, response):
        """
        Handle the kamikaze service request.
        """
        kamikaze_ns = request.robot_name
        self.get_logger().info(f'Received kamikaze request from {kamikaze_ns}')

        # Ensure only drones can use the kamikaze service and drone is alive
        if kamikaze_ns not in self.robots:
            self.get_logger().warn(f'{kamikaze_ns} not found. Cannot detonate.')
            return response
            
        robot = self.robots[kamikaze_ns]
        
        # Don't check health here - we want kamikazes to work during destruction process
        
        if robot["type"] == "ship":
            self.get_logger().warn(f"Kamikaze service is not available for flagships: {kamikaze_ns}")
            return response

        # Check valid position
        pos = robot["position"]
        if pos["x"] is None or pos["y"] is None or pos["z"] is None:
            self.get_logger().warn(f"Invalid position data for {kamikaze_ns}")
            return response

        # Format position for logging with only 2 decimal places
        formatted_pos = self._format_position(pos)
        self.get_logger().info(f'💥 KAMIKAZE: {kamikaze_ns} detonating at position {formatted_pos}')

        # Save nearby targets before applying damage to kamikaze drone
        nearby_targets = []
        with self.robot_lock:
            for target_ns, distance in robot["distances"].items():
                # Only include living robots within explosion range
                if (distance <= self.explosion_range and 
                    target_ns != kamikaze_ns and 
                    target_ns in self.robots and 
                    self.robots[target_ns]["health"] > 0):
                    nearby_targets.append((target_ns, distance))
            
            # Ensure the kamikaze drone is fully dead to prevent recursive calls
            robot["health"] = 0
        
        # Log the targets in explosion range
        if nearby_targets:
            formatted_targets = [(ns, self._format_float(dist)) for ns, dist in nearby_targets]
            self.get_logger().info(f'Robots within explosion range: {formatted_targets}')
        
        # Remove the drone from simulation first
        model_id = robot["model_id"]
        model_name = robot["model_name"]
        if model_id is not None and model_name is not None:
            self._kill_drone(kamikaze_ns, model_id, model_name)
        
        # Apply explosion damage to nearby robots
        for target_ns, distance in nearby_targets:
            self.get_logger().info(f'Applying {self.explosion_damage} damage to {target_ns} at distance {self._format_float(distance)}m')
            
            # Double-check that the target is still alive before applying damage
            # (in case another explosion has already destroyed it)
            with self.robot_lock:
                if target_ns in self.robots and self.robots[target_ns]["health"] > 0:
                    # Release the lock before calling _update_health to avoid deadlocks
                    target_is_alive = True
                else:
                    target_is_alive = False
                    
            # Only apply damage if target is still alive
            if target_is_alive:
                self._update_health(target_ns, damage=self.explosion_damage)

        return response

    def _find_aligned_targets(self, shooter_position, shooter_orientation, targets_in_range, missile_range):
        """
        Filter targets that are aligned with the shooter's orientation.
        """
        aligned_targets = []
        for target in targets_in_range:
            target_id = target[0]
            target_position = target[2]

            # Determine target padding based on type
            if self.robots[target_id]["type"] == "drone":
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

    def _update_health(self, ns, health=None, damage=0):
        """
        Update the health of a robot.
        
        Args:
            ns (str): The namespace of the robot
            health (int, optional): New health value to set
            damage (int, optional): Amount of damage to apply
        """
        # Validate namespace exists
        if ns not in self.robots:
            self.get_logger().warn(f"Robot {ns} not found, cannot update health")
            return
                
        # Skip if robot is already dead
        with self.robot_lock:
            if self.robots[ns]["health"] <= 0:
                return
                
            # Calculate new health if damage was specified
            current_health = self.robots[ns]["health"]
            if damage > 0:
                health = max(0, current_health - damage)
            
            if health is None:
                return
                
            # Update health in the robot dictionary
            old_health = current_health
            self.robots[ns]["health"] = health
            
            # Publish health to topics (inside the lock to prevent race conditions)
            health_msg = Int32()
            health_msg.data = health
            if ns in self.health_publishers:
                self.health_publishers[ns].publish(health_msg)
        
        # Log outside of the lock
        self.get_logger().info(f"Updated health for {ns}: {health}")
        
        # Handle robot destruction if health reaches 0
        if health <= 0 and old_health > 0:
            if self.robots[ns]["type"] == "drone":
                self._handle_drone_destruction(ns)
            else:
                self._handle_ship_destruction(ns)
                    
        return health

    def _handle_drone_destruction(self, ns, from_kamikaze=False):
        """Handle a drone being destroyed"""
        self.get_logger().info(f'### DESTROYED: {ns} is eliminated ###')
        
        # We don't need the from_kamikaze check anymore since we only trigger kamikaze directly
        # from damage (not recursively from other kamikazes)
        
        # Create a request for the kamikaze service
        request = Kamikaze.Request()
        request.robot_name = ns
        # Process it directly rather than using a client
        self._kamikaze_callback(request, Kamikaze.Response())

    def _handle_ship_destruction(self, ns):
        """Handle a ship being destroyed"""
        self.get_logger().info(f'### FLAGSHIP DESTROYED: {ns} ###')
        
        # End the game when a flagship is destroyed
        self._end_game()

    def _publish_health_status(self):
        """
        Publish health status for all robots periodically.
        """
        health_status = {}
        for ns, robot in self.robots.items():
            health_status[ns] = robot["health"]
            health_msg = Int32()
            health_msg.data = robot["health"]
            self.health_publishers[ns].publish(health_msg)
            
        if health_status:
            self.get_logger().info(f'Robot health: {health_status}')

    def _kill_drone(self, namespace, model_id, model_name):
        """
        Remove a drone from the simulation when it's destroyed.
        
        Args:
            namespace (str): The namespace of the drone to remove
            model_id: The Gazebo model ID
            model_name: The Gazebo model name
        """
        if model_id is None or model_name is None:
            self.get_logger().warn(f"No model data for {namespace}, cannot kill drone")
            return
        
        # Extract the base name pattern (remove instance number)
        base_name_parts = model_name.rsplit('_', 1)[0]
        
        # Call the kill_drone_from_game_master function with non-blocking mode
        success = kill_drone_from_game_master(
            namespace=namespace,
            drone_model_base_name=base_name_parts,
            drone_models={namespace: model_id},
            logger=self.get_logger(),
            world_name=self.gazebo_world_name,
            gz_node=self.gz_node,
            non_blocking=True
        )
        
        if not success:
            self.get_logger().error(f"Failed to initiate kill process for {namespace}")

    ###############################
    # GAME TIMING AND TERMINATION #
    ###############################

    def _game_timer_callback(self):
        """
        Monitor game progress and check for end conditions.
        """
        if self.start_time.seconds_nanoseconds()[0] == 0:
            self.start_time = self.get_clock().now()
            self.get_logger().info("Waiting for clock synchronization...")
            time.sleep(1)
            return

        current_time = self.get_clock().now()
        # Calculate elapsed time
        elapsed_duration = current_time - self.start_time
        elapsed_seconds = elapsed_duration.nanoseconds / 1e9      
        self.remaining_time = max(0, int(self.game_duration - elapsed_seconds))
        
        # Publish remaining time
        time_msg = Int32()
        time_msg.data = self.remaining_time
        self.time_publisher.publish(time_msg)
        
        # End game if time is up
        if elapsed_seconds > 1.0 and elapsed_seconds >= self.game_duration:
            self.get_logger().info(f"Game duration reached: {self._format_float(elapsed_seconds)} >= {self.game_duration}")
            self._end_game()
        
        # Add periodic progress updates
        if int(elapsed_seconds) % 60 == 0 and int(elapsed_seconds) > 0:
            self.get_logger().info(f"GAME STATUS: {int(elapsed_seconds)}/{self.game_duration} seconds elapsed, {self.remaining_time} seconds remaining")
            
            # Calculate current flagship health
            team1_ships = [ns for ns in self.team_1 if "/flag_ship_" in ns and self.robots[ns]["health"] > 0]
            team2_ships = [ns for ns in self.team_2 if "/flag_ship_" in ns and self.robots[ns]["health"] > 0]
            team1_ship_health = sum(self.robots[ns]["health"] for ns in team1_ships)
            team2_ship_health = sum(self.robots[ns]["health"] for ns in team2_ships)
            
            # Count surviving drones
            team1_surviving_drones = sum(1 for ns in self.team_1 if "/px4_" in ns and self.robots[ns]["health"] > 0)
            team2_surviving_drones = sum(1 for ns in self.team_2 if "/px4_" in ns and self.robots[ns]["health"] > 0)
            
            self.get_logger().info(f"STATUS: Team 1 ship health: {team1_ship_health}, drones: {team1_surviving_drones}")
            self.get_logger().info(f"STATUS: Team 2 ship health: {team2_ship_health}, drones: {team2_surviving_drones}")

    def _end_game(self):
        """
        Handle game termination and results logging.
        """
        self.get_logger().info('Game Over')
        
        # Calculate team status
        team1_ships = [ns for ns in self.team_1 if "/flag_ship_" in ns and self.robots[ns]["health"] > 0]
        team2_ships = [ns for ns in self.team_2 if "/flag_ship_" in ns and self.robots[ns]["health"] > 0]
        team1_ship_health = sum(self.robots[ns]["health"] for ns in team1_ships)
        team2_ship_health = sum(self.robots[ns]["health"] for ns in team2_ships)
        
        team1_surviving_drones = sum(1 for ns in self.team_1 if "/px4_" in ns and self.robots[ns]["health"] > 0)
        team2_surviving_drones = sum(1 for ns in self.team_2 if "/px4_" in ns and self.robots[ns]["health"] > 0)
        
        # Determine the winner
        result = "Game Over\n"
        
        if team2_ship_health == 0 and team1_ship_health > 0:
            result += 'Team 1 wins! (Destroyed enemy flagship)\n'
            winner = "team_1"
        elif team1_ship_health == 0 and team2_ship_health > 0:
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
        all_team1_ships = [ns for ns in self.team_1 if "/flag_ship_" in ns] 
        team1_surviving_drones_names = [ns for ns in self.team_1 if "/px4_" in ns and self.robots[ns]["health"] > 0]
        team1_destroyed_drones_names = [ns for ns in self.team_1 if "/px4_" in ns and self.robots[ns]["health"] <= 0]

        # Get surviving and destroyed drones for Team 2
        all_team2_ships = [ns for ns in self.team_2 if "/flag_ship_" in ns]
        team2_surviving_drones_names = [ns for ns in self.team_2 if "/px4_" in ns and self.robots[ns]["health"] > 0]
        team2_destroyed_drones_names = [ns for ns in self.team_2 if "/px4_" in ns and self.robots[ns]["health"] <= 0]

        # Add detailed statistics
        result += f"Team 1 flagship health: {team1_ship_health}/{len(all_team1_ships) * self.ship_health}\n"
        result += f"Team 1 surviving drones: {', '.join(team1_surviving_drones_names) if team1_surviving_drones_names else 'None'}\n"
        result += f"Team 1 destroyed drones: {', '.join(team1_destroyed_drones_names) if team1_destroyed_drones_names else 'None'}\n"
        result += f"Team 2 flagship health: {team2_ship_health}/{len(all_team2_ships) * self.ship_health}\n"
        result += f"Team 2 surviving drones: {', '.join(team2_surviving_drones_names) if team2_surviving_drones_names else 'None'}\n"
        result += f"Team 2 destroyed drones: {', '.join(team2_destroyed_drones_names) if team2_destroyed_drones_names else 'None'}\n"
        result += f"Winner: {winner}\n"

        self._save_game_results(result)
        
        self.get_logger().info("Results written successfully. Shutting down.")
        # Create a one-shot timer that will trigger shutdown after a short delay
        self.shutdown_timer = self.create_timer(1.0, self._delayed_shutdown, callback_group=rclpy.callback_groups.MutuallyExclusiveCallbackGroup())

    def _save_game_results(self, result):
        """Save game results to files."""
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
        threading.Thread(target=self._actual_shutdown).start()

    def _actual_shutdown(self):
        """Actually shut down ROS after a small delay"""
        time.sleep(0.5)  # Small delay to allow final messages to be logged
        self.get_logger().info("Terminating ROS...")
        rclpy.shutdown()
        
def main(args=None):
    rclpy.init(args=args)
    game_master_node = GameMasterNode()
    
    # Use atexit to ensure proper cleanup
    import atexit
    atexit.register(lambda: rclpy.shutdown() if rclpy.ok() else None)
    
    executor = rclpy.executors.MultiThreadedExecutor()
    try:
        executor.add_node(game_master_node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        game_master_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()