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

The game involves two teams competing against each other, with both drones and ships
having different capabilities (health, detection range, communication range).
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
from swarmz_interfaces.msg import Detections, Detection
from utils.tools import get_all_namespaces, get_distance, get_relative_position, get_relative_position_with_orientation
# from utils.kill_drone import kill_drone_processes, get_model_id, remove_model
from utils.gazebo_subscriber import GazeboPosesTracker
from swarmz_interfaces.srv import UpdateHealth
import time
import threading
import os
from datetime import datetime
import subprocess
from gz.msgs10.scene_pb2 import Scene
from gz.msgs10.empty_pb2 import Empty
from gz.msgs10.entity_pb2 import Entity
from gz.msgs10.boolean_pb2 import Boolean
from gz.transport13 import Node as GzNode


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
    """

    def __init__(self):
        """Initialize the Game Master node and set up all necessary components."""
        super().__init__('game_master_node')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])

        # Declare parameters with default values
        self.declare_parameter('drone_detection_range', 10.0)
        self.declare_parameter('ship_detection_range', 20.0)
        self.declare_parameter('drone_communication_range', 15.0)
        self.declare_parameter('ship_communication_range', 30.0)
        self.declare_parameter('drone_health', 100)
        self.declare_parameter('ship_health', 200)
        self.declare_parameter('drone_points', 10)
        self.declare_parameter('ship_points', 50)
        self.declare_parameter('game_duration', 300)  # 5 minutes

        self.drone_detection_range = self.get_parameter('drone_detection_range').get_parameter_value().double_value
        self.ship_detection_range = self.get_parameter('ship_detection_range').get_parameter_value().double_value
        self.drone_communication_range = self.get_parameter('drone_communication_range').get_parameter_value().double_value
        self.ship_communication_range = self.get_parameter('ship_communication_range').get_parameter_value().double_value
        self.drone_health = self.get_parameter('drone_health').get_parameter_value().integer_value
        self.ship_health = self.get_parameter('ship_health').get_parameter_value().integer_value
        self.drone_points = self.get_parameter('drone_points').get_parameter_value().integer_value
        self.ship_points = self.get_parameter('ship_points').get_parameter_value().integer_value
        self.game_duration = self.get_parameter('game_duration').get_parameter_value().integer_value

        # Get list of all namespaces
        self.namespaces = get_all_namespaces(self)
        # Wait until namespaces are detected
        while not self.namespaces:
            self.get_logger().warn("No valid namespaces detected. Waiting...")
            time.sleep(1)
            self.namespaces = get_all_namespaces(self)

        # Get Gazebo model IDs of the drones
        self.drone_model_ids = self.get_model_ids(self.namespaces)  # Returns: {"/px4_1": ID1, "/px4_2": ID2, "/px4_3": ID3}


        # Print the list of detected robots
        self.get_logger().info(f"Detected robots: {self.namespaces}")

        # Automatically compose teams
        drones = sorted([ns for ns in self.namespaces if '/px4_' in ns])
        ships = sorted([ns for ns in self.namespaces if '/flag_ship_' in ns])
        
        # Calculate team sizes (handling uneven numbers)
        drones_team_1 = (len(drones) + 1) // 2  # Larger team gets the extra drone if odd
        ships_team_1 = (len(ships) + 1) // 2    # Larger team gets the extra ship if odd
        
        # Split vehicles into teams
        self.team_1 = drones[:drones_team_1] + ships[:ships_team_1]
        self.team_2 = drones[drones_team_1:] + ships[ships_team_1:]
        
        # Warn about uneven teams
        if len(drones) % 2 != 0 or len(ships) % 2 != 0:
            warning = "\n" + "!" * 80 + "\n"
            warning += "WARNING: UNEVEN TEAM COMPOSITION DETECTED\n"
            warning += f"Team 1 has {len(self.team_1)} vehicles ({len(drones[:drones_team_1])} drones, {len(ships[:ships_team_1])} ships)\n"
            warning += f"Team 2 has {len(self.team_2)} vehicles ({len(drones[drones_team_1:])} drones, {len(ships[ships_team_1:])} ships)\n"
            warning += "!" * 80
            self.get_logger().warn(warning)
        
        self.get_logger().info(f"Team 1: {self.team_1}")
        self.get_logger().info(f"Team 2: {self.team_2}")

        # Initialize health points for each robot
        self.health_points = {ns: self.drone_health if 'px4_' in ns else self.ship_health for ns in self.namespaces}
        # Ensure flag_ship_1 and flag_ship_2 are in health_points
        self.health_points.setdefault('flag_ship_1', self.ship_health)
        self.health_points.setdefault('flag_ship_2', self.ship_health)
        
        # Ensure all team members are in health_points
        for ns in self.team_1 + self.team_2:
            self.health_points.setdefault(ns, self.drone_health if 'px4_' in ns else self.ship_health)

        # Initialize team points
        self.team_points = {'team_1': 0, 'team_2': 0}

        # Publishers fohealth_publishersr health points, detections, and communications
        self.health_publishers = {ns: self.create_publisher(Int32, f'{ns}/health', 10) for ns in self.namespaces}
        self.detection_publishers = {ns: self.create_publisher(Detections, f'{ns}/detections', 10) for ns in self.namespaces}
        self.communication_publishers = {ns: self.create_publisher(String, f'{ns}/out_going_messages', 10) for ns in self.namespaces}

        # Subscribers for communications
        self.communication_subscribers = {ns: self.create_subscription(String, f'{ns}/incoming_messages', lambda msg, ns=ns: self.communication_callback(msg, ns), 10) for ns in self.namespaces}

        # Timer to periodically update robot positions
        self.robot_poses = {}  # Combined position and orientation
        self.gz = GazeboPosesTracker(self.namespaces)
        self.update_positions_timer = self.create_timer(0.1, self.update_positions)

        # Timer to periodically publish detections
        self.timer = self.create_timer(1.0, self.detections_callback)

        # Timer to track game duration
        self.start_time = time.time()
        self.game_timer = self.create_timer(1.0, self.game_timer_callback)
        
        # Add time publisher
        self.time_publisher = self.create_publisher(Int32, '/game_master/time', 10)

        # Timer to periodically publish health status (changed from 2.0 to 20.0 seconds)
        self.health_timer = self.create_timer(20.0, self.publish_health_status)

        # Create the service to update health
        self.update_health_srv = self.create_service(UpdateHealth, 'update_health', self.update_health_callback)

        # Add debug information about subscribers and publishers
        self.get_logger().info("Setting up communication channels...")
        for ns in self.namespaces:
            self.get_logger().info(f"Created publisher for {ns}/out_going_messages")
            self.get_logger().info(f"Created subscriber for {ns}/incoming_messages")

    def update_positions(self):
        """
        Update the position and orientation of all robots in the simulation.
        Gets the latest pose data from Gazebo for each robot and stores it
        in self.robot_poses in a standardized format.
        """
        for ns in self.namespaces:
            pose = self.gz.get_pose(ns)
            # Store poses in a consistent format
            self.robot_poses[ns] = {
                'position': (pose['position']['x'], 
                           pose['position']['y'], 
                           pose['position']['z']),
                'orientation': (pose['orientation']['x'],
                              pose['orientation']['y'],
                              pose['orientation']['z'],
                              pose['orientation']['w'])
            }

    def game_timer_callback(self):
        """
        Monitor game progress and check for end conditions.
        
        Tracks:
        - Remaining game time
        - Flagship destruction
        - Game termination conditions
        
        Publishes the remaining time to /game_master/time topic.
        """
        elapsed_time = time.time() - self.start_time
        remaining_time = max(0, int(self.game_duration - elapsed_time))
        
        # Publish remaining time
        time_msg = Int32()
        time_msg.data = remaining_time
        self.time_publisher.publish(time_msg)
        
        if elapsed_time >= self.game_duration:
            self.end_game()
        else:
            for ship in ['flag_ship_1', 'flag_ship_2']:
                if self.health_points.get(ship, 0) <= 0:
                    self.end_game()

    def end_game(self):
        """
        Handle game termination and results logging.
        
        - Determines the winning team
        - Generates detailed game results
        - Saves results to:
            - game_results.txt (cumulative results)
            - individual_games/game_results_TIMESTAMP.txt (individual game results)
        """
        self.get_logger().info('Game Over')
        result = "Game Over\n"
        if self.team_points['team_1'] > self.team_points['team_2']:
            result += 'Team 1 wins!\n'
        elif self.team_points['team_1'] < self.team_points['team_2']:
            result += 'Team 2 wins!\n'
        else:
            result += 'It\'s a draw!\n'

        result += f"Team 1 points: {self.team_points['team_1']}\n"
        result += "Team 1 alive robots: " + ", ".join([ns for ns in self.team_1 if self.health_points.get(ns, 0) > 0]) + "\n"
        result += f"Team 2 points: {self.team_points['team_2']}\n"
        result += "Team 2 alive robots: " + ", ".join([ns for ns in self.team_2 if self.health_points.get(ns, 0) > 0]) + "\n"

        # Locate the SWARMz4 directory
        home_dir = os.path.expanduser("~")
        swarmz4_path = None
        for root, dirs, files in os.walk(home_dir):
            if 'SWARMz4' in dirs:
                swarmz4_path = os.path.join(root, 'SWARMz4')
                break

        if not swarmz4_path:
            self.get_logger().error("SWARMz4 directory not found! Logging results instead.")
            self.get_logger().info(result)
            rclpy.shutdown()
            return

        # Write to game_results.txt
        result_dir = os.path.join(swarmz4_path, 'game_results')
        result_file = os.path.join(result_dir, 'game_results.txt')
        # self.get_logger().info(f"Results directory: {result_dir}")
        # self.get_logger().info(f"Results file: {result_file}")
        
        # Ensure the directory exists
        if not os.path.exists(result_dir):
            # self.get_logger().info(f"Creating results directory at: {result_dir}")
            os.makedirs(result_dir)
        else:
            # self.get_logger().info(f"Results directory already exists at: {result_dir}")
            pass
        # self.get_logger().info(f"Writing results to file: {result_file}")

        game_number = 1
        if os.path.exists(result_file):
            self.get_logger().info(f"Result file exists. Reading current content.")
            with open(result_file, 'r') as file:
                content = file.read()
                game_number = content.count("--- Game") + 1
            with open(result_file, 'a') as file:
                self.get_logger().info(f"Appending results for Game {game_number}.")
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
            # self.get_logger().info(f"Creating individual games directory at: {individual_games_dir}")
            os.makedirs(individual_games_dir)
        else:
            # self.get_logger().info(f"Individual games directory already exists at: {individual_games_dir}")
            pass

        # Create a unique filename with a timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        individual_result_file = os.path.join(individual_games_dir, f'game_results_{timestamp}.txt')

        with open(individual_result_file, 'w') as file:
            file.write(result)

        self.get_logger().info("Results written successfully. Shutting down.")
        rclpy.shutdown()

    def detections_callback(self):
        """
        Manage the periodic publication of detection information.
        Uses multi-threading to efficiently process and publish
        detection data for all robots simultaneously.
        """
        # Check if we have valid pose data
        if not self.robot_poses or None in self.robot_poses[next(iter(self.robot_poses))]['position']:
            return
            
        threads = []
        for ns in self.namespaces:
            thread = threading.Thread(target=self.publish_detections, args=(ns,))
            threads.append(thread)
            thread.start()

        for thread in threads:
            thread.join()

    def publish_detections(self, ns):
        """
        Publish detections for a specific robot.
        :param ns: The namespace of the robot.
        """
        detections_msg = Detections()
        detections_msg.header.stamp = self.get_clock().now().to_msg()
        detections_msg.detections = self.get_detections(ns)
        self.detection_publishers[ns].publish(detections_msg)

    def is_friend(self, ns1, ns2):
        """
        Determine if two robots are friends.
        :param ns1: Namespace of the first robot.
        :param ns2: Namespace of the second robot.
        :return: True if they are friends, False otherwise.
        """
        return (ns1 in self.team_1 and ns2 in self.team_1) or (ns1 in self.team_2 and ns2 in self.team_2)
    
    def get_detections(self, namespace):
        """
        Calculate which robots are within detection range of the specified robot.

        Args:
            namespace (str): The namespace of the detecting robot

        Returns:
            list[Detection]: List of Detection messages containing:
                - Vehicle type (drone/ship)
                - Friend/foe status
                - Relative position in FRD (Forward-Right-Down) coordinates
        """
        detections = []
        if namespace not in self.robot_poses:
            return detections
            
        transmitter_pose = self.robot_poses[namespace]
        # Validate transmitter pose
        if None in transmitter_pose['position'] or None in transmitter_pose['orientation']:
            return detections
            
        detection_range = self.drone_detection_range if '/px4_' in namespace else self.ship_detection_range
        
        for robot, receiver_pose in self.robot_poses.items():
            if robot == namespace:
                continue
                
            # Validate receiver pose
            if None in receiver_pose['position'] or None in receiver_pose['orientation']:
                continue
                
            try:
                distance = get_distance(transmitter_pose['position'], receiver_pose['position'])
                if distance <= detection_range:
                    detection = Detection()
                    detection.vehicle_type = Detection.DRONE if '/px4_' in robot else Detection.SHIP
                    detection.is_friend = self.is_friend(namespace, robot)
                    
                    # Get relative position in NED frame
                    relative_position = get_relative_position_with_orientation(
                        transmitter_pose['position'], 
                        transmitter_pose['orientation'],
                        receiver_pose['position']
                    )
                    
                    detection.relative_position.x = relative_position[0]   # Forward
                    detection.relative_position.y = -relative_position[1]   # Right
                    detection.relative_position.z = -relative_position[2]   # Down
                    
                    detections.append(detection)
            except (TypeError, ValueError) as e:
                self.get_logger().warn(f'Error calculating detection between {namespace} and {robot}: {e}')
                continue
        
        return detections

    def communication_callback(self, msg, sender_ns):
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

    def get_health(self, ns):
        """
        Get the health of the specified robot.
        :param ns: The namespace of the robot.
        :return: The health of the robot.
        """
        return self.health_points.get(ns, 0)

    def update_health_callback(self, request, response):
        """
        Handle the update health service request.
        :param request: The service request containing the robot namespace and damage.
        :param response: The service response.
        :return: The updated response.
        """
        ns = request.robot_name
        damage = request.damage
        self.update_health(ns, damage=damage)
        self.get_logger().info(f'Health of {ns} updated by {damage} points')
        return response

    def update_health(self, ns, health=None, damage=0):
        """
        Update the health of the specified robot by setting a new health value or reducing it by a given damage amount.
        :param ns: The namespace of the robot.
        :param health: The health value to set (optional).
        :param damage: The amount of damage to apply (optional).
        """
        if ns not in self.namespaces:

            self.get_logger().warn(f'{ns} not found in robot list : {self.namespaces}. Cannot update health.')
            return
        if damage > 0:
            current_health = self.get_health(ns)
            health = max(0, current_health - damage)
        
        if health is not None:
            self.health_points[ns] = health
            self.get_logger().info(f'{ns} health is now {health}')
            health_msg = Int32()
            health_msg.data = health
            self.health_publishers[ns].publish(health_msg)
            if health == 0:
                self.update_team_points(ns)
                if 'px4_' in ns:
                    self.get_logger().info(f'{ns} is destroyed, awarding {self.drone_points} points')
                    # instance_number = int(ns.split('_')[1])
                    self.kill_drone(ns)
                    if ns in self.namespaces:
                        # Remove the destroyed drone from the list of namespaces
                        self.namespaces.remove(ns)
                        # Update the GazeboPosesTracker with the new list of namespaces
                        self.gz = GazeboPosesTracker(self.namespaces)             
                elif 'flag_ship_' in ns:
                    self.get_logger().info(f'{ns} is destroyed, awarding {self.ship_points} points')
                    self.get_logger().info(f'{ns} is destroyed, ending game')
                    self.end_game()
        return

    def get_model_ids(self, robot_list):
        """
        Find the IDs of multiple models by their names using the Gazebo Transport API.
        :param robot_list: A list of robot namespace names (e.g., ["/px4_1", "/px4_2"]).
        :return: A dictionary mapping each robot namespace to its model ID, or None if not found.
        """
        node = GzNode()
        scene_info = Scene()

        # Request scene info from Gazebo
        self.get_logger().info("Checking scene info for robots.")
        result, response = node.request("/world/default/scene/info", Empty(), Empty, Scene, 1000)
        if not result:
            self.get_logger().info("Failed to retrieve scene info from Gazebo.")
            return {}
        scene_info = response

        # Create a mapping of robot namespaces to expected model names
        namespace_to_model = {
            robot_name: f"x500_lidar_front_{robot_name.split('_')[-1]}" for robot_name in robot_list
        }
        model_ids = {}

        # Search for matching models in the scene
        for model in scene_info.model:
            if model.name in namespace_to_model.values():
                # Extract instance number from model name
                instance_number = model.name.split("_")[-1]

                # Find the corresponding namespace key
                robot_name = f"/px4_{instance_number}"
                model_ids[robot_name] = model.id
                self.get_logger().info(f"Found model '{model.name}' (Namespace: '{robot_name}') with ID: {model.id}")

        # Log missing robots
        for robot_name, model_name in namespace_to_model.items():
            if robot_name not in model_ids:
                self.get_logger().info(f"Model '{model_name}' (Namespace: '{robot_name}') not found.")

        return model_ids

    def remove_model(self, model_id):
        """
        Remove the model using its ID via Gazebo Transport API.
        
        :param model_id: The ID of the model to be removed.
        """
        node = GzNode()
        entity_msg = Entity()
        entity_msg.id = model_id
        entity_msg.type = Entity.MODEL  # Type 1 for models

        # Request to remove the model from Gazebo
        result, response = node.request("/world/default/remove", entity_msg, Entity, Boolean, 1000)
        if result and response.data:
            self.get_logger().info(f"Successfully requested removal of model ID {model_id}")
        else:
            self.get_logger().info(f"Failed to request removal of model ID {model_id}")

    def kill_drone(self, namespace):
        """
        Kill the drone processes and remove the model from Gazebo.
        :param robotnamespace_name: The namespace of the drone.
        """
        # Remove the drone model from Gazebo
        model_id = self.drone_model_ids.get(namespace)
        self.get_logger().info(f'model_id: {model_id}') 
        if (model_id):
            self.remove_model(model_id)
            self.get_logger().info(f'removed model {namespace} with ID {model_id}')
        else:
            self.get_logger().warn(f'failed to find model ID of {namespace}')
        
        # Kill the drone processes
        self.kill_drone_processes(namespace)
        self.get_logger().info(f'killed drone processes for {namespace}')
        return

    def kill_drone_processes(self, namespace):
        """
        Kill the processes of all ROS 2 nodes running on the concerned px4_ instance.
        :param namespace: The namespace of the drone.
        """
        self.get_logger().info(f'Killing processes of all ROS 2 nodes in namespace {namespace}')
        try:
            # Kill ros2 processes in the namespace
            try:
                subprocess.run(
                    f'pgrep -f "\-r __ns:={namespace}" | xargs kill -9',
                    shell=True, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            except subprocess.CalledProcessError:
                pass  # Suppressing errors

            # Kill the px4 processes
            try:
                subprocess.run(
                    f'pkill -f "px4 -i {namespace.split("_")[-1]}"',
                    shell=True, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            except subprocess.CalledProcessError:
                pass  # Suppressing errors

            # Kill any other px4 related processes
            try:
                subprocess.run(
                    f'pkill -f "px4_{namespace.split("_")[-1]}"',
                    shell=True, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            except subprocess.CalledProcessError:
                pass  # Suppressing errors

        except subprocess.CalledProcessError:
            pass  # Suppressing any remaining errors

    def update_team_points(self, ns):
        """
        Update the team points based on the destruction of a robot.
        :param ns: The namespace of the destroyed robot.
        """
        if ns in self.team_1:
            self.team_points['team_2'] += self.drone_points if 'px4_' in ns else self.ship_points
        elif ns in self.team_2:
            self.team_points['team_1'] += self.drone_points if 'px4_' in ns else self.ship_points

    def publish_health_status(self):
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

def main(args=None):
    rclpy.init(args=args)
    game_master_node = GameMasterNode()
    try:
        rclpy.spin(game_master_node)
    except KeyboardInterrupt:
        pass
    finally:
        game_master_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
