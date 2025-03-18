"""
Game Master Dynamic Test Client

This module implements a demonstration client for the Game Master combat simulation system.
It showcases various features of the Game Master by orchestrating a choreographed demonstration
with multiple PX4 drones in a simulated environment.

Features demonstrated:
- Formation flying (arranging drones in a circle)
- Inter-drone messaging
- Detection visualization
- Missile firing
- Kamikaze attacks
- Game timing and completion

Prerequisites:
- ROS 2 environment with SWARMz4 packages installed
- PX4 simulation with 10 drones (namespaced from /px4_1 to /px4_10)
- Running game_master_node
- Available missile and kamikaze services

Usage:
    ros2 run game_master game_master_client_dynamic_test.py
    (in separate terminal of game master and missile and kamikaze services)

    ros2 launch game_master game_master_demo.launch.py
    (This will launch the game_master_node, missile_server, kamikaze server and the game_master_client_dynamic_test.py
    in one terminal, all the output will be displayed in the same terminal)
    
Demo Sequence:
1. Drones take off and arrange in a circle at the center of the field (~1 minute)
2. Drone /px4_1 sends a test message to demonstrate communication
3. Detection outputs are displayed from /px4_1 and /px4_6
4. Drones /px4_2 and /px4_9 fire missiles
5. Drone /px4_3 executes a kamikaze attack
6. Demo continues until game timeout

Note: The demo is timed to account for the approximately 1-minute drone takeoff and
positioning phase before the main feature demonstrations begin.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from px4_controllers_interfaces.action import GotoPosition
from swarmz_interfaces.srv import Kamikaze, Missile
from swarmz_interfaces.msg import Detections, Detection
from std_msgs.msg import String, Int32
from geometry_msgs.msg import Point
import math
import time


class DemoNode(Node):
    """
    Demonstration node for Game Master features
    """
    def __init__(self):
        super().__init__('game_master_demo')
        self.get_logger().info('Starting Game Master Demo')
        
        # Configuration
        self.field_x = 250.0  # Field size in X
        self.field_y = 500.0  # Field size in Y
        self.circle_radius = 10.0  # Circle radius for formation
        self.hover_height = 5.0  # Hover height in meters
        self.drones = [f'/px4_{i}' for i in range(1, 11)]  # 10 drones
        
        # Phase/timing control
        self.demo_start_time = self.get_clock().now()
        self.current_phase = 'init'
        self.phase_complete = {}
        self.detection_queues = {'/px4_1': [], '/px4_6': []}
        self.goal_completed = {drone: False for drone in self.drones}  # Track completed goals
        self.goals_sent = False  # Flag to track if goals have been sent
        
        # Create action clients for movement
        self.action_clients = {}
        self.goal_handles = {}
        for drone in self.drones:
            self.action_clients[drone] = ActionClient(
                self, 
                GotoPosition, 
                f'{drone}/goto_position'
            )
        
        # Set up missile and kamikaze service clients
        self.missile_client = self.create_client(Missile, 'fire_missile')
        self.kamikaze_client = self.create_client(Kamikaze, 'kamikaze')
        
        # Set up message publishers and subscribers
        self.message_publishers = {}
        self.received_messages = {drone: set() for drone in self.drones}
        
        for drone in self.drones:
            # Message publishers
            self.message_publishers[drone] = self.create_publisher(
                String,
                f'{drone}/incoming_messages',
                10
            )
            
            # Message subscribers
            self.create_subscription(
                String,
                f'{drone}/out_going_messages',
                lambda msg, d=drone: self.message_callback(msg, d),
                10
            )

        # Subscribe to specific drone detections
        self.detection_received = {'/px4_1': False, '/px4_6': False}
        # Add tracking for phase 4 detections
        self.phase4_detection_shown = {'/px4_1': False, '/px4_6': False}
        self.create_subscription(
            Detections, 
            '/px4_1/detections', 
            lambda msg: self.detection_callback(msg, '/px4_1'),
            10
        )
        self.create_subscription(
            Detections, 
            '/px4_6/detections', 
            lambda msg: self.detection_callback(msg, '/px4_6'),
            10
        )
        
        # Time remaining subscription
        self.create_subscription(
            Int32,
            '/game_master/time',
            self.time_callback,
            10
        )
        
        # Demo sequence timer (10Hz)
        self.demo_timer = self.create_timer(1.0, self.demo_sequence)
        self.get_logger().info('Demo node initialized')
    
    def demo_sequence(self):
        """Manage the demo sequence timeline"""
        elapsed_time = (self.get_clock().now() - self.demo_start_time).nanoseconds / 1e9
        
        # 1. Move drones to circle formation
        if self.current_phase == 'init' and elapsed_time > 10.0:
            self.get_logger().info('############################################')
            self.get_logger().info('### 1. Phase: Moving to circle formation ###')
            self.get_logger().info('############################################')
            self.current_phase = 'moving'
            self.move_to_circle_formation()
            self.goals_sent = True

        # 2. Check if all drones have reached positions            
        elif self.current_phase == 'moving':
            if self.goals_sent and self.check_all_goals_complete():
                self.get_logger().info('###########################################')
                self.get_logger().info('### 2. Phase: Circle formation complete ###')
                self.get_logger().info('###########################################')
                self.current_phase = 'formation_complete'

        # 3. Send test message from /px4_1 and listen from othrers
        elif self.current_phase == 'formation_complete' and elapsed_time > 90.0:
            self.get_logger().info('#######################################')
            self.get_logger().info('### 3. Phase: Message demonstration ###')
            self.get_logger().info('#######################################')
            self.current_phase = 'messaging'
            self.send_test_message()

        # 4. Process /px4_1 and /px4_6 detections   
        elif self.current_phase == 'messaging' and elapsed_time > 110.0:
            if not self.phase_complete.get('detections', False):
                self.get_logger().info('#########################################')
                self.get_logger().info('### 4. Phase: Detection demonstration ###')
                self.get_logger().info('#########################################')
                self.current_phase = 'detections'
                # Reset phase 4 detection flags
                self.phase4_detection_shown = {'/px4_1': False, '/px4_6': False}
                # Process any queued detections
                self.process_queued_detections()
                # Set a timer to remain in detection phase for a while
                self.detection_phase_start_time = elapsed_time
                # Detections are automatic via the game master node
                self.phase_complete['detections'] = True

        # 5. Fire missiles from /px4_2 and /px4_9        
        elif self.current_phase == 'detections' and elapsed_time > 130.0:
            self.get_logger().info('#######################################')
            self.get_logger().info('### 5. Phase: Missile demonstration ###')
            self.get_logger().info('#######################################')
            self.current_phase = 'missiles'
            self.fire_missiles()

        # 6. Execute kamiakaze attack with /px4_3    
        elif self.current_phase == 'missiles' and elapsed_time > 150.0:
            self.get_logger().info('########################################')
            self.get_logger().info('### 6. Phase: Kamikaze demonstration ###')
            self.get_logger().info('########################################')
            self.current_phase = 'kamikaze'
            self.execute_kamikaze()

        # 7. Wait for game timeout    
        elif self.current_phase == 'kamikaze' and elapsed_time > 170.0:
            if not self.phase_complete.get('waiting', False):
                self.get_logger().info('######################################')
                self.get_logger().info('### 7. Phase: Waiting for the end  ###')
                self.get_logger().info('######################################')
                self.current_phase = 'waiting'
                self.phase_complete['waiting'] = True

    def move_to_circle_formation(self):
        """
        Move all drones to form a circle at the center of the field
        """
        center_x = self.field_x / 2.0
        center_y = self.field_y / 2.0
        
        for i, drone in enumerate(self.drones):
            # Calculate position on circle
            angle = (i / len(self.drones)) * 2 * math.pi
            x = center_x + self.circle_radius * math.cos(angle)
            y = center_y + self.circle_radius * math.sin(angle)
            
            # Calculate yaw to face center
            # Converting from radians to degrees for NED frame where 0 degrees points north
            yaw = math.degrees(math.atan2(center_y - y, center_x - x))
            self.get_logger().info('################ DEBUG ###############')
            
            # Create goal message
            goal_msg = GotoPosition.Goal()
            goal_msg.target.position.x = float(x)
            goal_msg.target.position.y = float(y)
            goal_msg.target.position.z = -float(self.hover_height)
            goal_msg.target.yaw = float(yaw)
            
            # Send goal
            self.get_logger().info(f'Moving {drone} to position: x={x:.2f}, y={y:.2f}, z=-{self.hover_height}, yaw={yaw:.2f} degrees')
            self.send_goal(drone, goal_msg)

    def send_goal(self, drone, goal_msg):
        """
        Send a goal to a drone
        """
        # Wait for action server
        if not self.action_clients[drone].wait_for_server(timeout_sec=1.0):
            self.get_logger().error(f'Action server not available for {drone}')
            return
        
        # Send goal
        send_goal_future = self.action_clients[drone].send_goal_async(
            goal_msg,
            feedback_callback=lambda feedback: self.goal_feedback_callback(feedback, drone)
        )
        send_goal_future.add_done_callback(
            lambda future, d=drone: self.goal_response_callback(future, d)
        )

    def goal_response_callback(self, future, drone):
        """
        Handle goal acceptance/rejection
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(f'{drone}: Goal rejected')
            return

        self.goal_handles[drone] = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda future, d=drone: self.goal_result_callback(future, d)
        )

    def goal_result_callback(self, future, drone):
        """
        Handle goal completion
        """
        result = future.result().result
        if result.success:
            self.get_logger().info(f'{drone}: Goal succeeded')
            self.goal_completed[drone] = True
        else:
            self.get_logger().warn(f'{drone}: Goal failed')

    def goal_feedback_callback(self, feedback_msg, drone):
        """
        Handle goal feedback
        """
        # Only show feedback during the 'moving' phase
        if self.current_phase != 'moving':
            return
        
        # Reduce threshold for logging to prevent log flooding
        # Only log when almost at target (0.5m) instead of 2.0m
        if feedback_msg.feedback.distance_to_target < 0.5:
            # Add a counter to further reduce the frequency of logs
            if not hasattr(self, '_feedback_log_counter'):
                self._feedback_log_counter = {}
        
            # Only log every 5th message for each drone
            self._feedback_log_counter[drone] = self._feedback_log_counter.get(drone, 0) + 1
            if self._feedback_log_counter[drone] % 10 == 0:
                self.get_logger().info(f'{drone}: Close to target: {feedback_msg.feedback.distance_to_target:.2f}m')

    def check_all_goals_complete(self):
        """
        Check if all drones have completed their goals
        """
        # First ensure all drones have received goals
        if not all(drone in self.goal_handles for drone in self.drones):
            return False
            
        # Check if at least half of the drones have reached their positions
        # This is a compromise - waiting for all drones might take too long
        completed_count = sum(1 for drone in self.drones if self.goal_completed[drone])
        min_required = len(self.drones) // 2
        
        if completed_count >= min_required:
            self.get_logger().info(f'{completed_count}/{len(self.drones)} drones have reached target positions')
            return True
            
        return False

    def send_test_message(self):
        """
        Have px4_1 send a test message
        """
        self.get_logger().info('Sending test message from /px4_1')
        msg = String()
        msg.data = 'Hello from px4_1! This is a demo message.'
        self.message_publishers['/px4_1'].publish(msg)

    def message_callback(self, msg, sender):
        """
        Handle received messages
        """
        # Only log each message once per drone
        if msg.data not in self.received_messages[sender]:
            self.received_messages[sender].add(msg.data)
            self.get_logger().info(f'Message received by {sender}: {msg.data}')

    def detection_callback(self, msg, drone):
        """
        Handle detection information
        """
        # Only process detections during detection phase
        if self.current_phase == 'detections':
            # Only process if we haven't shown a detection for this drone yet
            if not self.phase4_detection_shown[drone]:
                self.process_detection(msg, drone)
        # Queue detections before phase 4 for potential use later
        elif self.current_phase in ['init', 'moving', 'formation_complete', 'messaging']:
            # Only keep the most recent detection
            if drone in self.detection_queues:
                self.detection_queues[drone] = [msg]  # Replace with newest
        # Ignore detections in later phases
    
    def process_queued_detections(self):
        """
        Process any detections that were received before the detections phase
        """
        for drone, queue in self.detection_queues.items():
            if queue and not self.phase4_detection_shown[drone]:
                self.process_detection(queue[0], drone)
        # Clear the queues
        self.detection_queues = {k: [] for k in self.detection_queues.keys()}
    
    def process_detection(self, msg, drone):
        """
        Display detection information
        """
        # Only show detection if we're in phase 4 and haven't shown it yet
        if self.current_phase == 'detections' and not self.phase4_detection_shown[drone]:
            # Mark that we've shown a detection for this drone in phase 4
            self.phase4_detection_shown[drone] = True
            
            self.get_logger().info(f'Detections from {drone}:')
            for i, detection in enumerate(msg.detections):
                self.get_logger().info(f'  Detection {i+1}:')
                vehicle_type = "Drone" if detection.vehicle_type == Detection.DRONE else "Ship"
                self.get_logger().info(f'    Type: {vehicle_type}')
                self.get_logger().info(f'    Friend: {detection.is_friend}')
                pos = detection.relative_position
                self.get_logger().info(f'    Position (FRD): x={pos.x:.2f}, y={pos.y:.2f}, z={pos.z:.2f}')
        # We don't show detections in any other phase

    def fire_missiles(self):
        """
        Have drones 2 and 9 fire missiles
        """
        for drone in ['/px4_1', '/px4_7']:
            self.get_logger().info(f'{drone} firing missile')
            if not self.missile_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().warn('Missile service not available')
                continue
                
            request = Missile.Request()
            request.robot_name = drone
            self.missile_client.call_async(request)

    def execute_kamikaze(self):
        """
        Have drone 3 execute kamikaze
        """
        self.get_logger().info('/px4_3 executing kamikaze')
        if not self.kamikaze_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Kamikaze service not available')
            return
            
        request = Kamikaze.Request()
        request.robot_name = '/px4_3'
        
        # For kamikaze, we anticipate the drone may be destroyed before response
        # so we'll log success on request sent, not response received
        self.get_logger().info('/px4_3 kamikaze request sent - drone will destroy itself')
        
        # Still make the async call, but we don't expect a response
        try:
            future = self.kamikaze_client.call_async(request)
            # Add callback but don't expect it to complete successfully
            future.add_done_callback(self.kamikaze_callback)
        except Exception as e:
            self.get_logger().error(f'Failed to send kamikaze request: {str(e)}')

    def kamikaze_callback(self, future):
        """
        Special callback for kamikaze that expects no response
        """
        try:
            # Try to get response, but don't expect success
            future.result()
            self.get_logger().info('Received kamikaze response (unexpected)')
        except Exception:
            # This is expected - the drone likely destroyed itself
            self.get_logger().info('No kamikaze response (drone likely destroyed as expected)')

    def time_callback(self, msg):
        """
        Handle game time updates
        """
        # Only log time every 30 seconds to avoid flooding
        if msg.data % 30 == 0 and msg.data > 0:
            self.get_logger().info(f'Game time remaining: {msg.data} seconds')


def main(args=None):
    rclpy.init(args=args)
    demo_node = DemoNode()
    
    try:
        rclpy.spin(demo_node)
    except KeyboardInterrupt:
        demo_node.get_logger().info('Demo interrupted by user')
    finally:
        demo_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
