#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Int32, String
from geometry_msgs.msg import Pose
import math
import signal
import time
from rclpy.action import ActionClient
from cannon_interfaces.action import Cannon
import tf_transformations
from swarmz_interfaces.srv import Missile
from swarmz_interfaces.msg import Detections

class boat_test(Node):

    def __init__(self):
        super().__init__('boat_client')
        self.get_logger().info("Launching ship client example")
        self.declare_parameter("tolerance_angle", 5.0)
        self.declare_parameter("tolerance_distance", 10.0)
        self.node_namespace = self.get_namespace()

        # Publishers for thrusters control
        self.left_thruster_pub = self.create_publisher(Float64, f'{self.node_namespace}/left_propeller_cmd', 10)
        self.right_thruster_pub = self.create_publisher(Float64, f'{self.node_namespace}/right_propeller_cmd', 10)
        # Subscriber to get ship's position
        self.pos_subscriber = self.create_subscription(Pose, f'{self.node_namespace}/localization', self.update_position_callback, 10)
        # Subscriber to get ship's detections
        self.detections_subscriber = self.create_subscription(Detections, f'{self.node_namespace}/detections', self.detections_callback, 10)
        # Subscriber to get ship's health
        self.health_subscriber = self.create_subscription(Int32, f'{self.node_namespace}/health', self.health_callback, 10)
        # Subscriber to get ship's incoming messages
        self.incoming_subscriber = self.create_subscription(String, f'{self.node_namespace}/incoming_messages', self.incoming_callback, 10)
        # Subscriber to get ship's outgoing messages
        self.outgoing_subscriber = self.create_subscription(String, f'{self.node_namespace}/out_going_messages', self.outgoing_callback, 10)

        # Parameters and initial values
        self.thruster_min_speed = -2.5
        self.thruster_max_speed = 2.5
        self.cannon_max_pitch = math.pi/2.0
        self.cannon_max_yaw = math.pi
        self.cannon_x_offset = 1.65
        self.cannon_z_offset = -0.48
        self.cannon_pitch_offset = math.pi/2.0
        self.pose_x = None
        self.pose_y = None
        self.pose_z = None
        self.yaw = None
        self.health = None
        self.incoming_message = None
        self.out_going_message = None
        self.assign_target_done = False
        self.fire_missile_done = True
        self.mission_state = 0  # 0: init, 1: targeting, 2: moving
        
        # Set destination based on team
        if self.node_namespace == "/flag_ship_1":
            self.desired_position_x = 125.0
            self.desired_position_y = 240.0
        else:
            self.desired_position_x = 125.0
            self.desired_position_y = 260.0

        # Action client for cannon control
        self.cannon_client = ActionClient(self, Cannon, f'{self.node_namespace}/cannon')

        # Service client for missile
        self.missile_client = self.create_client(Missile, '/game_master/fire_missile')

        # Create timer for mission execution (replacing the manual looping)
        self.timer = self.create_timer(1.0, self.mission_timer_callback)

        # Handle shutdown signal to publish 0 commands and stop the boat
        signal.signal(signal.SIGINT, self.shutdown_callback)
        signal.signal(signal.SIGTERM, self.shutdown_callback)
        
        # Wait 30 seconds before starting the mission
        self.get_logger().info("Waiting for drones to take off (30 seconds)...")
        self.startup_time = self.get_clock().now()

        # Add detection timeout tracking
        self.targeting_start_time = None
        self.targeting_timeout = 10.0  # Give 10 seconds to find a target before moving on
        self.max_detection_range = 60.0  # Maximum range in meters to consider targets

        self.get_logger().info("Initialization finished")

    def mission_timer_callback(self):
        """Main state machine for the boat mission"""
        # Initial waiting period (30 seconds)
        if self.mission_state == 0:
            elapsed = (self.get_clock().now() - self.startup_time).nanoseconds / 1e9
            if elapsed >= 30.0:
                self.get_logger().info("Wait period complete. Starting targeting phase...")
                self.mission_state = 1
                self.targeting_start_time = self.get_clock().now()
                
        # Targeting phase - check for timeout
        elif self.mission_state == 1:
            # If we've been targeting for too long without finding a valid target, move on
            elapsed_targeting = (self.get_clock().now() - self.targeting_start_time).nanoseconds / 1e9
            if elapsed_targeting >= self.targeting_timeout and not self.assign_target_done:
                self.get_logger().info("No targets detected within 60m after timeout. Moving to objective.")
                self.mission_state = 2
                self.get_logger().info(f"Moving to position: ({self.desired_position_x}, {self.desired_position_y})")
                
        # Navigation phase
        elif self.mission_state == 2:
            self.go_to(self.desired_position_x, self.desired_position_y)

    def update_position_callback(self, msg):
        # Getting the current pose and yaw from the localization topic
        self.pose_x = msg.position.x
        self.pose_y = msg.position.y
        self.pose_z = msg.position.z
        q = msg.orientation
        quaternion = [q.x, q.y, q.z, q.w]
        roll, pitch, yaw = tf_transformations.euler_from_quaternion(quaternion)
        self.yaw = yaw*180.0/math.pi
    
    def health_callback(self, msg):
        self.health = msg.data
    
    def incoming_callback(self, msg):
        self.incoming_message = msg.data

    def outgoing_callback(self, msg):
        self.out_going_message = msg.data

    def detections_callback(self, msg):
        """Process target detections based on current mission state"""
        # Only process detections in targeting phase
        if self.mission_state != 1 or not self.fire_missile_done or self.assign_target_done:
            return
            
        possible_targets = []
        for target in msg.detections:
            x = target.relative_position.x
            y = target.relative_position.y
            z = target.relative_position.z
            distance = self.calculate_distance(x, y, z)
            
            # Only consider targets within 60m range
            if distance <= self.max_detection_range:
                target_pitch, target_yaw = self.calculate_pitch_yaw(x, y, z)
                
                # Check if target is within cannon's range
                if abs(target_pitch) < self.cannon_max_pitch and abs(target_yaw) < self.cannon_max_yaw:
                    possible_targets.append([distance, target_pitch, target_yaw])
        
        if possible_targets:
            # Sort by distance (closest first)
            possible_targets.sort(key=lambda x: x[0])
            self.assign_target_done = True
            closest_target_distance = possible_targets[0][0]
            self.get_logger().info(f"Target acquired at {closest_target_distance:.1f}m! Aiming cannon...")
            
            # Aim at closest target
            self.send_goal(possible_targets[0][1], possible_targets[0][2])

    def calculate_distance(self, x, y, z):
        return math.sqrt(math.pow(x, 2) + math.pow(y, 2) + math.pow(z, 2))
    
    def calculate_pitch_yaw(self, x, y, z):
        """Calculate cannon angles to target"""
        x = x - self.cannon_x_offset 
        z = z - self.cannon_z_offset
        target_yaw = -math.atan2(y, x) 
        horizontal_distance = math.sqrt(x**2 + y**2)
        target_pitch = math.atan2(z, horizontal_distance) + self.cannon_pitch_offset
        return target_pitch, target_yaw
    
    def go_to(self, x, y):
        """Navigate to specified coordinates"""
        if self.pose_x is None or self.pose_y is None or self.yaw is None:
            return
             
        desired_orientation_yaw = math.atan2((y - self.pose_y), (x - self.pose_x))*180.0/math.pi
        distance_to_target = self.calculate_distance(self.pose_x - x, self.pose_y - y, 0)
        
        if abs(desired_orientation_yaw - self.yaw) > self.get_parameter("tolerance_angle").value:
            self.go_to_angle(desired_orientation_yaw)
        elif distance_to_target > self.get_parameter("tolerance_distance").value:
            self.go_to_position(distance_to_target)
        else:
            self.get_logger().info(f"Desired position reached: x={self.pose_x:.1f}, y={self.pose_y:.1f}")
            self.thruster_cmd(0.0, 0.0)

    def go_to_position(self, distance_to_target):
        """Move forward toward destination"""
        if distance_to_target < 5.0:
            cmd = distance_to_target * 0.6
        else:
            cmd = self.thruster_max_speed
        self.thruster_cmd(cmd, cmd)
    
    def go_to_angle(self, goal):
        """Rotate to face target direction"""
        error = goal - self.yaw
        
        if abs(error) < 20.0:
            cmd = abs(error) * 0.25
        else:
            cmd = self.thruster_max_speed

        if error < 0:
            self.thruster_cmd(cmd, -cmd)
        else:
            self.thruster_cmd(-cmd, cmd)
    
    def thruster_cmd(self, cmd_left, cmd_right):
        """Send commands to thrusters with limits"""
        if cmd_left > self.thruster_max_speed:
            cmd_left = self.thruster_max_speed
        elif cmd_left < self.thruster_min_speed:
            cmd_left = self.thruster_min_speed
        
        if cmd_right > self.thruster_max_speed:
            cmd_right = self.thruster_max_speed
        elif cmd_right < self.thruster_min_speed:
            cmd_right = self.thruster_min_speed

        left_thrust_msg = Float64()
        right_thrust_msg = Float64()
        left_thrust_msg.data = cmd_left
        right_thrust_msg.data = cmd_right
        self.left_thruster_pub.publish(left_thrust_msg)
        self.right_thruster_pub.publish(right_thrust_msg)

    def send_goal(self, pitch, yaw):
        """Aim cannon at target"""
        goal_msg = Cannon.Goal()
        goal_msg.pitch = pitch
        goal_msg.yaw = yaw
        self.cannon_client.wait_for_server()
        self.get_logger().info(f'Aiming cannon: pitch={pitch:.2f}, yaw={yaw:.2f}')
        self._send_goal_future = self.cannon_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        # Uncomment for detailed aiming feedback
        # self.get_logger().info(f'Feedback: current_pitch={feedback.current_pitch:.2f}, '
        #                        f'current_yaw={feedback.current_yaw:.2f}')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """After cannon is aimed, fire the missile"""
        result = future.result().result
        self.get_logger().info(f'Cannon aim result: Success = {result.success}')
        
        # Fire missile
        while not self.missile_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for missile service...')
            
        missile_request = Missile.Request()
        missile_request.robot_name = self.node_namespace
        future = self.missile_client.call_async(missile_request)
        future.add_done_callback(self.handle_missile_response)
        self.fire_missile_done = False

    def handle_missile_response(self, future):
        """After missile is fired, move to navigation phase"""
        try:
            response = future.result()
            self.get_logger().info(f"Missile fired: {response.has_fired}, ammo: {response.ammo}")
            # Move to navigation phase
            self.mission_state = 2
            self.get_logger().info(f"Moving to position: ({self.desired_position_x}, {self.desired_position_y})")
        except Exception as e:
            self.get_logger().error(f"Missile service call failed: {e}")
    
    def shutdown_callback(self, signum, frame):
        """Clean shutdown"""
        self.thruster_cmd(0.0, 0.0)
        self.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    
    client = boat_test()
    
    try:
        # Use standard spin instead of manual spin_once loop
        rclpy.spin(client)
    except KeyboardInterrupt:
        print("Program interrupted by user")
    finally:
        # Ensure clean shutdown
        client.thruster_cmd(0.0, 0.0)
        rclpy.shutdown()

if __name__ == '__main__':
    main()