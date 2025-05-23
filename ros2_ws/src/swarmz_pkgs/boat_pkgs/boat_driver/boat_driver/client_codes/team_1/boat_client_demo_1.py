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
        
        # Alignment parameters
        self.declare_parameter("alignment_pitch_tolerance", 0.1)   # Increased from 0.05
        self.declare_parameter("alignment_yaw_tolerance", 0.1)     # Increased from 0.05
        self.declare_parameter("alignment_max_attempts", 5)
        self.declare_parameter("alignment_check_delay", 0.5)       # seconds
        
        # Target tracking for cannon alignment
        self.target_pitch = None
        self.target_yaw = None
        self.alignment_attempts = 0
        self.alignment_timer = None
        self.missile_fired = False  # Flag to prevent multiple firings
        
        # Position tracking variables
        self.pose_x = None
        self.pose_y = None
        self.pose_z = None
        self.yaw = None
        
        # Initial spawn position tracking (in map frame)
        self.spawn_x = None
        self.spawn_y = None
        self.spawn_z = None
        self.spawn_yaw = None
        self.first_localization_received = False
        
        self.health = None
        self.incoming_message = None
        self.out_going_message = None
        self.assign_target_done = False
        self.fire_missile_done = True
        self.mission_state = 0  # 0: init, 1: targeting, 2: moving
        
        # Set destination based on team (in map frame)
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
        self.targeting_timeout = 12.0  # Give 12 seconds to find a target before moving on (arbitraty numbere)
        self.max_detection_range = 20.0  # Maximum range in meters to consider targets

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
            # Log navigation progress every 5 seconds
            elapsed = (self.get_clock().now() - self.startup_time).nanoseconds / 1e9
            if int(elapsed) % 5 == 0:
                self.get_logger().info(f"Navigation status: Current position ({self.pose_x:.1f}, {self.pose_y:.1f}), "
                                       f"Target ({self.desired_position_x}, {self.desired_position_y})")

    def update_position_callback(self, msg):
        # Getting the current pose and yaw from the localization topic
        self.pose_x = msg.position.x
        self.pose_y = msg.position.y
        self.pose_z = msg.position.z
        q = msg.orientation
        quaternion = [q.x, q.y, q.z, q.w]
        roll, pitch, yaw = tf_transformations.euler_from_quaternion(quaternion)
        self.yaw = yaw*180.0/math.pi
    
        # Record the initial spawn position (in map frame) from the first localization message
        if not self.first_localization_received:
            self.spawn_x = self.pose_x
            self.spawn_y = self.pose_y
            self.spawn_z = self.pose_z
            self.spawn_yaw = self.yaw
            self.first_localization_received = True
            self.get_logger().info(f"Initial spawn position set: x={self.spawn_x}, y={self.spawn_y}, z={self.spawn_z}, yaw={self.spawn_yaw}")
    
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
            
            # Store target angles for alignment verification
            self.target_pitch = possible_targets[0][1]
            self.target_yaw = possible_targets[0][2]
            
            # Aim at closest target
            self.send_goal(self.target_pitch, self.target_yaw)

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
        """Navigate to specified coordinates in the map frame"""
        if self.pose_x is None or self.pose_y is None or self.yaw is None:
            return
            
        # Calculate target position relative to current boat position
        # 1. First find the difference vector in map frame
        dx = x - self.pose_x  # Target x - current x in map frame
        dy = y - self.pose_y  # Target y - current y in map frame
        
        # Calculate direct distance to target in map frame
        distance_to_target_map = math.sqrt(dx**2 + dy**2)
        
        # Calculate desired heading in the map frame
        desired_heading_map = math.atan2(dy, dx) * 180.0 / math.pi
        
        # Convert desired heading to local frame by finding the difference
        # between desired heading and current yaw
        angle_error = desired_heading_map - self.yaw
        
        # Normalize angle to -180 to 180 range
        while angle_error > 180.0:
            angle_error -= 360.0
        while angle_error < -180.0:
            angle_error += 360.0
            
        # Log detailed navigation info
        self.get_logger().info(f"Navigation details: Current=({self.pose_x:.1f}, {self.pose_y:.1f}), Target=({x}, {y})")
        self.get_logger().info(f"Current heading: {self.yaw:.1f}°, Desired heading: {desired_heading_map:.1f}°, Error: {angle_error:.1f}°")
        self.get_logger().info(f"Distance to target: {distance_to_target_map:.1f}m")
        
        # If we're close enough to target
        if distance_to_target_map <= self.get_parameter("tolerance_distance").value:
            self.get_logger().info(f"Desired position reached: x={self.pose_x:.1f}, y={self.pose_y:.1f}")
            self.thruster_cmd(0.0, 0.0)
            return
            
        # Check if we need to turn or move forward
        # Use a more lenient angle tolerance for real-world conditions
        if abs(angle_error) > 20.0:  # More aggressive turning threshold
            # Turn to face target direction
            self.get_logger().info(f"Turning to angle {desired_heading_map:.1f}° (error: {angle_error:.1f}°)")
            self.turn_to_heading(angle_error)
        else:
            # Move forward toward destination with some course correction
            self.get_logger().info(f"Moving forward, distance: {distance_to_target_map:.1f}m with heading adjustment")
            self.move_with_course_correction(distance_to_target_map, angle_error)

    def turn_to_heading(self, angle_error):
        """Improved turning function with more aggressive behavior"""
        # Determine turn direction and power based on error
        turn_power = self.thruster_max_speed  # Use max power for faster turning
        
        if abs(angle_error) < 60.0:
            # For smaller errors, scale down power but keep it significant
            turn_power = max(1.0, abs(angle_error) * 0.05)  # At least 1.0 power
            
        # Apply turn commands based on direction
        if angle_error < 0:  # Need to turn left
            self.thruster_cmd(turn_power, -turn_power)
            self.get_logger().debug(f"Turning left with power {turn_power:.2f}")
        else:  # Need to turn right
            self.thruster_cmd(-turn_power, turn_power)
            self.get_logger().debug(f"Turning right with power {turn_power:.2f}")
    
    def move_with_course_correction(self, distance, angle_error):
        """Move forward with minor course corrections"""
        # Base forward power - always significant
        forward_power = self.thruster_max_speed
        if distance < 10.0:
            forward_power = max(1.0, distance * 0.2)  # At least 1.0 power
            
        # Apply small differential based on heading error for course correction
        correction = angle_error * 0.05  # Small correction factor
        
        left_power = forward_power - correction
        right_power = forward_power + correction
        
        # Ensure minimum effective power
        left_power = max(0.8, min(self.thruster_max_speed, left_power))
        right_power = max(0.8, min(self.thruster_max_speed, right_power))
        
        self.get_logger().debug(f"Forward with correction: L={left_power:.2f}, R={right_power:.2f}")
        self.thruster_cmd(left_power, right_power)

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
        """After cannon is aimed, verify alignment before firing"""
        result = future.result().result
        self.get_logger().info(f'Cannon aim result: Success = {result.success}')
        
        if result.success:
            # Reset alignment attempts counter
            self.alignment_attempts = 0
            
            # Start alignment verification
            self.get_logger().info('Verifying cannon alignment before firing...')
            self.verify_alignment()
        else:
            self.get_logger().error('Failed to aim cannon, aborting missile fire')
            self.fire_missile_done = True
            self.assign_target_done = False
            # Reset targeting state to try again
            self.mission_state = 1
            self.targeting_start_time = self.get_clock().now()

    def verify_alignment(self):
        """Verify the cannon is aligned with the target before firing"""
        # Cancel any existing alignment timer
        if self.alignment_timer is not None:
            self.alignment_timer.cancel()
        
        # If max attempts reached, abort
        if self.alignment_attempts >= self.get_parameter("alignment_max_attempts").value:
            self.get_logger().warn(f"Failed to achieve stable alignment after {self.alignment_attempts} attempts. Aborting.")
            self.fire_missile_done = True
            self.assign_target_done = False
            self.missile_fired = False  # Reset missile fired flag
            # Reset targeting state to try again
            self.mission_state = 1
            self.targeting_start_time = self.get_clock().now()
            return
            
        # Request current cannon position
        self.alignment_attempts += 1
        self._send_goal_future = self.cannon_client.send_goal_async(
            Cannon.Goal(pitch=self.target_pitch, yaw=self.target_yaw),
            feedback_callback=self.check_alignment_feedback
        )
        
        # Schedule next alignment check
        self.alignment_timer = self.create_timer(
            self.get_parameter("alignment_check_delay").value,
            self.verify_alignment_timer_callback
        )
    
    def check_alignment_feedback(self, feedback_msg):
        """Check if cannon is properly aligned based on feedback"""
        # Avoid processing feedback after firing
        if self.missile_fired:
            return
            
        feedback = feedback_msg.feedback
        pitch_error = abs(feedback.current_pitch - self.target_pitch)
        yaw_error = abs(feedback.current_yaw - self.target_yaw)
        
        pitch_tolerance = self.get_parameter("alignment_pitch_tolerance").value
        yaw_tolerance = self.get_parameter("alignment_yaw_tolerance").value
        
        # Check if alignment is within tolerance
        if pitch_error <= pitch_tolerance and yaw_error <= yaw_tolerance:
            # Cancel the verification timer
            if self.alignment_timer is not None:
                self.alignment_timer.cancel()
                self.alignment_timer = None
                
            # Only fire if we haven't fired already
            if not self.missile_fired:
                self.get_logger().info(f'Cannon aligned! Pitch error: {pitch_error:.3f}, Yaw error: {yaw_error:.3f}')
                self.missile_fired = True  # Set flag to prevent multiple firings
                self.fire_missile()
        else:
            self.get_logger().debug(f'Alignment not yet achieved. Pitch error: {pitch_error:.3f}, Yaw error: {yaw_error:.3f}')
    
    def verify_alignment_timer_callback(self):
        """Timer callback to check alignment status"""
        # This is called if the feedback callback didn't detect proper alignment
        if self.missile_fired:
            # Already fired, don't try to align again
            if self.alignment_timer is not None:
                self.alignment_timer.cancel()
                self.alignment_timer = None
            return
            
        # Get the latest cannon position feedback
        self._send_goal_future = self.cannon_client.send_goal_async(
            Cannon.Goal(pitch=self.target_pitch, yaw=self.target_yaw),
            feedback_callback=self.check_alignment_feedback
        )
        
        # Log why alignment might be failing
        self.get_logger().warn(f'Alignment check {self.alignment_attempts}/{self.get_parameter("alignment_max_attempts").value} - '
                              f'Target pitch/yaw: ({self.target_pitch:.3f}, {self.target_yaw:.3f})')
        
        # Cancel this timer to prevent it from firing again
        if self.alignment_timer is not None:
            self.alignment_timer.cancel()
            self.alignment_timer = None
            
        # Try alignment again
        self.verify_alignment()
        
    def fire_missile(self):
        """Fire missile after verifying alignment"""
        # Fire missile once
        if self.missile_fired:
            self.get_logger().info('Missile already fired, skipping')
            return
            
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
            if response.has_fired:
                self.get_logger().info(f"Missile fired: {response.has_fired}, ammo: {response.ammo}")
                # Move to navigation phase
                self.mission_state = 2
                self.get_logger().info(f"Moving to position: ({self.desired_position_x}, {self.desired_position_y})")
                
                # Stop any further cannon alignment or firing attempts
                self.fire_missile_done = True
                self.missile_fired = True
            else:
                self.get_logger().warn(f"Missile failed to fire. Ammo: {response.ammo}")
                # Reset so we can try again
                self.fire_missile_done = True 
                self.assign_target_done = False
                self.missile_fired = False
                self.mission_state = 1
        except Exception as e:
            self.get_logger().error(f"Missile service call failed: {e}")
            self.fire_missile_done = True
            self.assign_target_done = False
            self.missile_fired = False
    
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