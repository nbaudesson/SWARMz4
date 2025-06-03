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
from swarmz_interfaces.msg import Detections, Detection

class BoatClient(Node):

    def __init__(self):
        super().__init__('boat_client')
        self.get_logger().info("Launching unified ship client")
        self.declare_parameter("tolerance_angle", 5.0)
        self.declare_parameter("tolerance_distance", 10.0)
        self.node_namespace = self.get_namespace()
        
        # State machine states
        self.STATE_WAITING = 0       # Initial wait period
        self.STATE_TARGETING = 1     # Searching for drone targets
        self.STATE_NAVIGATING = 2    # Moving to center of map
        self.state = self.STATE_WAITING
        
        # Missile states
        self.MISSILE_IDLE = "IDLE"
        self.MISSILE_TARGET_ACQUIRED = "TARGET_ACQUIRED"
        self.MISSILE_FIRING = "FIRING"
        self.MISSILE_FIRED = "FIRED"
        self.MISSILE_FAILED = "FAILED"
        self.MISSILE_COOLDOWN = "COOLDOWN"
        self.missile_state = self.MISSILE_IDLE
        
        # Add missile cooldown tracking
        self.missile_cooldown = 6.0  # 6 seconds from game_master_params.yaml
        self.last_missile_fire_time = None

        # Target types
        self.TARGET_TYPE_NONE = 0
        self.TARGET_TYPE_DRONE = 1
        self.TARGET_TYPE_BOAT = 2
        self.current_target_type = self.TARGET_TYPE_NONE

        # Publishers for thrusters control
        self.left_thruster_pub = self.create_publisher(Float64, f'{self.node_namespace}/left_propeller_cmd', 10)
        self.right_thruster_pub = self.create_publisher(Float64, f'{self.node_namespace}/right_propeller_cmd', 10)
        
        # Subscriber to get ship's position
        self.pos_subscriber = self.create_subscription(Pose, f'{self.node_namespace}/localization', self.position_callback, 10)
        
        # Subscriber to get ship's detections
        self.detections_subscriber = self.create_subscription(Detections, f'{self.node_namespace}/detections', self.detections_callback, 10)
        
        # Subscriber to get ship's health
        self.health_subscriber = self.create_subscription(Int32, f'{self.node_namespace}/health', self.health_callback, 10)
        
        # Message communication
        self.message_publisher = self.create_publisher(String, f'{self.node_namespace}/out_going_messages', 10)
        self.message_subscriber = self.create_subscription(String, f'{self.node_namespace}/incoming_messages', self.message_callback, 10)

        # Parameters
        self.thruster_min_speed = -1.2  # Updated to match max speed
        self.thruster_max_speed = 1.2   # Updated with correct max speed
        self.cannon_max_pitch = math.pi/2.0
        self.cannon_max_yaw = math.pi
        self.cannon_x_offset = 1.65
        self.cannon_z_offset = -0.48
        self.cannon_pitch_offset = math.pi/2.0
        
        # Alignment parameters
        self.declare_parameter("alignment_pitch_tolerance", 0.15)
        self.declare_parameter("alignment_yaw_tolerance", 0.15)
        
        # Position tracking
        self.pose_x = None
        self.pose_y = None
        self.pose_z = None
        self.yaw = None
        self.health = None
        
        # Initial spawn position tracking
        self.spawn_x = None
        self.spawn_y = None
        self.first_localization_received = False
        
        # Destination - center of map
        self.destination_x = 125.0
        self.destination_y = 250.0
        
        # Target tracking for cannon alignment
        self.target_pitch = None
        self.target_yaw = None
        self.goal_handle = None
        self.current_detections = []
        
        # Maximum firing range
        self.max_detection_range = 81.0  # Use the ship_missile_range from game_master_params.yaml
        
        # Targeting
        self.target_info = None  # Will store [distance, pitch, yaw, is_drone] for current target
        
        # Action client for cannon control
        self.cannon_client = ActionClient(self, Cannon, f'{self.node_namespace}/cannon')
        
        # Service client for missile
        self.missile_client = self.create_client(Missile, '/game_master/fire_missile')
        
        # Timer configurations
        self.startup_time = self.get_clock().now()
        self.startup_wait_seconds = 30.0  # Wait 30 seconds before starting
        
        # Logging control
        self.last_nav_log_time = 0.0
        self.nav_log_interval = 30.0  # Only log navigation details every 30 seconds
        
        # Anti-stuck measures
        self.last_position_x = None
        self.last_position_y = None
        self.stuck_counter = 0
        self.stuck_threshold = 5
        self.last_significant_movement_time = time.time()
        self.movement_timeout = 60.0  # Force unstuck maneuver if no significant movement in 60 seconds
        
        # Add tracking for drone targeting attempts
        self.drone_targeting_attempts = 0
        self.max_drone_targeting_attempts = 2  # Number of attempts before moving to navigation
        
        # Add aiming timeout tracking
        self.aiming_start_time = None
        self.aiming_timeout = 1.0  # 1 second timeout
        self.aiming_timer = None
    
        # Speed control for different states
        self.targeting_speed_factor = 0.1  # 10% of normal speed during targeting
        self.cruising_speed_factor = 0.95   # 95% of max speed during normal navigation 
        self.current_speed_factor = self.cruising_speed_factor
    
        # Create timers
        self.main_timer = self.create_timer(1.0, self.main_loop)
        self.targeting_timer = self.create_timer(1.0, self.update_target)
        self.initial_movement_done = False
        self.initial_movement_timer = self.create_timer(0.5, self.initial_movement_impulse)
        
        # Shutdown handling
        signal.signal(signal.SIGINT, self.shutdown_callback)
        signal.signal(signal.SIGTERM, self.shutdown_callback)
        
        self.get_logger().info("Initialization complete")
    
    def initial_movement_impulse(self):
        """Send an initial impulse to get the boat moving from its starting position"""
        if not self.initial_movement_done and self.pose_x is not None:
            self.get_logger().info("Applying initial movement impulse to overcome inertia")
            # Reduced initial impulse to respect max speed
            self.thruster_cmd(1.2, 1.2)
            time.sleep(0.5)  # Apply impulse for half a second
            # Immediately stop after impulse
            self.thruster_cmd(0.0, 0.0)
            self.get_logger().info("Initial impulse complete, stopping until targeting is complete")
            self.initial_movement_done = True
            # Stop this timer
            self.initial_movement_timer.cancel()
    
    def main_loop(self):
        """Main state machine for boat operation"""
        if self.pose_x is None:
            return  # Wait for position data
            
        # STATE: Initial waiting period
        if self.state == self.STATE_WAITING:
            elapsed = (self.get_clock().now() - self.startup_time).nanoseconds / 1e9
            if elapsed >= self.startup_wait_seconds:
                self.get_logger().info("Wait period complete. Starting mission...")
                
                # Check if there are any drones within 30 meters
                if self.has_drones_within_range(30.0):
                    self.get_logger().info("Drones detected within 30m. Entering targeting mode.")
                    self.state = self.STATE_TARGETING
                    # Set reduced speed for targeting
                    self.current_speed_factor = self.targeting_speed_factor
                    self.get_logger().info(f"Reducing speed to {self.targeting_speed_factor*100}% for targeting phase")
                    # Explicitly stop thrusters until first shot is fired
                    self.thruster_cmd(0.0, 0.0)
                else:
                    self.get_logger().info("No drones detected within 30m. Skipping targeting and going straight to navigation.")
                    self.state = self.STATE_NAVIGATING
                    self.current_speed_factor = self.cruising_speed_factor
                    self.get_logger().info(f"Setting cruising speed to {self.cruising_speed_factor*100}% of max")
        
        # STATE: Targeting - look for drones to shoot
        elif self.state == self.STATE_TARGETING:
            # Keep the boat stationary during initial targeting phase
            # unless we've already fired our first shots
            if self.drone_targeting_attempts == 0:
                # Ensure boat is stopped during initial targeting
                self.thruster_cmd(0.0, 0.0)
            else:
                # After first shot, allow very slow movement (use go_to with targeting speed)
                self.go_to(self.destination_x, self.destination_y)
                
            if self.missile_state == self.MISSILE_FIRED:
                # Only transition to NAVIGATING state if we've made enough attempts
                # (the state transition is now handled in handle_missile_response)
                self.missile_state = self.MISSILE_IDLE
                self.target_info = None
                
        # STATE: Navigation - move to center while looking for targets
        elif self.state == self.STATE_NAVIGATING:
            # Check if boat is stuck
            self.detect_and_fix_stuck()
                
            # Normal navigation
            self.go_to(self.destination_x, self.destination_y)
            
            # If we acquired a target while navigating, temporarily stop to fire
            if self.missile_state == self.MISSILE_TARGET_ACQUIRED:
                # Boat stops moving while firing
                self.thruster_cmd(0.0, 0.0)
                
                # Once missile is fired, we'll continue navigating
                if self.missile_state == self.MISSILE_FIRED:
                    self.missile_state = self.MISSILE_IDLE
                    self.target_info = None
    
    def detect_and_fix_stuck(self):
        """Detect if boat is stuck and apply recovery maneuvers"""
        current_time = time.time()
        
        if self.last_position_x is not None and self.last_position_y is not None:
            # Calculate distance moved since last check
            distance_moved = math.sqrt(
                (self.pose_x - self.last_position_x)**2 + 
                (self.pose_y - self.last_position_y)**2
            )
            
            # Detect if boat is stuck - adjusted for 1.2 m/s max speed
            if distance_moved < 0.2:  # Should move at least ~17% of max speed
                self.stuck_counter += 1
                if self.stuck_counter >= self.stuck_threshold:
                    self.get_logger().warn(f"Boat appears to be stuck at ({self.pose_x:.1f}, {self.pose_y:.1f}) - applying unstuck maneuver")
                    # Apply unstuck maneuvers respecting max speed
                    self.thruster_cmd(-1.2, 1.2)  # Max rotation within limits
                    time.sleep(2.5)  # Longer turn to ensure rotation
                    self.thruster_cmd(1.2, 1.2)  # Max forward within limits
                    time.sleep(2.0)  # Longer forward impulse
                    self.stuck_counter = 0
                    self.last_significant_movement_time = current_time
            else:
                self.stuck_counter = 0
                if distance_moved > 0.5:  # If we moved more than ~40% of max speed, reset timeout
                    self.last_significant_movement_time = current_time
        
        # Force unstuck maneuver if no significant movement for too long
        if current_time - self.last_significant_movement_time > self.movement_timeout:
            self.get_logger().warn(f"No significant movement in {self.movement_timeout}s, performing emergency maneuver")
            # Recovery maneuvers respecting max speed
            self.get_logger().info("Performing emergency reorientation within speed limits")
            # Alternating pattern to break out of stuck situations
            self.thruster_cmd(-1.2, 1.2)  # Max rotation within limits
            time.sleep(3.5)  # Longer rotation time
            self.thruster_cmd(1.2, 1.2)  # Max forward within limits
            time.sleep(2.5)
            self.thruster_cmd(-1.2, -1.2)  # Backward movement
            time.sleep(1.5)
            self.thruster_cmd(1.2, 1.2)  # Forward again
            time.sleep(2.0)
            self.last_significant_movement_time = current_time
            
        # Update position history
        self.last_position_x = self.pose_x
        self.last_position_y = self.pose_y
    
    def update_target(self):
        """Update target information and fire if ready"""
        # Check if in targeting state with previous attempts but no more targets
        if self.state == self.STATE_TARGETING and self.drone_targeting_attempts > 0:
            has_drones = self.has_drones_within_range(self.max_detection_range)
            if not has_drones:
                self.get_logger().warn("No more drones in range after initial targeting. Switching to navigation.")
                self.state = self.STATE_NAVIGATING
                self.current_speed_factor = self.cruising_speed_factor
                self.get_logger().info(f"Setting cruising speed to {self.cruising_speed_factor*100}% of max")
                return
    
        if not self.current_detections:
            return
    
        # Check cooldown period before targeting
        current_time = self.get_clock().now()
        if self.last_missile_fire_time is not None:
            elapsed = (current_time - self.last_missile_fire_time).nanoseconds / 1e9
            if elapsed < self.missile_cooldown:
                # Still in cooldown period
                remaining = self.missile_cooldown - elapsed
                if self.missile_state != self.MISSILE_COOLDOWN:
                    self.get_logger().info(f"Missile system in cooldown: {remaining:.1f}s remaining")
                    self.missile_state = self.MISSILE_COOLDOWN
                return
    
        # We're out of cooldown now
        if self.missile_state == self.MISSILE_COOLDOWN:
            self.get_logger().info("Missile system ready to fire")
            self.missile_state = self.MISSILE_IDLE
            
        # Only look for targets when not actively firing
        if self.missile_state not in [self.MISSILE_IDLE, self.MISSILE_FAILED]:
            return
    
        # Reset failed state so we can try again
        if self.missile_state == self.MISSILE_FAILED:
            self.missile_state = self.MISSILE_IDLE
            
        # Find best target from current detections
        self.find_best_target()
        
        # If target acquired and we're in targeting state or navigating, fire
        if self.target_info and self.missile_state == self.MISSILE_TARGET_ACQUIRED:
            # Aim cannon at target and fire when ready
            self.target_pitch = self.target_info[1]
            self.target_yaw = self.target_info[2]
            self.send_goal(self.target_pitch, self.target_yaw)
    
    def find_best_target(self):
        """Find the best target from current detections"""
        best_drone_target = None
        best_boat_target = None
        
        # Process all detections to find potential targets
        for target in self.current_detections:
            x = target.relative_position.x
            y = target.relative_position.y
            z = target.relative_position.z
            distance = self.calculate_distance(x, y, z)
            
            # Only consider targets within range
            if distance <= self.max_detection_range:
                target_pitch, target_yaw = self.calculate_pitch_yaw(x, y, z)
                
                # Check if target is within cannon's range
                if abs(target_pitch) < self.cannon_max_pitch and abs(target_yaw) < self.cannon_max_yaw:
                    # Determine if target is a drone or boat using vehicle_type field
                    is_drone = (target.vehicle_type == Detection.DRONE)
                    
                    if is_drone and (best_drone_target is None or distance < best_drone_target[0]):
                        best_drone_target = [distance, target_pitch, target_yaw, True]
                    elif not is_drone and not target.is_friend and (best_boat_target is None or distance < best_boat_target[0]):
                        best_boat_target = [distance, target_pitch, target_yaw, False]
        
        # Prioritize targets based on state:
        # - In TARGETING state, prefer drones
        # - In NAVIGATING state, take any target with preference to boats
        if self.state == self.STATE_TARGETING and best_drone_target:
            self.select_target(best_drone_target)
            self.current_target_type = self.TARGET_TYPE_DRONE
        elif self.state == self.STATE_NAVIGATING:
            if best_boat_target:
                self.select_target(best_boat_target)
                self.current_target_type = self.TARGET_TYPE_BOAT
            elif best_drone_target:
                self.select_target(best_drone_target)
                self.current_target_type = self.TARGET_TYPE_DRONE
    
    def select_target(self, target_info):
        """Select a target and update state"""
        self.target_info = target_info
        self.missile_state = self.MISSILE_TARGET_ACQUIRED
        target_type = "drone" if target_info[3] else "boat"
        self.get_logger().info(f"Target {target_type} acquired at {target_info[0]:.1f}m! Aiming cannon...")
    
    def position_callback(self, msg):
        """Handle position updates"""
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
            self.first_localization_received = True
            self.get_logger().info(f"Initial spawn position set: x={self.spawn_x}, y={self.spawn_y}")
    
    def health_callback(self, msg):
        """Handle health updates"""
        self.health = msg.data
        # Send a message if health is critically low
        if self.health <= 2:
            self.send_message(f"{self.node_namespace} health critical: {self.health}/6")
    
    def message_callback(self, msg):
        """Handle incoming messages"""
        # Check for position queries about this boat
        message_lower = msg.data.lower()
        if self.node_namespace.lower() in message_lower and "position" in message_lower:
            if self.pose_x is not None and self.pose_y is not None:
                self.get_logger().info(f"{self.node_namespace} reporting position: x={self.pose_x:.1f}, y={self.pose_y:.1f}")
    
    def detections_callback(self, msg):
        """Store detections for processing"""
        self.current_detections = msg.detections
    
    def calculate_distance(self, x, y, z):
        """Calculate 3D distance"""
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
        dx = x - self.pose_x
        dy = y - self.pose_y
        
        # Calculate direct distance to target in map frame
        distance_to_target = math.sqrt(dx**2 + dy**2)
        
        # Calculate desired heading in the map frame
        desired_heading = math.atan2(dx, dy) * 180.0 / math.pi
        
        # Convert desired heading to local frame
        angle_error = (desired_heading - self.yaw) % 360.0
        if angle_error > 180.0:
            angle_error -= 360.0

        # Only log navigation details once every 20 seconds
        current_time = time.time()
        should_log = current_time - self.last_nav_log_time >= self.nav_log_interval
        
        if should_log:
            self.get_logger().info(f"Navigation details: Current=({self.pose_x:.1f}, {self.pose_y:.1f}), Target=({x}, {y})")
            self.get_logger().info(f"Current heading: {self.yaw:.1f}°, Desired heading: {desired_heading:.1f}°, Error: {angle_error:.1f}°")
            self.get_logger().info(f"Distance to target: {distance_to_target:.1f}m")
            self.last_nav_log_time = current_time
    
        # If we're close enough to target - always log this important event
        if distance_to_target <= self.get_parameter("tolerance_distance").value:
            self.get_logger().info(f"Desired position reached: x={self.pose_x:.1f}, y={self.pose_y:.1f}")
            self.thruster_cmd(0.0, 0.0)
            return
            
        # Check if we need to turn or move forward
        if abs(angle_error) > 20.0:
            # Turn to face target direction
            if should_log:
                self.get_logger().info(f"Turning to angle {desired_heading:.1f}° (error: {angle_error:.1f}°)")
            self.turn_to_heading(angle_error)
        else:
            # Move forward toward destination with course correction
            if should_log:
                self.get_logger().info(f"Moving forward, distance: {distance_to_target:.1f}m with heading adjustment")
            self.move_with_course_correction(distance_to_target, angle_error)

    def turn_to_heading(self, angle_error):
        """Improved turning function with appropriate turn power"""
        # Scale turn power based on angle error but respect max speed
        if abs(angle_error) > 90.0:
            turn_power = self.thruster_max_speed  # Use max speed for large turns
        elif abs(angle_error) < 60.0:
            # For smaller errors, scale down power
            turn_power = max(0.5, min(self.thruster_max_speed, abs(angle_error) * 0.01))
        else:
            turn_power = self.thruster_max_speed * 0.8
        
        # Apply turn commands based on direction
        if angle_error < 0:  # Need to turn left
            self.thruster_cmd(turn_power, -turn_power)
        else:  # Need to turn right
            self.thruster_cmd(-turn_power, turn_power)
    
    def move_with_course_correction(self, distance, angle_error):
        """Move forward with minor course corrections"""
        # Base power with current speed factor - ensure a clear application of speed factor
        forward_power = self.thruster_max_speed * self.current_speed_factor
        
        # Log actual forward power for debugging
        if self.state == self.STATE_TARGETING:
            self.get_logger().debug(f"Using targeting speed: {forward_power:.2f} ({self.current_speed_factor*100}%)")
    
        if distance < 10.0:
            forward_power = max(0.3, min(forward_power, distance * 0.1 * self.current_speed_factor))

        # Course correction factor
        correction = angle_error * 0.05
        
        left_power = forward_power - correction
        right_power = forward_power + correction
        
        # Ensure powers are within limits and apply speed factor
        left_power = max(-self.thruster_max_speed * self.current_speed_factor, 
                        min(self.thruster_max_speed * self.current_speed_factor, left_power))
        right_power = max(-self.thruster_max_speed * self.current_speed_factor, 
                        min(self.thruster_max_speed * self.current_speed_factor, right_power))
        
        self.thruster_cmd(left_power, right_power)

    def thruster_cmd(self, cmd_left, cmd_right, override_limits=False):
        """Send commands to thrusters with corrected limits"""
        # Even with override, never exceed the actual max speed of 1.2
        cmd_left = max(-self.thruster_max_speed, min(self.thruster_max_speed, cmd_left))
        cmd_right = max(-self.thruster_max_speed, min(self.thruster_max_speed, cmd_right))

        left_thrust_msg = Float64()
        right_thrust_msg = Float64()
        left_thrust_msg.data = cmd_left
        right_thrust_msg.data = cmd_right
        
        self.left_thruster_pub.publish(left_thrust_msg)
        self.right_thruster_pub.publish(right_thrust_msg)

    def send_message(self, message):
        """Send a message to other team members"""
        msg = String()
        msg.data = message
        self.message_publisher.publish(msg)
        self.get_logger().info(f"Sent message: {message}")
    
    def send_goal(self, pitch, yaw):
        """Aim cannon at target"""
        # Cancel any existing aiming timer
        if self.aiming_timer:
            self.aiming_timer.cancel()
        
        goal_msg = Cannon.Goal()
        goal_msg.pitch = pitch
        goal_msg.yaw = yaw
        self.cannon_client.wait_for_server()
        self.get_logger().info(f'Aiming cannon: pitch={pitch:.2f}, yaw={yaw:.2f}')
        
        # Record when we started aiming
        self.aiming_start_time = self.get_clock().now()
        
        # Start a timer to check for aiming timeout
        self.aiming_timer = self.create_timer(0.2, self.check_aiming_timeout)
        
        self._send_goal_future = self.cannon_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        """Handle cannon aiming feedback"""
        # Nothing to do here, but we could track aiming progress if needed
        pass

    def goal_response_callback(self, future):
        """Handle goal response"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            self.aiming_start_time = None
            if self.aiming_timer:
                self.aiming_timer.cancel()
                self.aiming_timer = None
            return

        self.get_logger().info('Goal accepted')
        self.goal_handle = goal_handle
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """After cannon is aimed, fire missile"""
        # Clean up aiming timeout tracking
        self.aiming_start_time = None
        if self.aiming_timer:
            self.aiming_timer.cancel()
            self.aiming_timer = None
            
        result = future.result().result
        self.get_logger().info(f'Cannon aim result: Success = {result.success}')
        
        if result.success:
            # Fire missile
            self.get_logger().info('Cannon aimed successfully, firing missile')
            self.fire_missile()
        else:
            self.get_logger().error('Failed to aim cannon, aborting missile fire')
            self.missile_state = self.MISSILE_FAILED

    def fire_missile(self):
        """Fire missile after aiming"""
        # First check if we're in cooldown
        current_time = self.get_clock().now()
        if self.last_missile_fire_time is not None:
            elapsed = (current_time - self.last_missile_fire_time).nanoseconds / 1e9
            if elapsed < self.missile_cooldown:
                # Still in cooldown period
                remaining = self.missile_cooldown - elapsed
                self.get_logger().warn(f"Can't fire - missile system in cooldown: {remaining:.1f}s remaining")
                self.missile_state = self.MISSILE_COOLDOWN
                return
    
        # Update state and fire missile
        self.missile_state = self.MISSILE_FIRING
        self.get_logger().info('Attempting to fire missile')
        
        # Fire missile
        while not self.missile_client.wait_for_service(timeout_sec=1.0):
            # self.get_logger().info('Waiting for missile service...')
            continue
            
        missile_request = Missile.Request()
        missile_request.robot_name = self.node_namespace
        self.get_logger().info(f"Firing missile from {self.node_namespace}")
        future = self.missile_client.call_async(missile_request)
        future.add_done_callback(self.handle_missile_response)

    def handle_missile_response(self, future):
        """Handle missile firing response"""
        try:
            response = future.result()
            if response.has_fired:
                # Record the time when missile was successfully fired
                self.last_missile_fire_time = self.get_clock().now()
                
                target_type = "drone" if self.current_target_type == self.TARGET_TYPE_DRONE else "boat"
                self.get_logger().info(f"Missile fired at {target_type}: {response.has_fired}, ammo: {response.ammo}")
                self.missile_state = self.MISSILE_FIRED
                
                # If we're in targeting state and this was a drone
                if self.state == self.STATE_TARGETING:
                    if self.current_target_type == self.TARGET_TYPE_DRONE:
                        self.drone_targeting_attempts += 1
                        
                        # Only transition to navigation after multiple attempts or if we're running low on ammo
                        if self.drone_targeting_attempts >= self.max_drone_targeting_attempts or response.ammo <= 1:
                            self.get_logger().info(f"Made {self.drone_targeting_attempts} drone targeting attempts. Moving to navigation phase.")
                            self.state = self.STATE_NAVIGATING
                            # Return to cruising speed
                            self.current_speed_factor = self.cruising_speed_factor
                            self.get_logger().info(f"Returning to cruising speed ({self.cruising_speed_factor*100}%)")
                        else:
                            self.get_logger().info(f"Looking for another drone target. Attempt {self.drone_targeting_attempts}/{self.max_drone_targeting_attempts}")
                            # Now that we've fired once, we can allow limited movement
                            self.get_logger().info("Allowing limited movement while looking for more targets")
                            # Reset missile state so we can target again
                            self.missile_state = self.MISSILE_IDLE
                            self.target_info = None
            else:
                self.get_logger().warn(f"Missile failed to fire. Ammo: {response.ammo}")
                
                # If "cooldown" is mentioned in the response, enter cooldown state
                if "cooldown" in response.message.lower():
                    # Set last_missile_fire_time to now minus some time to represent where we are in cooldown
                    # Using the class attribute for cooldown period
                    self.last_missile_fire_time = self.get_clock().now() - rclpy.duration.Duration(seconds=0.1)  # Just started cooldown
                    self.missile_state = self.MISSILE_COOLDOWN
                    self.get_logger().info("Entering missile cooldown state")
                else:
                    self.missile_state = self.MISSILE_FAILED
        except Exception as e:
            self.get_logger().error(f"Missile service call failed: {e}")
            self.missile_state = self.MISSILE_FAILED
    
    def shutdown_callback(self, signum, frame):
        """Clean shutdown"""
        self.thruster_cmd(0.0, 0.0)
        self.destroy_node()
        rclpy.shutdown()

    def has_drones_within_range(self, max_range):
        """Check if there are any drones within the specified range"""
        if not self.current_detections:
            return False
            
        for target in self.current_detections:
            if target.vehicle_type == Detection.DRONE:
                x = target.relative_position.x
                y = target.relative_position.y
                z = target.relative_position.z
                distance = self.calculate_distance(x, y, z)
                
                if distance <= max_range:
                    return True
                    
        return False

    def check_aiming_timeout(self):
        """Check if aiming is taking too long and cancel if needed"""
        if self.aiming_start_time is None:
            # No active aiming
            self.aiming_timer.cancel()
            self.aiming_timer = None
            return
            
        # Calculate elapsed time
        elapsed = (self.get_clock().now() - self.aiming_start_time).nanoseconds / 1e9
        
        if elapsed > self.aiming_timeout:
            self.get_logger().warn(f"Cannon aiming timeout after {elapsed:.2f}s - updating target position")
            
            # Cancel current goal if we have a handle
            if self.goal_handle:
                self.get_logger().info("Cancelling current aiming goal")
                self.goal_handle.cancel_goal_async()
            
            # Reset for a new targeting attempt
            self.aiming_start_time = None
            self.aiming_timer.cancel()
            self.aiming_timer = None
            
            # Force state reset to look for updated target
            self.missile_state = self.MISSILE_IDLE
            
            # Find new target with updated position
            if self.current_detections:
                self.find_best_target()

def main(args=None):
    rclpy.init(args=args)
    
    boat_client = BoatClient()
    
    try:
        rclpy.spin(boat_client)
    except KeyboardInterrupt:
        print("Program interrupted by user")
    finally:
        boat_client.thruster_cmd(0.0, 0.0)
        rclpy.shutdown()

if __name__ == '__main__':
    main()