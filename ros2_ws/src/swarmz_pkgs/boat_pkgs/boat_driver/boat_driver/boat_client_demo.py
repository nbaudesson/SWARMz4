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
        self.declare_parameter("tolerance_angle",5.0)
        self.declare_parameter("tolerance_distance",10.0)
        self.node_namespace = self.get_namespace()

        #Publishers for thrusters control
        self.left_thruster_pub = self.create_publisher(Float64, f'{self.node_namespace}/left_propeller_cmd', 10)
        self.right_thruster_pub = self.create_publisher(Float64, f'{self.node_namespace}/right_propeller_cmd', 10)
        #Subscriber to get ship's position
        self.pos_subscriber = self.create_subscription(Pose, f'{self.node_namespace}/localization', self.update_position_callback, 10)
        #Subscriber to get ship's detections
        self.detections_subscriber = self.create_subscription(Detections, f'{self.node_namespace}/detections', self.detections_callback, 10)
        #Subscriber to get ship's health
        self.health_subscriber = self.create_subscription(Int32, f'{self.node_namespace}/health', self.health_callback, 10)
        #Subscriber to get ship's incoming messages
        self.incoming_subscriber = self.create_subscription(String, f'{self.node_namespace}/incoming_messages', self.incoming_callback, 10)
        #Subscriber to get ship's outgoing messages
        self.outgoing_subscriber = self.create_subscription(String, f'{self.node_namespace}/out_going_messages', self.outgoing_callback, 10)

        #Parameters and initial values
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

        # Action client for cannon control
        self.cannon_client = ActionClient(self, Cannon, f'{self.node_namespace}/cannon')

        # Service client for missile
        self.missile_client = self.create_client(Missile, '/game_master/fire_missile')

        # Handle shutdown signal to publish 0 commands and stop the boat
        signal.signal(signal.SIGINT, self.shutdown_callback)
        signal.signal(signal.SIGTERM, self.shutdown_callback)

        self.get_logger().info("Inizialization finished")

    def update_position_callback(self,msg):
        
        #Getting the current pose and yaw from the localization topic
        self.pose_x = msg.position.x
        self.pose_y = msg.position.y
        self.pose_z = msg.position.z
        q = msg.orientation
        quaternion = [q.x, q.y, q.z, q.w]
        roll, pitch, yaw  = tf_transformations.euler_from_quaternion(quaternion)
        self.yaw = yaw*180.0/math.pi
        
    
    def health_callback(self,msg):

        self.health = msg.data
    
    def incoming_callback(self,msg):

        self.incoming_message = msg.data

    def outgoing_callback(self,msg):

        self.out_going_message = msg.data

    def detections_callback(self,msg):
        
        '''The message received contains all the current detections made by the ship in a customized ros2 message,
        to check the entire structure of the message go to swarm interfaces and take a look to Detections message

        The general structure of each detection is the following: 
            - vehicle_type: 0/1
            is_friend: true/false
            relative_position:
                x: 
                y:
                z:

        with this information you can select your target and point your cannon towards it

        The following code do this for a specific case, you should get inspired by this and define your own logic
        
        In this case we are just taking the closest detection, aim our cannon, and shoot to it
        '''
        if not self.assign_target_done:
            posible_targets = []
            for target in msg.detections:
                x = target.relative_position.x
                y = target.relative_position.y
                z = target.relative_position.z
                target_pitch, target_yaw =self.calculate_pitch_yaw(x,y,z)
                distance = self.calculate_distance(self.pose_x - x,self.pose_y - y, self.pose_z - z)
                if abs(target_pitch) < self.cannon_max_pitch and abs(target_yaw) < self.cannon_max_yaw:
                    posible_targets.append([distance,target_pitch,target_yaw])
            
            if posible_targets:
                posible_targets.sort(key=lambda x: x[0])
                self.assign_target_done = True
                if self.fire_missile_done:
                    self.send_goal(posible_targets[0][1],posible_targets[0][2])

    def calculate_distance (self,x,y,z):

        return math.sqrt(math.pow(x,2) + math.pow(y,2) + math.pow(z,2))
    
    def calculate_pitch_yaw(self, x,y,z):
        '''Given the relative position of the target we calculate the pitch and yaw need for the cannon
        to be aligned.
        Lets remember the relative position got by the detection is respect to the base link of the ship 
        and not to the cannon so some transformations are needed to be correctly aligned
        '''
        x = x - self.cannon_x_offset 
        z = z - self.cannon_z_offset
        target_yaw = -math.atan2(y, x) 
        horizontal_distance = math.sqrt(x**2 + y**2)
        target_pitch = math.atan2(z, horizontal_distance) + self.cannon_pitch_offset
        return target_pitch, target_yaw
    
    def go_to(self,x,y):
         
        '''Check if the ship is oriented towards the objective if not we rotate. Once the ship is aligned we move in that direction
        until the ship's position is within the tolerance defined, in any other case we send a 0 command to the
        thrusters
        '''
         
        desired_orientation_yaw = math.atan2((y - self.pose_y),(x - self.pose_x))*180.0/math.pi
        distance_to_target = abs(self.calculate_distance(self.pose_x - x,self.pose_y - y,0))
        if abs(desired_orientation_yaw - self.yaw) > self.get_parameter("tolerance_angle").value:
            self.go_to_angle(desired_orientation_yaw)

        elif distance_to_target > self.get_parameter("tolerance_distance").value:
            
            self.go_to_position(distance_to_target)
                
        else:
            self.get_logger().info(f"Desired position already achieved: current x: {self.pose_x}, current y: {self.pose_y}, current yaw: {self.yaw}")
            self.thruster_cmd(0.0,0.0)

    def go_to_position(self,distance_to_target):
        
        #Given the desired x and y position for the ship we calculate the distance and send the command to the thrusters
        #Here a simple proportional controller is applied but you are free to designed your own controller
        if distance_to_target < 5.0:
            cmd = distance_to_target * 0.6
        else:
            cmd = self.thruster_max_speed
        self.thruster_cmd(cmd,cmd)
    
    def go_to_angle(self, goal):
        #Given the desired angle we calculate the error and send the command to the thrusters
        #Here a simple proportional controller is applied but you are free to designed your own controller
        error = goal - self.yaw
        
        if abs(error) < 20.0:
            cmd = abs(error) * 0.25
        else:
            cmd = self.thruster_max_speed

        if error < 0:
            self.thruster_cmd(cmd,-cmd)
        else:
            self.thruster_cmd(-cmd,cmd)
    
    def thruster_cmd(self,cmd_left,cmd_right):

        #Keep the command within the allowed limits
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
        right_thrust_msg.data =cmd_right
        self.left_thruster_pub.publish(left_thrust_msg)
        self.right_thruster_pub.publish(right_thrust_msg)

    def send_goal(self, pitch, yaw):

        goal_msg = Cannon.Goal()
        goal_msg.pitch = pitch
        goal_msg.yaw = yaw
        self.cannon_client.wait_for_server()
        self.get_logger().info(f'Sending goal: pitch={pitch}, yaw={yaw}')
        self._send_goal_future = self.cannon_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        # self.get_logger().info(f'Feedback: current_pitch={feedback.current_pitch:.2f}, '
        #                        f'current_yaw={feedback.current_yaw:.2f}')

    def goal_response_callback(self, future):

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):

        result = future.result().result
        self.get_logger().info(f'Result: Success = {result.success}')
        while not self.missile_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for missile service...')
        missile_request = Missile.Request()
        missile_request.robot_name = self.node_namespace
        future = self.missile_client.call_async(missile_request)
        # Handle the result asynchronously
        future.add_done_callback(self.handle_missile_response)
        self.fire_missile_done = False

    def handle_missile_response(self, future):

        try:
            response = future.result()
            self.get_logger().info(f"Missile service returned: has_fired: {response.has_fired}, ammo: {response.ammo}")
        except Exception as e:
            self.get_logger().error(f"Missile service call failed: {e}")
    
    def shutdown_callback(self, signum, frame):
        # Publish 0 cmd before shutdown
        self.thruster_cmd(0.0,0.0)
        # Shutdown the node gracefully
        self.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    '''In this demo there will be a drone close to the ship, the ship will detect it, point the
    the cannon towards it and shoot, after that the ship will moveto the center 
    of the spawn position and it will try to keep this position
    '''
    print("Waiting for all the drones to be in position before starting the ship client...")
    
    client = boat_test()
    time.sleep(60)

    while rclpy.ok()and client.fire_missile_done:  
        rclpy.spin_once(client)

    if client.node_namespace == "/flag_ship_1":
        desired_position_x = 50.0
        desired_position_y = 125.0
    else:
        desired_position_x = 450.0
        desired_position_y = 125.0
    
    try:
        while rclpy.ok():  
            rclpy.spin_once(client)
            client.go_to(desired_position_x,desired_position_y)
    except KeyboardInterrupt:
        print("Programme interrompu par l'utilisateur")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()