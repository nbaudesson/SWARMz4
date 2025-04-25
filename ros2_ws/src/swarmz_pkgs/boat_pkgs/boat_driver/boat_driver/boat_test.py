#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose
import math
import signal
from rclpy.action import ActionClient
from cannon_interfaces.action import Cannon
from scipy.spatial.transform import Rotation as R

class boat_test(Node):

    def __init__(self):
        super().__init__('boat_test')
        self.declare_parameter("team_id",1)
        self.declare_parameter("tolerance_angle",1.0)
        self.declare_parameter("tolerance_distance",5.0)
        self.team_id = self.get_parameter("team_id").value
        #Publishers for thruster control
        self.left_thruster_pub = self.create_publisher(Float64, f'/model/flag_ship_{self.team_id}/joint/left_engine_propeller_joint/cmd_thrust', 10)
        self.right_thruster_pub = self.create_publisher(Float64, f'/model/flag_ship_{self.team_id}/joint/right_engine_propeller_joint/cmd_thrust', 10)
        #Subscriber to get ship's position
        self.pos_subscriber = self.create_subscription(Pose, f'/flag_ship_{self.team_id}/pose', self.update_position_callback, 10)
        #Parameters and initial values
        self.min_speed = -2.5
        self.max_speed = 2.5
        self.pose_x = None
        self.pose_y = None
        self.yaw = None
        self.target_x = None
        self.target_y = None
        self.target_yaw = None
        self._done = True
        # action client for cannon control
        self._action_client = ActionClient(self, Cannon, 'cannon') 
        # Goal position ship
        self.goal_position = [50.0,50.0,90.0] #x,y,yaw
        self.get_logger().info("Launching ship client example")

        self.timer = self.create_timer(0.05, self.control)

        # Handle shutdown signal to publish 0 and stop the boat
        signal.signal(signal.SIGINT, self.shutdown_callback)
        signal.signal(signal.SIGTERM, self.shutdown_callback)

    def update_position_callback(self,msg):

        self.pose_x = msg.position.x
        self.pose_y = msg.position.y
        q = msg.orientation
        quaternion = [q.x, q.y, q.z, q.w]
        rotation = R.from_quat(quaternion)  # Convertit le quaternion en un objet de rotation
        self.yaw = rotation.as_euler('xyz', degrees=True)[2]
        #print(f'yaw: {self.yaw}')
        # self.pose.orientation.x = msg.orientation.x
        # self.pose.orientation.y = msg.orientation.y
        # self.pose.orientation.z = msg.orientation.z
        # self.pose.orientation.w = msg.orientation.w

    def control(self):

        if self.target_yaw is not None and abs(self.target_yaw - self.yaw) > self.get_parameter("tolerance_angle").value:
            self.go_to_angle(self.target_yaw)
        elif self.target_x is not None and self.target_y is not None and (abs(self.calculate_distance(self.target_x, self.target_y)) > self.get_parameter("tolerance_distance").value):
            self.go_to_position(self.target_x, self.target_y)
        else:
            self.thruster_cmd(0.0,0.0)
    
    def calculate_distance(self, target_x, target_y):

        return math.sqrt(math.pow((self.pose_x - target_x),2) + math.pow((self.pose_y - target_y),2))
    
    def go_to_position(self, goal_x, goal_y):

        target_distance = self.calculate_distance(goal_x,goal_y)

        if target_distance < 5.0:
            cmd = target_distance * 0.5
        else:
            cmd = self.max_speed

        self.thruster_cmd(cmd,cmd)
    
    def go_to_angle(self, goal):

        error = goal - self.yaw

        if abs(error) < 20.0:
            cmd = abs(error) * 0.15
        else:
            cmd = self.max_speed

        if error < 0:
            self.thruster_cmd(cmd,-cmd)
        else:
            self.thruster_cmd(-cmd,cmd)

    def thruster_cmd(self,cmd_left,cmd_right):

        if cmd_left > self.max_speed:
            cmd_left = self.max_speed
        elif cmd_left < self.min_speed:
            cmd_left = self.min_speed
        
        if cmd_right > self.max_speed:
            cmd_right = self.max_speed
        elif cmd_right < self.min_speed:
            cmd_right = self.min_speed

        left_thrust_msg = Float64()
        right_thrust_msg = Float64()
        left_thrust_msg.data = cmd_left
        right_thrust_msg.data =cmd_right
        self.left_thruster_pub.publish(left_thrust_msg)
        self.right_thruster_pub.publish(right_thrust_msg)
        #self.get_logger().info(f'Publishing thrust command: left:{left_thrust_msg.data}   right:{right_thrust_msg.data}')

    def shutdown_callback(self, signum, frame):
        # Create and publish a message with 0 value before shutdown
        self.thruster_cmd(0.0,0.0)
        self.get_logger().info('Publishing: 0.0 before shutdown')
        # Shutdown the node gracefully
        self.destroy_node()
        rclpy.shutdown()
    
    def send_goal(self, pitch, yaw, target_ship):

        goal_msg = Cannon.Goal()
        goal_msg.pitch = pitch
        goal_msg.yaw = yaw
        goal_msg.target_ship = target_ship

        self._action_client.wait_for_server()

        self.get_logger().info(f'Sending goal: pitch={pitch}, yaw={yaw}, ship={target_ship}')
        self._send_goal_future = self._action_client.send_goal_async(
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
        self._done = True


def main(args=None):
    rclpy.init(args=args)
    client = boat_test()
    team_id = client.get_parameter("team_id").value
    
    # List of 4 demo goals
    goals = [
            {'pitch': 1.0, 'yaw': -2.13, 'target_ship': f'flag_ship_{team_id}'},
            {'pitch': 0.5, 'yaw': 1.0, 'target_ship': f'flag_ship_{team_id}'},
            {'pitch': -1.0, 'yaw': -1.0, 'target_ship': f'flag_ship_{team_id}'},
            {'pitch': 0.0, 'yaw': 0.0, 'target_ship': f'flag_ship_{team_id}'}
        ]

    for goal in goals:
        client._done = False
        client.send_goal(goal['pitch'], goal['yaw'], goal['target_ship'])

        # Wait until the goal is done before sending the next one
        while not client._done:
            rclpy.spin_once(client)
    if client.team_id==1:
        client.target_x = 50.0
        client.target_y = 125.0
    else:
        client.target_x = 450.0
        client.target_y = 125.0

    client.target_yaw = math.atan2((client.target_y - client.pose_y),(client.target_x - client.pose_x))*180.0/math.pi
    print(f'target yaw: {client.target_yaw}')
    try:
        while rclpy.ok():  
            rclpy.spin_once(client)

    except KeyboardInterrupt:
        print("Programme interrompu par l'utilisateur")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()