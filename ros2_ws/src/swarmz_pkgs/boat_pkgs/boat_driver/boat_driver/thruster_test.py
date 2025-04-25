#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import random
import signal

class Thruster_pub_test(Node):

    def __init__(self):
        super().__init__('thruster_pub_test')
        self.left_thruster_pub1 = self.create_publisher(Float64, '/model/flag_ship_1/joint/left_engine_propeller_joint/cmd_thrust', 10)
        self.right_thruster_pub1 = self.create_publisher(Float64, '/model/flag_ship_1/joint/right_engine_propeller_joint/cmd_thrust', 10)
        self.left_thruster_pub2 = self.create_publisher(Float64, '/model/flag_ship_2/joint/left_engine_propeller_joint/cmd_thrust', 10)
        self.right_thruster_pub2 = self.create_publisher(Float64, '/model/flag_ship_2/joint/right_engine_propeller_joint/cmd_thrust', 10)
        self.get_logger().info("Launching thruster test")
        self.timer = self.create_timer(10, self.thruster_callback)
        self.min_speed = -2.5
        self.max_speed = 2.5

        # Handle shutdown signal to publish 0 and stop the boat
        signal.signal(signal.SIGINT, self.shutdown_callback)
        signal.signal(signal.SIGTERM, self.shutdown_callback)
    def thruster_callback(self):

        # Creation of thrust message
        left_thrust_msg = Float64()
        left_thrust_msg.data = random.uniform(self.min_speed, self.max_speed) # random value within the range of the minimum and maximum boat's speed
        right_thrust_msg = Float64()
        right_thrust_msg.data = random.uniform(self.min_speed, self.max_speed) # random value within the range of the minimum and maximum boat's speed
        # Publish thrust command
        self.left_thruster_pub1.publish(left_thrust_msg)
        self.right_thruster_pub1.publish(right_thrust_msg)
        self.left_thruster_pub2.publish(left_thrust_msg)
        self.right_thruster_pub2.publish(right_thrust_msg)
        self.get_logger().info(f'Publishing thrust command: left:{left_thrust_msg.data}   right:{right_thrust_msg.data}')

    def shutdown_callback(self, signum, frame):
        # Create and publish a message with 0 value before shutdown
        msg = Float64()
        msg.data = 0.0
        self.left_thruster_pub1.publish(msg)
        self.right_thruster_pub1.publish(msg)
        self.left_thruster_pub2.publish(msg)
        self.right_thruster_pub2.publish(msg)
        self.get_logger().info('Publishing: 0.0 before shutdown')

        # Shutdown the node gracefully
        self.destroy_node()
        rclpy.shutdown()
    

def main(args=None):
    rclpy.init(args=args)
    thrust_test= Thruster_pub_test()

    # Register the shutdown process
    try:
        rclpy.spin(thrust_test)
    except KeyboardInterrupt:
        print("Programme interrompu par l'utilisateur")
    finally:
        thrust_test.destroy_node()

if __name__ == '__main__':
    main()