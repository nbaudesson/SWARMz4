import rclpy
from rclpy.node import Node
from swarmz_interfaces.srv import Kamikaze
from swarmz_interfaces.srv import Missile
from time import sleep

class KamikazeTestNode(Node):
    def __init__(self):
        super().__init__('kamikaze_test_node')
        self.declare_parameter('robot_name', '/px4_1')
        self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value
        self.kamikaze_client = self.create_client(Kamikaze, 'kamikaze')
        while not self.kamikaze_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Waiting for kamikaze service...')

    def call_kamikaze_service(self):
        self.get_logger().info(f'Requesting kamikaze for {self.robot_name}...')
        request = Kamikaze.Request()
        request.robot_name = self.robot_name
        future = self.kamikaze_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def call_missile_service(self):
        self.missile_client = self.create_client(Missile, 'fire_missile')
        while not self.missile_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Waiting for missile service...')
        request = Missile.Request()
        request.robot_name = '/px4_1'
        self.get_logger().info('Requesting missile launch from /px4_1...')
        future = self.missile_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

def main(args=None):
    rclpy.init(args=args)
    kamikaze_test_node = KamikazeTestNode()

    try:
        for i in range(2, 0, -1):
            kamikaze_test_node.get_logger().info(f'Countdown: {i} seconds remaining')
            sleep(1)
        kamikaze_test_node.call_missile_service()
        # sleep(2)
        # kamikaze_test_node.robot_name = '/px4_2'
        # kamikaze_test_node.call_kamikaze_service()
    except KeyboardInterrupt:
        kamikaze_test_node.get_logger().info('Interrupted by user')
    finally:
        kamikaze_test_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
