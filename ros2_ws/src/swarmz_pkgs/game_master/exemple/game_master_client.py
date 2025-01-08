import rclpy
from rclpy.node import Node
from swarmz_interfaces.srv import Missile, Kamikaze
import sys
import tty
import termios
import select

class WeaponsController(Node):

    def __init__(self, robot_name):
        super().__init__(f'{robot_name}_controller')
        self.robot_name = robot_name
        self.missile_client = self.create_client(Missile, 'fire_missile')
        
        if 'px4_' in robot_name:
            self.kamikaze_client = self.create_client(Kamikaze, 'kamikaze')
        else:
            self.kamikaze_client = None

        while not self.missile_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for missile service...')
        
        if self.kamikaze_client:
            while not self.kamikaze_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('Waiting for kamikaze service...')

    def fire_missile(self):
        request = Missile.Request()
        request.robot_name = self.robot_name
        future = self.missile_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f'Missile fired by {self.robot_name}, remaining ammo: {future.result().ammo}')
        else:
            self.get_logger().error('Failed to fire missile')

    def perform_kamikaze(self):
        if self.kamikaze_client:
            request = Kamikaze.Request()
            request.robot_name = self.robot_name
            future = self.kamikaze_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                self.get_logger().info(f'Kamikaze performed by {self.robot_name}')
            else:
                self.get_logger().error('Failed to perform kamikaze')
        else:
            self.get_logger().warn(f'{self.robot_name} does not support kamikaze')

    def display_controls(self):
        self.get_logger().info("Controls:")
        self.get_logger().info("M - Fire missile")
        self.get_logger().info("K - Perform kamikaze")
        self.get_logger().info("Q - Quit")

def get_key():
    # Setup terminal for raw input
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        # Check if there's input available
        if select.select([sys.stdin], [], [], 0)[0]:
            key = sys.stdin.read(1)
        else:
            key = ''
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key

def main(args=None):
    rclpy.init(args=args)
    weapons_controller = WeaponsController('px4_1')
    weapons_controller.display_controls()

    try:
        while True:
            # Process ROS callbacks
            rclpy.spin_once(weapons_controller, timeout_sec=0.1)
            
            # Check for keyboard input
            key = get_key()
            
            if key.lower() == 'm':
                weapons_controller.fire_missile()
            elif key.lower() == 'k':
                weapons_controller.perform_kamikaze()
            elif key.lower() == 'q':
                break

    except KeyboardInterrupt:
        pass
    finally:
        weapons_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()