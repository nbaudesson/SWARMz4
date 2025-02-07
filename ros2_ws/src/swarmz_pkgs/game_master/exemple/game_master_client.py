import rclpy
from rclpy.node import Node
from swarmz_interfaces.srv import Missile, Kamikaze
import sys
import tty
import termios
import select

settings = termios.tcgetattr(sys.stdin)

class WeaponsController(Node):

    def __init__(self, robot_name):
        super().__init__(f'{robot_name}_controller')
        self.robot_name = robot_name

        self.missile_client = self.create_client(Missile, 'fire_missile')
        
        if 'px4_' in self.robot_name:
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
        self.get_logger().info('Requesting kamikaze...')
        if self.kamikaze_client:
            self.get_logger().info(f'Performing kamikaze with {self.robot_name}')
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

def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key

msg = """
Reading from the keyboard and activating ROS2 services!
---------------------------
m - Fire missile
k - Perform kamikaze
q - Quit
"""

def main(args=None):
    rclpy.init(args=args)
    
    node = rclpy.create_node('temp_node')
    robot_name = node.declare_parameter('robot_name', '').get_parameter_value().string_value
    node.destroy_node()

    if not robot_name:
        print("Usage: ros2 run game_master game_master_client_node --ros-args -p robot_name:=<robot_name>")
        return

    weapons_controller = WeaponsController(robot_name)
    print(msg)

    try:
        while True:
            # Process ROS callbacks
            rclpy.spin_once(weapons_controller, timeout_sec=1)
            
            # Check for keyboard input
            key = getKey()
            
            if key.lower() == 'm':
                weapons_controller.fire_missile()
            elif key.lower() == 'k':
                weapons_controller.perform_kamikaze()
            elif key.lower() == 'q':
                break

    except KeyboardInterrupt:
        from time import sleep
        sleep(20)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        weapons_controller.destroy_node()
        rclpy.shutdown()
    finally:
        from time import sleep
        sleep(20)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        weapons_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
