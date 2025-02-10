from swarmz_interfaces.srv import Kamikaze, UpdateHealth
import rclpy
from rclpy.node import Node
from utils.tools import get_all_namespaces, get_distance
from utils.gazebo_subscriber import GazeboPosesTracker
import time

class KamikazeServiceServer(Node):

    def __init__(self):
        super().__init__('kamikaze_service_server')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])

        # Declare parameters with default values
        self.declare_parameter('explosion_damage', 100)
        self.declare_parameter('explosion_range', 6.0)

        # Get parameter values
        self.explosion_damage = self.get_parameter('explosion_damage').get_parameter_value().integer_value
        self.explosion_range = self.get_parameter('explosion_range').get_parameter_value().double_value

        # Get list of all namespaces
        self.namespaces = get_all_namespaces(self)
        # Wait until namespaces are detected
        while not self.namespaces:
            self.get_logger().warn("No valid namespaces detected. Waiting...")
            time.sleep(1)
            self.namespaces = get_all_namespaces(self)

        # Print the list of detected robots
        self.get_logger().info(f"Detected robots: {self.namespaces}")

        # Create a GazeboPosesTracker object to track robot positions
        self.gz = GazeboPosesTracker(self.namespaces)
        # self.robots_poses = self.gz.poses

        # Create the UpdateHealth client to communicate with the GameMasterNode
        self.update_health_client = self.create_client(UpdateHealth, 'update_health')
        while not self.update_health_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Waiting for update_health service...')

        # Create the kamikaze service
        self.srv = self.create_service(Kamikaze, 'kamikaze', self.kamikaze_callback)
        self.get_logger().info('KamikazeServiceServer initialized with parameters: '
                        f'explosion_damage={self.explosion_damage}, '
                        f'explosion_range={self.explosion_range}')
    

    def update_health_request(self, robot_name, damage):
        """
        Create a request to update health of a robot.
        :param robot_name: The name of the robot to update health.
        :param damage: The amount of damage to apply.
        :return: The request object.
        """
        request = UpdateHealth.Request()
        request.robot_name = robot_name
        request.damage = damage
        future = self.update_health_client.call_async(request)
        return future

    def kamikaze_callback(self, request, response):
        self.get_logger().info(f'Received kamikaze request from {request.robot_name}')
        """
        Handle the kamikaze service request.
        :param request: The service request containing the robot name.
        :param response: The service response.
        :return: The updated response.
        """
        # Get the namespace of the robot requesting the kamikaze action
        kamikaze_ns = request.robot_name
        if not kamikaze_ns:
            self.get_logger().warn('Robot name is None. Cannot detonate.')
            return response
        if not kamikaze_ns.startswith('/'):
            kamikaze_ns = '/' + kamikaze_ns

        if kamikaze_ns not in self.namespaces:
            self.get_logger().warn(f'{kamikaze_ns} not found in robot list. Cannot detonate.')
            return response

        # Get the position of the kamikaze robot
        kamikaze_pose = self.gz.get_pose(kamikaze_ns)
        kamikaze_position = (kamikaze_pose['position']['x'], kamikaze_pose['position']['y'], kamikaze_pose['position']['z'])
        self.get_logger().info(f'Kamikaze robot {kamikaze_ns} at position {kamikaze_position}')

        # Call the update_health service of the GameMasterNode to apply damage to the kamikaze robot
        try:
            self.update_health_request(kamikaze_ns, self.explosion_damage)
        except Exception as e:
            self.get_logger().error(f'Exception occurred while calling update_health service: {e}')
            return response

        # Check all other robots within the explosion range and apply damage
        for target_ns in self.namespaces:
            if target_ns != kamikaze_ns:
                self.get_logger().info(f'Looping over {target_ns}')
                try:
                    robot_position = self.gz.get_robot_position(target_ns)
                except Exception as e:
                    self.get_logger().error(f'Exception occurred while getting pose of {target_ns}: {e}')
                    continue
                distance = get_distance(kamikaze_position, robot_position)
                self.get_logger().info(f'Checking robot {target_ns} at distance {distance}')
                if distance <= self.explosion_range:
                    self.get_logger().info(f'Robot {target_ns} is within explosion range')
                    # Call the update_health service of the GameMasterNode to apply damage to the kamikaze robot
                    try:
                        self.update_health_request(target_ns, self.explosion_damage)
                    except Exception as e:
                        self.get_logger().error(f'Exception occurred while calling update_health service: {e}')
                        return response
                else:
                    self.get_logger().info(f'Robot {target_ns} is out of explosion range')
        return response

def main(args=None):
    rclpy.init(args=args)
    kamikaze_service_server = KamikazeServiceServer()
    try:
        rclpy.spin(kamikaze_service_server)
    except Exception as e:
        kamikaze_service_server.get_logger().error(f'Exception occurred in kamikaze node {e}')
    finally:
        kamikaze_service_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
