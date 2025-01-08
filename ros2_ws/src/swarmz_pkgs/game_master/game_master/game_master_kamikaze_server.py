from swarmz_interfaces.srv import Kamikaze, UpdateHealth
import rclpy
from rclpy.node import Node
from game_master.utils.tools import get_all_namespaces, get_distance
from game_master.utils.gazebo_subscriber import GazeboPosesTracker

class KamikazeServiceServer(Node):

    def __init__(self):
        super().__init__('kamikaze_service_server')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])

        # Declare parameters with default values
        self.declare_parameter('explosion_damage', 50)
        self.declare_parameter('explosion_range', 2.5)

        self.explosion_damage = self.get_parameter('explosion_damage').get_parameter_value().integer_value
        self.explosion_range = self.get_parameter('explosion_range').get_parameter_value().double_value

        # Get list of all namespaces
        self.namespaces = get_all_namespaces(self)

        # Create the service
        self.srv = self.create_service(Kamikaze, 'kamikaze', self.kamikaze_callback)

    def kamikaze_callback(self, request, response):
        """
        Handle the kamikaze service request.
        :param request: The service request containing the robot name.
        :param response: The service response.
        :return: The updated response.
        """
        # Get the namespace of the robot requesting the kamikaze action
        kamikaze_ns = request.robot_name

        # Get the position of the kamikaze robot
        gz = GazeboPosesTracker([kamikaze_ns])
        kamikaze_pose = gz.get_pose(kamikaze_ns)
        kamikaze_position = (kamikaze_pose['position']['x'], kamikaze_pose['position']['y'], kamikaze_pose['position']['z'])

        # Update the health points of the kamikaze robot to zero
        self.update_health(kamikaze_ns, 0)

        # Check all other robots within the explosion range and apply damage
        for target_ns in self.namespaces:
            if target_ns != kamikaze_ns:
                robot_pose = gz.get_pose(target_ns)
                robot_position = (robot_pose['position']['x'], robot_pose['position']['y'], robot_pose['position']['z'])
                distance = get_distance(kamikaze_position, robot_position)
                if distance <= self.explosion_range:
                    # Call the update_health service of the GameMasterNode
                    client = self.create_client(UpdateHealth, 'update_health')
                    if client.service_is_ready():
                        # Create a request to update health
                        update_request = UpdateHealth.Request()
                        update_request.robot_name = target_ns
                        update_request.damage = self.explosion_damage
                        # Call the service asynchronously
                        future = client.call_async(update_request)
                        rclpy.spin_until_future_complete(self, future)
                        if future.result() is not None:
                            # Log success message
                            self.get_logger().info(f'Successfully damaged {target_ns} with {self.explosion_damage} damage')
                        else:
                            # Log error message
                            self.get_logger().error(f'Failed to update health of {target_ns}')
                    else:
                        # Log error message if service is not available
                        self.get_logger().error('update_health service is not available')

        return response

def main(args=None):
    rclpy.init(args=args)
    kamikaze_service_server = KamikazeServiceServer()
    try:
        rclpy.spin(kamikaze_service_server)
    except KeyboardInterrupt:
        pass
    finally:
        kamikaze_service_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
