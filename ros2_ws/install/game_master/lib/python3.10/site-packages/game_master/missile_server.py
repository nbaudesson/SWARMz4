from swarmz_interfaces.srv import Missile, UpdateHealth
import rclpy
from rclpy.node import Node
from utils.tools import get_all_namespaces, get_distance, is_aligned
from utils.gazebo_subscriber import GazeboPosesTracker
import time

class MissileServiceServer(Node):

    def __init__(self):
        super().__init__('missile_service_server')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])

        # Declare parameters with default values
        self.declare_parameter('drone_missile_range', 100)
        self.declare_parameter('ship_missile_range', 200)
        self.declare_parameter('drone_missile_damage', 10)
        self.declare_parameter('ship_missile_damage', 20)
        self.declare_parameter('drone_cooldown', 1.0)
        self.declare_parameter('ship_cooldown', 2.0)
        self.declare_parameter('drone_magazine', 5)
        self.declare_parameter('ship_magazine', 10)
        self.declare_parameter('laser_width', 0.1)
        self.declare_parameter('laser_length', 1.0)

        self.declare_parameter('drone_padding_x', 0.5)
        self.declare_parameter('drone_padding_y', 0.5)
        self.declare_parameter('drone_padding_z', 0.5)
        self.declare_parameter('ship_padding_x', 6.0)
        self.declare_parameter('ship_padding_y', 1.0)
        self.declare_parameter('ship_padding_z', 1.0)

        self.drone_padding_x = self.get_parameter('drone_padding_x').get_parameter_value().double_value
        self.drone_padding_y = self.get_parameter('drone_padding_y').get_parameter_value().double_value
        self.drone_padding_z = self.get_parameter('drone_padding_z').get_parameter_value().double_value
        self.ship_padding_x = self.get_parameter('ship_padding_x').get_parameter_value().double_value
        self.ship_padding_y = self.get_parameter('ship_padding_y').get_parameter_value().double_value
        self.ship_padding_z = self.get_parameter('ship_padding_z').get_parameter_value().double_value

        # Keep track of all robots magazines
        # 1. Get list of all namespaces
        namespaces = get_all_namespaces(self)
        # 2. Make a dictionary of namespace : drone/ship_magazine
        self.magazines = {ns: self.get_parameter('drone_magazine').get_parameter_value().integer_value if 'drone' in ns else self.get_parameter('ship_magazine').get_parameter_value().integer_value for ns in namespaces}
        
        # Dictionary to track the last fire timestamp for each robot
        self.last_fire_time = {ns: 0 for ns in namespaces}

        # Create the service
        self.srv = self.create_service(Missile, 'fire_missile', self.fire_missile_callback)

    def fire_missile_callback(self, request, response):
        """
        Handle the missile firing request.
        :param request: The service request containing the robot name.
        :param response: The service response indicating if the missile was fired and the remaining ammo.
        :return: The updated response.
        """
        # 1. Get shooter namespace
        shooter_ns = request.robot_name

        # 2. Check if corresponding namespace in the magazine dictionary has a value > 0
        current_time = time.time()
        cooldown = self.get_parameter('drone_cooldown').get_parameter_value().double_value if 'drone' in shooter_ns else self.get_parameter('ship_cooldown').get_parameter_value().double_value
        if self.magazines[shooter_ns] > 0 and (current_time - self.last_fire_time[shooter_ns]) >= cooldown:
            # Decrease magazine count
            self.magazines[shooter_ns] -= 1
            response.has_fired = True
            response.ammo = self.magazines[shooter_ns]
            # Update last fire time
            self.last_fire_time[shooter_ns] = current_time
        else:
            response.has_fired = False
            response.ammo = self.magazines[shooter_ns]
            return response

        # 3. Get shooter position and orientation
        gz = GazeboPosesTracker([shooter_ns])
        shooter_pose = gz.get_pose(shooter_ns)
        shooter_position = (shooter_pose['position']['x'], shooter_pose['position']['y'], shooter_pose['position']['z'])
        shooter_orientation = (shooter_pose['orientation']['x'], shooter_pose['orientation']['y'], shooter_pose['orientation']['z'], shooter_pose['orientation']['w'])

        # 4. Out of all robots in gazebo get list of ID's, distance and positions of those that are within radius of missile range
        all_robots = get_all_namespaces(self)
        all_robots.remove(shooter_ns)  # Remove the shooter from the list
        missile_range = self.get_parameter('drone_missile_range').get_parameter_value().integer_value if 'drone' in shooter_ns else self.get_parameter('ship_missile_range').get_parameter_value().integer_value
        targets_in_range = []
        for robot in all_robots:
            robot_pose = gz.get_pose(robot)
            robot_position = (robot_pose['position']['x'], robot_pose['position']['y'], robot_pose['position']['z'])
            distance = get_distance(shooter_position, robot_position)
            if distance <= missile_range:
                targets_in_range.append((robot, distance, robot_position))

        # 5. Out of those robots remove those that are not aligned with shooter's orientation, using laser_width as threshold
        laser_width = self.get_parameter('laser_width').get_parameter_value().double_value
        aligned_targets = []
        for target in targets_in_range:
            target_id = target[0]
            target_position = target[2]
            if 'drone' in target_id:
                target_padding = (self.drone_padding_x, self.drone_padding_y, self.drone_padding_z)
            else:
                target_padding = (self.ship_padding_x, self.ship_padding_y, self.ship_padding_z)
            if is_aligned(self, shooter_position, shooter_orientation, target_position, target_padding, laser_width):
                aligned_targets.append(target)

        # 6. For the element of the list with the smallest distance, send ID and damage to game_master
        if aligned_targets:
            target = min(aligned_targets, key=lambda x: x[1])
            target_id = target[0]
            damage = self.get_parameter('drone_missile_damage').get_parameter_value().integer_value if 'drone' in shooter_ns else self.get_parameter('ship_missile_damage').get_parameter_value().integer_value
            
            # Call the update_health service of the GameMasterNode
            client = self.create_client(UpdateHealth, 'update_health')
            if client.service_is_ready():
                # Create a request to update health
                update_request = UpdateHealth.Request()
                update_request.robot_name = target_id
                update_request.damage = damage
                # Call the service asynchronously
                future = client.call_async(update_request)
                rclpy.spin_until_future_complete(self, future)
                if future.result() is not None:
                    # Log success message
                    self.get_logger().info(f'Successfully shot {target_id} giving {damage} damage')
                else:
                    # Log error message
                    self.get_logger().error(f'Failed to update health of {target_id}')
            else:
                # Log error message if service is not available
                self.get_logger().error('update_health service is not available')

        # 8. return response has_fired = true and ammo = namespace.magazine
        return response

def main(args=None):
    rclpy.init(args=args)
    missile_service_server = MissileServiceServer()
    try:
        rclpy.spin(missile_service_server)
    except KeyboardInterrupt:
        pass
    finally:
        missile_service_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()