import rclpy
from rclpy.node import Node
import time
from rclpy.action import ActionClient
from swarmz_interfaces.srv import Kamikaze, Missile
from px4_controllers_interfaces.msg import PointYaw
from geometry_msgs.msg import Point
from px4_controllers_interfaces.action import GotoPosition
from cannon_interfaces.action import Cannon


class MissileKamikazeTest(Node):
    def __init__(self, drone_names):
        super().__init__('gazebo_subscriber_test')
        self.get_logger().info('Starting Gazebo Subscriber Demo')
        
        self.declare_parameter("team_id",1)
        self.team_id = self.get_parameter("team_id").value

        self.action_finished = False
        self.drones = drone_names

        # Service clients
        self.missile_client = self.create_client(Missile, 'fire_missile')
        self.kamikaze_client = self.create_client(Kamikaze, 'kamikaze')

        # Action client for the cannon control 
        self._cannon_action_client = ActionClient(self, Cannon, 'cannon') 
        
        # Action clients px4 control 
        self._drones_action_clients = {}
        self.goal_handles = {}
        for drone in self.drones:
            self._drones_action_clients[drone] = ActionClient(self, GotoPosition, f'{drone}/goto_position')

        while not self.missile_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Missile service not available, waiting...')

        while not self.kamikaze_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Kamikaze service not available, waiting...')

        self.missile_req = Missile.Request()
        self.kamikaze_req = Kamikaze.Request()

        while not  self._cannon_action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn(f'Cannon Action server not available, waiting...')
            return
        
        while not self._drones_action_clients[drone].wait_for_server(timeout_sec=1.0):
            self.get_logger().warn(f'Drone Action server not available for {drone}, waiting...')
            return
        
    def send_drone_movement(self, drone, target):
        goal_msg = GotoPosition.Goal()
        goal_msg.target = target

        if not self._drones_action_clients[drone].wait_for_server(timeout_sec=5.0):
            self.get_logger().warn(f'Drone Action server not available for {drone}')
            return

        send_goal_future = self._drones_action_clients[drone].send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        send_goal_future.add_done_callback(lambda future: self.goal_response_callback(future, drone))

    def send_cannon_goal(self, pitch, yaw, target_ship):

        goal_msg = Cannon.Goal()
        goal_msg.pitch = pitch
        goal_msg.yaw = yaw
        goal_msg.target_ship = target_ship

        if not  self._cannon_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn(f'Cannon Action server not available')
            return

        self.get_logger().info(f'Sending goal: pitch={pitch}, yaw={yaw}, ship={target_ship}')
        self._send_goal_future = self._cannon_action_client.send_goal_async(
            goal_msg,
        )

        # self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future, drone):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info(f'Goal for {drone} rejected')
            return

        self.get_logger().info(f'Goal for {drone} accepted')
        self.goal_handles[drone] = goal_handle

        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Action completed: success = {result.success}')
        self.action_finished = True

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Feedback -> Distance to target: {feedback.distance_to_target:.2f}m, Time elapsed: {feedback.time_elapsed:.2f}s'
        )

    def send_missile_request(self, robot_name):
        self.missile_req.robot_name = robot_name
        future = self.missile_client.call_async(self.missile_req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            return future.result()
        else:
            self.get_logger().error("Missile service call failed.")
            return None

    def send_kamikaze_request(self, robot_name):
        self.kamikaze_req.robot_name = robot_name
        future = self.kamikaze_client.call_async(self.kamikaze_req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            return future.result()
        else:
            self.get_logger().error("Kamikaze service call failed.")
            return None


def main(args=None):
    rclpy.init(args=args)

    drone_names = ["/px4_0", "/px4_1", "/px4_2", "/px4_3", "/px4_4"]
    test = MissileKamikazeTest(drone_names)

    # Flag ship cannon action 
    team_id = test.get_parameter("team_id").value
    goal = {'pitch': 1.56, 'yaw': 0.0, 'target_ship': f'flag_ship_{team_id}'}
    test.send_cannon_goal(goal['pitch'], goal['yaw'], goal['target_ship'])

    # Define different target positions for each drone
    target_positions = {
        "/px4_0": PointYaw(position=Point(x=50.0, y=50.0, z=-3.0), yaw=90.0),
        "/px4_1": PointYaw(position=Point(x=50.0, y=52.0, z=-3.0), yaw=0.0),
        "/px4_2": PointYaw(position=Point(x=-8.0, y=0.0, z= 3.0), yaw=0.0),
        "/px4_3": PointYaw(position=Point(x=52.0, y=52.0, z=-3.0), yaw=-90.0),
        "/px4_4": PointYaw(position=Point(x=52.0, y=50.0, z=-3.0), yaw=180.0),
    }

    # Send movement commands to each drone with its unique target position
    for drone_name, target in target_positions.items():
        test.send_drone_movement(drone_name, target)
     
    try:
        while rclpy.ok() and not test.action_finished:
            rclpy.spin_once(test)

        # Waiting for the drones to stabilized
        time.sleep(5.0)

        # Once the movement action is completed, call the missile service for specific robot
        shooters = ["/px4_0", "/px4_1", "/px4_2", "/px4_3", "/px4_4", "/flag_ship_1"]
        for robot_name in shooters:
            response_missile = test.send_missile_request(robot_name)
            if response_missile:
                test.get_logger().info(
                    f'Result of the missile launch from {robot_name}: Fired={response_missile.has_fired}, Remaining ammo={response_missile.ammo}'
                )
            else:
                test.get_logger().error(f'Missile request failed for {robot_name}')
        
        # Once the movement action is completed, call the kamikaze service for specific robot
        kamikaze = ["/px4_2"]
        for robot_name in kamikaze:
            response_kamikaze = test.send_kamikaze_request(robot_name)
            if response_kamikaze:
                test.get_logger().info(
                    f'{robot_name} exploded'
                )
            else:
                test.get_logger().error(f'Kamikaze request failed for {robot_name}')

    except KeyboardInterrupt:
        print("Program interrupted by the user")
    finally:
        test.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
