import rclpy
from rclpy.node import Node
from swarmz_interfaces.srv import Kamikaze
from swarmz_interfaces.srv import Missile
from std_msgs.msg import String
from swarmz_interfaces.msg import Detections
from time import sleep, time

class TestNode(Node):
    def __init__(self):
        super().__init__('test_node')
        self.declare_parameter('robot_name', '/px4_1')
        self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value
        self.kamikaze_client = self.create_client(Kamikaze, 'kamikaze')
        while not self.kamikaze_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Waiting for kamikaze service...')

        # Change publisher topic to incoming_messages
        self.msg_publisher = self.create_publisher(String, '/px4_1/incoming_messages', 10)
        self.msg_counter = 0
        self.msg_timer = self.create_timer(2.0, self.publish_message)

        # Subscribe to all robots' out_going_messages
        self.create_subscription(
            String,
            '/px4_2/out_going_messages',
            lambda msg: self.incoming_message_callback(msg, '/px4_2'),
            10
        )
        self.create_subscription(
            String,
            '/px4_3/out_going_messages',
            lambda msg: self.incoming_message_callback(msg, '/px4_3'),
            10
        )
        self.create_subscription(
            String,
            '/px4_4/out_going_messages',
            lambda msg: self.incoming_message_callback(msg, '/px4_4'),
            10
        )

        # Add subscriber for detections
        self.create_subscription(
            Detections,
            '/px4_4/detections',
            self.detections_callback,
            10
        )

        # Add sequence control
        self.start_time = time()
        self.phase = 'publishing'  # phases: publishing, missile, kamikaze
        self.sequence_timer = self.create_timer(0.1, self.sequence_manager)
        self.missile_fired = False
        self.kamikaze_fired = False
        self.service_future = None
        self.waiting_for_service = False

    def sequence_manager(self):
        """Manage the test sequence"""
        elapsed_time = time() - self.start_time

        # Handle ongoing service calls
        if self.waiting_for_service and self.service_future:
            if self.service_future.done():
                self.waiting_for_service = False
                self.service_future = None
                if self.phase == 'missile_countdown':
                    self.phase = 'kamikaze_countdown'
                    self.countdown_start = time()
                elif self.phase == 'kamikaze_countdown':
                    self.phase = 'complete'
            return

        if self.phase == 'publishing' and elapsed_time >= 10.0:
            self.phase = 'missile_countdown'
            self.countdown_start = time()
            
        elif self.phase == 'missile_countdown':
            countdown_elapsed = time() - self.countdown_start
            remaining = 2.0 - countdown_elapsed
            if remaining > 0:
                self.get_logger().info(f'Missile countdown: {int(remaining)} seconds')
            elif not self.missile_fired and not self.waiting_for_service:
                self.get_logger().info('Firing missile...')
                self.service_future = self.call_missile_service_async()
                self.missile_fired = True
                self.waiting_for_service = True

        elif self.phase == 'kamikaze_countdown':
            countdown_elapsed = time() - self.countdown_start
            remaining = 2.0 - countdown_elapsed
            if remaining > 0:
                self.get_logger().info(f'Kamikaze countdown: {int(remaining)} seconds')
            elif not self.kamikaze_fired and not self.waiting_for_service:
                self.get_logger().info('Initiating kamikaze...')
                self.robot_name = '/px4_2'
                self.service_future = self.call_kamikaze_service_async()
                self.kamikaze_fired = True
                self.waiting_for_service = True

    def call_missile_service_async(self):
        """Asynchronous missile service call"""
        self.missile_client = self.create_client(Missile, 'fire_missile')
        while not self.missile_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Waiting for missile service...')
        request = Missile.Request()
        request.robot_name = '/px4_1'
        return self.missile_client.call_async(request)

    def call_kamikaze_service_async(self):
        """Asynchronous kamikaze service call"""
        request = Kamikaze.Request()
        request.robot_name = self.robot_name
        return self.kamikaze_client.call_async(request)

    def publish_message(self):
        """Publish an incrementing message"""
        if self.phase != 'complete':  # Keep publishing until sequence is complete
            msg = String()
            msg.data = str(self.msg_counter)
            self.msg_publisher.publish(msg)
            self.get_logger().info(f'Published message to /px4_1/incoming_messages: {msg.data}')
            self.msg_counter += 1

    def incoming_message_callback(self, msg, receiver_ns):
        """Handle incoming messages"""
        self.get_logger().info(f'Received message on {receiver_ns}/out_going_messages: {msg.data}')

    def detections_callback(self, msg):
        """Handle detection messages"""
        self.get_logger().info(f'Received detections on /px4_4/detections:')
        for i, detection in enumerate(msg.detections):
            self.get_logger().info(f'  Detection {i+1}:')
            self.get_logger().info(f'    Type: {"Drone" if detection.vehicle_type == 0 else "Ship"}')
            self.get_logger().info(f'    Is friend: {detection.is_friend}')
            self.get_logger().info(f'    Position: x={detection.relative_position.position.x:.2f}, '
                                 f'y={detection.relative_position.position.y:.2f}, '
                                 f'z={detection.relative_position.position.z:.2f}')

def main(args=None):
    rclpy.init(args=args)
    test_node = TestNode()

    try:
        rclpy.spin(test_node)
    except KeyboardInterrupt:
        test_node.get_logger().info('Interrupted by user')
    finally:
        test_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
