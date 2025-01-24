import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from tf_transformations import quaternion_matrix

from utils.gazebo_subscriber import GazeboPosesTracker

def get_distance(tf1, tf2):
    """
    Compute 3D distance between two transforms (x, y, z).
    :param tf1: tuple (x1, y1, z1)
    :param tf2: tuple (x2, y2, z2)
    :return: Euclidean distance
    """
    x1, y1, z1 = tf1
    x2, y2, z2 = tf2
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2 + (z2 - z1)**2)

def get_laser_distance(node, robot_name, laser_topic="laser_beam", timeout_sec=1.0):
    """
    Retrieve laser scan distance for a robot from its topic.
    :param node: The rclpy node
    :param robot_name: The robot's name (assumes unique topic per robot)
    :param laser_topic: The laser topic to subscribe to
    :param timeout_sec: Timeout for spinning to get laser data
    :return: Minimum distance from laser scan or None if no data
    """

    logger = rclpy.logging.get_logger('get_laser_distance')

    def laser_callback(msg):
        nonlocal distance
        if msg.ranges:
            distance = min(r for r in msg.ranges if not math.isinf(r) and not math.isnan(r))  # Filter out inf/nan
        else:
            logger.warn("Received LaserScan with empty ranges")
            distance = float('inf')

    distance = None
    qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.VOLATILE)
    sub = node.create_subscription(LaserScan, f'/{robot_name}/{laser_topic}', laser_callback, qos)
    rclpy.spin_once(node, timeout_sec=timeout_sec)  # Spin to allow data processing

    node.destroy_subscription(sub)  # Clean up subscription
    if distance is None:
        logger.warn("No LaserScan data received within timeout")
    return distance

def get_all_robots(node):
    """
    Get a list of all robot names in the ROS 2 system.
    :param node: The rclpy node
    :return: List of robot names
    """
    all_nodes = node.get_node_names_and_namespaces()
    return [name for name, _ in all_nodes if 'robot' in name]

def get_all_namespaces(node):
    """
    Get a list of all unique namespaces in the ROS 2 system through topics.
    :param node: The rclpy node
    :return: List of unique namespaces
    """
    topic_names_and_types = node.get_topic_names_and_types()
    namespaces = set()
    for topic_name, _ in topic_names_and_types:
        namespace = topic_name.split('/')[1]
        if namespace and ('px4' in namespace or 'ship' in namespace):
            namespaces.add(f'/{namespace}')
    return list(namespaces)

def get_all_drones(node):
    """
    Get all PX4 drones running in the ROS 2 system through topics.
    :param node: The rclpy node
    :return: List of PX4 drone names
    """
    topic_names_and_types = node.get_topic_names_and_types()
    drones = set()
    for topic_name, _ in topic_names_and_types:
        if 'px4' in topic_name:
            namespace = topic_name.split('/')[1]
            if namespace:
                drones.add(f'/{namespace}')
    return list(drones)

def get_all_ships(node):
    """
    Get all ship robots running in the ROS 2 system through topics.
    :param node: The rclpy node
    :return: List of ship robot names
    """
    topic_names_and_types = node.get_topic_names_and_types()
    ships = set()
    for topic_name, _ in topic_names_and_types:
        if 'ship' in topic_name:
            namespace = topic_name.split('/')[1]
            if namespace:
                ships.add(f'/{namespace}')
    return list(ships)

def get_relative_position(shooter_position, target_position):
    """
    Calculate the relative position of the target with respect to the shooter.
    This function simply calculates the difference in positions without considering orientation.
    :param shooter_position: Tuple (x, y, z) representing the shooter's position.
    :param target_position: Tuple (x, y, z) representing the target's position.
    :return: Dictionary containing the relative position.
    """
    relative_position = {
        "x": target_position[0] - shooter_position[0],
        "y": target_position[1] - shooter_position[1],
        "z": target_position[2] - shooter_position[2]
    }
    return relative_position

def get_relative_position_with_orientation(shooter_position, shooter_orientation, target_position):
    """
    Transform the target position to the shooter's reference frame considering orientation.
    This function calculates the relative position and transforms it based on the shooter's orientation.
    :param shooter_position: Tuple (x, y, z) representing the shooter's position.
    :param shooter_orientation: Tuple (x, y, z, w) representing the shooter's orientation (quaternion).
    :param target_position: Tuple (x, y, z) representing the target's position.
    :return: Tuple (x, y, z) representing the transformed position in the shooter's reference frame.
    """
    # Convert quaternion to rotation matrix
    rotation_matrix = quaternion_matrix(shooter_orientation)[:3, :3]

    # Calculate the direction vector from shooter to target
    relative_position = [target_position[i] - shooter_position[i] for i in range(3)]

    # Transform the relative position vector to the shooter's reference frame
    transformed_position = rotation_matrix.T.dot(relative_position)

    return tuple(transformed_position)

def is_aligned(node, shooter_position, shooter_orientation, target_position, target_padding, threshold, verbose=False):
    """
    Check if a shooter's X-axis points toward a target position within a given threshold.
    :param node: The rclpy node
    :param shooter_position: Shooter's position (x, y, z)
    :param shooter_orientation: Shooter's orientation (quaternion x, y, z, w)
    :param target_position: Target's position (x, y, z)
    :param target_padding: Tuple (padding_x, padding_y, padding_z) for the target's bounding box
    :param threshold: Alignment threshold in meters (radius of the cone)
    :param verbose: Enable or disable debug prints
    :return: True if aligned, False otherwise
    """
    try:
        # Shooter's X-axis in local frame
        x_dir = [1, 0, 0]

        # Get the relative position of the target in the shooter's reference frame
        relative_position = get_relative_position_with_orientation(shooter_position, shooter_orientation, target_position)

        # Normalize vectors
        norm_relative = math.sqrt(sum([i**2 for i in relative_position]))
        relative_position = [i / norm_relative for i in relative_position]

        # Calculate the dot product
        dot_product = sum([x * y for x, y in zip(x_dir, relative_position)])

        # Calculate the angle between the vectors
        angle = math.acos(dot_product)

        # Calculate the distance from the shooter to the target
        distance = get_distance(shooter_position, target_position)

        # Calculate the perpendicular distance to the target from the shooter's X-axis
        perpendicular_distance = distance * math.sin(angle)

        # Calculate the bounding box limits
        padding_x, padding_y, padding_z = target_padding
        bounding_box_corners = [
            (target_position[0] - padding_x, target_position[1] - padding_y, target_position[2] - padding_z),
            (target_position[0] - padding_x, target_position[1] - padding_y, target_position[2] + padding_z),
            (target_position[0] - padding_x, target_position[1] + padding_y, target_position[2] - padding_z),
            (target_position[0] - padding_x, target_position[1] + padding_y, target_position[2] + padding_z),
            (target_position[0] + padding_x, target_position[1] - padding_y, target_position[2] - padding_z),
            (target_position[0] + padding_x, target_position[1] - padding_y, target_position[2] + padding_z),
            (target_position[0] + padding_x, target_position[1] + padding_y, target_position[2] - padding_z),
            (target_position[0] + padding_x, target_position[1] + padding_y, target_position[2] + padding_z)
        ]

        # Project the bounding box corners to the shooter's reference frame
        projected_corners = [get_relative_position_with_orientation(shooter_position, shooter_orientation, corner) for corner in bounding_box_corners]

        # Check if any of the projected corners are within the threshold
        within_threshold = any(
            math.sqrt(corner[1]**2 + corner[2]**2) <= threshold
            for corner in projected_corners
        )

        if verbose:
            # Debug prints
            print(f"Relative position: {relative_position}")
            print(f"Dot product: {dot_product}")
            print(f"Angle: {angle}")
            print(f"Distance: {distance}")
            print(f"Perpendicular distance: {perpendicular_distance}")
            print(f"Bounding box corners: {bounding_box_corners}")
            print(f"Projected corners: {projected_corners}")
            print(f"Within threshold: {within_threshold}")

        return within_threshold
    except Exception as e:
        node.get_logger().error(f"Failed to calculate alignment: {e}")
        return False

def test_is_aligned(node, threshold, padding_x, padding_y, padding_z, verbose=False):
    """
    Test the is_aligned function with various scenarios and a given threshold.
    :param node: The rclpy node
    :param threshold: Alignment threshold in meters
    :param padding_x: Padding along the x-axis
    :param padding_y: Padding along the y-axis
    :param padding_z: Padding along the z-axis
    :param verbose: Enable or disable debug prints
    """

    target_padding = (padding_x, padding_y, padding_z)

    # Test case 1: Perfect alignment (should pass)
    print("Test case 1: Perfect alignment")
    shooter_position = (0, 0, 0)
    shooter_orientation = (0, 0, 0, 1)  # No rotation
    target_position = (1, 0, 0)
    assert is_aligned(node, shooter_position, shooter_orientation, target_position, target_padding, threshold, verbose), "Test case 1 failed"

    # Test case 2: Misalignment (should fail)
    print("Test case 2: Misalignment")
    target_position = (0, 1, 0)
    assert not is_aligned(node, shooter_position, shooter_orientation, target_position, target_padding, threshold, verbose), "Test case 2 failed"

    # Test case 3: Threshold alignment (should pass)
    print("Test case 3: Threshold alignment")
    shooter_orientation = (0, 0, math.sin(math.pi/8), 1)  # 22.5 degrees rotation around Z-axis
    target_position = (1, 1, 0)
    assert is_aligned(node, shooter_position, shooter_orientation, target_position, target_padding, threshold, verbose), "Test case 3 failed"

    # Test case 4: Out of threshold alignment (should fail with tight threshold)
    print("Test case 4: Out of threshold alignment")
    shooter_orientation = (0, 0, math.sin(math.pi/8), 1)  # 22.5 degrees rotation around Z-axis
    target_position = (1, 1, 0)
    assert not is_aligned(node, shooter_position, shooter_orientation, target_position, target_padding, 0.05, verbose), "Test case 4 failed"

    # Test case 5: Shooter aims too far above the target (should fail)
    print("Test case 5: Shooter aims too far above the target")
    shooter_position = (0, 0, 0)
    shooter_orientation = (0, -math.sin(math.pi/4), 0, 1)  # 45 degrees rotation around Y-axis
    target_position = (1, 0, 0)
    assert not is_aligned(node, shooter_position, shooter_orientation, target_position, target_padding, threshold, verbose), "Test case 5 failed"

    print("All test cases passed!")

def main():
    # Initialize the rclpy library
    rclpy.init()

    # Create a Node instance
    node = Node('tool_node')
    node.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])

    # Declare and get the threshold parameter
    node.declare_parameter('threshold', 0.1)
    threshold = node.get_parameter('threshold').get_parameter_value().double_value

    # Declare and get the padding parameters
    node.declare_parameter('padding_x', 0.5)
    node.declare_parameter('padding_y', 0.5)
    node.declare_parameter('padding_z', 0.5)
    padding_x = node.get_parameter('padding_x').get_parameter_value().double_value
    padding_y = node.get_parameter('padding_y').get_parameter_value().double_value
    padding_z = node.get_parameter('padding_z').get_parameter_value().double_value

    # Declare and get the verbose parameter
    node.declare_parameter('verbose', False)
    verbose = node.get_parameter('verbose').get_parameter_value().bool_value

    ### distance test ###
    model_names = ["px4_1", "px4_2"]
    gz = GazeboPosesTracker(model_names)

    ### laser test ###
    # Call the function with the Node instance
    robot_name = "px4_2"
    laser_topic = "laser_beam"
    distance = get_laser_distance(node, robot_name, laser_topic)

    if distance is not None:
        print(f"The minimum laser distance is: {distance:.2f} meters")
    else:
        print("Failed to retrieve laser distance")

    ### alignment test ###
    test_is_aligned(node, threshold, padding_x, padding_y, padding_z, verbose)

    # Shut down the rclpy library
    rclpy.shutdown()

if __name__ == '__main__':
    main()