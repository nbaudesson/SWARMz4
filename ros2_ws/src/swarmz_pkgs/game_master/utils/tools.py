import numpy as np
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
    
    :param shooter_position: Tuple (x, y, z) representing the shooter's global position.
    :param shooter_orientation: Tuple (x, y, z, w) representing the shooter's orientation (quaternion).
    :param target_position: Tuple (x, y, z) representing the target's global position.
    :return: Tuple (x, y, z) representing the target position in the shooter's local reference frame.
    """
    shooter_position = np.array(shooter_position)
    target_position = np.array(target_position)
    
    # Convert the shooter's quaternion to a 4x4 transformation matrix, then take the 3x3 rotation part.
    rotation_matrix = quaternion_matrix(shooter_orientation)[:3, :3]
    
    # Compute the vector from shooter to target in global frame.
    relative_position = target_position - shooter_position
    
    # To transform a global vector into the shooter's local frame, we apply the inverse rotation (transpose).
    transformed_position = rotation_matrix.T.dot(relative_position)
    
    return tuple(transformed_position)

def is_aligned(node, shooter_position, shooter_orientation, target_position, target_padding, range, threshold, verbose=False):
    """
    Check if target is within a cone defined by range (height) and threshold (base radius).
    The cone expands linearly from apex (shooter) to base, forming a true cone shape.
    
    :param range: Height/length of the cone
    :param threshold: Radius of the cone at its base (maximum radius)
    """
    try:
        # Get relative position in shooter's reference frame
        rel_pos = get_relative_position_with_orientation(shooter_position, shooter_orientation, target_position)
        
        # Check if target is in front and within range
        forward_distance = rel_pos[0]
        if forward_distance <= 0 or forward_distance > range:
            return False
            
        # Calculate allowed radius at this distance (forms a cone)
        # At shooter (distance=0): radius=0
        # At max range: radius=threshold
        max_radius =((threshold) * (forward_distance / range)) + target_padding[0]
        
        # Check if target is within cone radius at this distance
        radius_at_point = math.sqrt(rel_pos[1]**2 + rel_pos[2]**2)
        
        if verbose:
            node.get_logger().info(f"Relative position: {rel_pos}")
            node.get_logger().info(f"Distance along cone: {forward_distance}")
            node.get_logger().info(f"Radius at point: {radius_at_point}")
            node.get_logger().info(f"Max allowed radius at this distance: {max_radius}")
            
        return radius_at_point <= max_radius
        
    except Exception as e:
        node.get_logger().error(f"Failed to calculate alignment: {e}")
        return False

def get_cube_corners(position, orientation, padding, is_quaternion=True):
    """
    Calculate the 8 corners of a cube in global coordinates using different padding for each axis.
    
    :param position: np.array([x, y, z]) center position in global frame
    :param orientation: Either quaternion [x, y, z, w] or euler angles [roll, pitch, yaw]
    :param padding: Tuple (padding_x, padding_y, padding_z) half-lengths of the cube edges
    :param is_quaternion: True if orientation is quaternion, False if euler angles
    :return: np.array of shape (8, 3) containing corner positions in global frame
    """
    # Convert inputs to numpy arrays
    position = np.array(position)
    orientation = np.array(orientation)
    padding_x, padding_y, padding_z = padding
    
    # Define corners in local frame using different padding for each axis
    corners_local = np.array([
        [-padding_x, -padding_y, -padding_z],  # back bottom left
        [-padding_x, -padding_y, padding_z],   # back top left
        [-padding_x, padding_y, -padding_z],   # front bottom left
        [-padding_x, padding_y, padding_z],    # front top left
        [padding_x, -padding_y, -padding_z],   # back bottom right
        [padding_x, -padding_y, padding_z],    # back top right
        [padding_x, padding_y, -padding_z],    # front bottom right
        [padding_x, padding_y, padding_z]      # front top right
    ])
    
    # Get rotation matrix
    if is_quaternion:
        rotation_matrix = quaternion_matrix(orientation)[:3, :3]
    else:
        # Assuming ZYX order for euler angles
        roll, pitch, yaw = orientation
        from scipy.spatial.transform import Rotation as R
        rotation_matrix = R.from_euler('zyx', [yaw, pitch, roll]).as_matrix()
    
    # Rotate all corners at once, using numpy's matrix multiplication operator @,
    #  which is more efficient than doing individual dot products.
    rotated_corners = corners_local @ rotation_matrix.T
    
    # Translate all corners at once
    global_corners = rotated_corners + position
    
    return global_corners

def is_aligned_HB(node, shooter_position, shooter_orientation, target_position, target_padding, range, threshold, verbose=False):
    """
    Check if a shooter's X-axis points toward any part of the target's hitbox.
    Uses cone-rectangle intersection tests for each face of the box.
    
    The algorithm:
    1. Get box corners in global frame
    2. Transform corners to shooter's perspective
    3. For each face of the box:
       a. Check if face is facing the shooter
       b. Check if any corner is inside the cone
       c. Project face onto YZ plane and check circle-rectangle intersection
    """
    try:
        # First get box corners in global space, then transform to shooter's view
        corners = get_cube_corners(
            target_position,
            (0, 0, 0, 1),  # No rotation for target
            target_padding
        )
        
        # Transform all corners to how the shooter sees them
        # After this, shooter is at origin looking along +X axis
        corners_relative = np.array([
            get_relative_position_with_orientation(
                shooter_position,
                shooter_orientation,
                corner
            ) for corner in corners
        ])
        
        if verbose:
            node.get_logger().info("=== Starting Alignment Check ===")
            node.get_logger().info(f"Target position: {target_position}")
            node.get_logger().info(f"Target padding: {target_padding}")
            node.get_logger().info(f"Shooter position: {shooter_position}")
            node.get_logger().info(f"Range: {range}, Threshold: {threshold}")
            node.get_logger().info(f"Corners in shooter frame:\n{corners_relative}")

        # Define each face by its four corner indices
        # Order matters: corners must form a valid rectangle
        faces = [
            (0,1,2,3),  # -X face (front)
            (4,5,6,7),  # +X face (back)
            (0,1,4,5),  # -Y face (left)
            (2,3,6,7),  # +Y face (right)
            (0,2,4,6),  # -Z face (bottom)
            (1,3,5,7)   # +Z face (top)
        ]

        for face_idx, face_indices in enumerate(faces):
            if verbose:
                node.get_logger().info(f"\n=== Checking Face {face_idx} ===")
            
            face_corners = corners_relative[list(face_indices)]
            edge1 = face_corners[1] - face_corners[0]
            edge2 = face_corners[2] - face_corners[0]
            normal = np.cross(edge1, edge2)
            normal = normal / np.linalg.norm(normal)
            center = np.mean(face_corners, axis=0)
            
            if center[0] <= 0:
                if verbose:
                    node.get_logger().info(f"Skipping face {face_idx}: Behind shooter (X = {center[0]:.3f})")
                continue

            if verbose:
                node.get_logger().info(f"Face center: {center}")
                node.get_logger().info(f"Face normal: {normal}")

            # Corner check
            if verbose:
                node.get_logger().info("\n--- Checking corners ---")
            
            for corner_idx, corner in enumerate(face_corners):
                forward_dist = corner[0]
                if 0 < forward_dist <= range:
                    lateral_dist = math.sqrt(corner[1]**2 + corner[2]**2)
                    cone_radius = (threshold * forward_dist) / range
                    
                    if verbose:
                        node.get_logger().info(f"Corner {corner_idx}:")
                        node.get_logger().info(f"  Position: {corner}")
                        node.get_logger().info(f"  Forward distance: {forward_dist:.3f}")
                        node.get_logger().info(f"  Lateral distance: {lateral_dist:.3f}")
                        node.get_logger().info(f"  Cone radius at this distance: {cone_radius:.3f}")
                        node.get_logger().info(f"  Within cone: {lateral_dist <= cone_radius}")
                    
                    if lateral_dist <= cone_radius:
                        if verbose:
                            node.get_logger().info(f"Hit detected on corner {corner_idx} of face {face_idx}")
                        return True

            # Face intersection check
            if verbose:
                node.get_logger().info("\n--- Checking face intersection ---")
            
            face_x = center[0]
            if face_x <= 0 or face_x > range:
                if verbose:
                    node.get_logger().info(f"Face {face_idx} out of range: X = {face_x:.3f}")
                continue
            
            min_y = min(c[1] for c in face_corners)
            max_y = max(c[1] for c in face_corners)
            min_z = min(c[2] for c in face_corners)
            max_z = max(c[2] for c in face_corners)
            cone_radius = (threshold * face_x) / range
            
            rect_half_width = (max_y - min_y)/2
            rect_half_height = (max_z - min_z)/2
            rect_center_y = (min_y + max_y)/2
            rect_center_z = (min_z + max_z)/2

            if verbose:
                node.get_logger().info(f"Face bounds in YZ plane:")
                node.get_logger().info(f"  Y: [{min_y:.3f}, {max_y:.3f}]")
                node.get_logger().info(f"  Z: [{min_z:.3f}, {max_z:.3f}]")
                node.get_logger().info(f"  Cone radius at this distance: {cone_radius:.3f}")
                node.get_logger().info(f"  Rectangle center: ({rect_center_y:.3f}, {rect_center_z:.3f})")
                node.get_logger().info(f"  Rectangle size: {rect_half_width*2:.3f} x {rect_half_height*2:.3f}")
                node.get_logger().info(f"  Distance to center: Y={abs(rect_center_y):.3f}, Z={abs(rect_center_z):.3f}")
                node.get_logger().info(f"  Allowed distance: Y={rect_half_width + cone_radius:.3f}, Z={rect_half_height + cone_radius:.3f}")

            if abs(rect_center_y) <= rect_half_width + cone_radius and \
               abs(rect_center_z) <= rect_half_height + cone_radius:
                if verbose:
                    node.get_logger().info(f"Hit detected on face {face_idx}")
                return True

        if verbose:
            node.get_logger().info("\n=== No intersection found with any face ===")
        return False
        
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
    cone_range = 100.0  # Test with 100m range
    base_radius = threshold  # Use threshold as base radius

    # Test case 1: Perfect alignment (should pass)
    print("Test case 1: Perfect alignment")
    shooter_position = (0, 0, 0)
    shooter_orientation = (0, 0, 0, 1)  # No rotation
    target_position = (1, 0, 0)
    assert is_aligned(node, shooter_position, shooter_orientation, target_position, target_padding, cone_range, base_radius, verbose), "Test case 1 failed"

    # Test case 2: Misalignment (should fail)
    print("Test case 2: Misalignment")
    target_position = (0, 1, 0)
    assert not is_aligned(node, shooter_position, shooter_orientation, target_position, target_padding, cone_range, base_radius, verbose), "Test case 2 failed"

    # Test case 3: Threshold alignment (should pass)
    print("Test case 3: Threshold alignment")
    shooter_orientation = (0, 0, math.sin(math.pi/8), 1)  # 22.5 degrees rotation around Z-axis
    target_position = (1, 1, 0)
    assert is_aligned(node, shooter_position, shooter_orientation, target_position, target_padding, cone_range, base_radius, verbose), "Test case 3 failed"

    # Test case 4: Out of threshold alignment (should fail with tight threshold)
    print("Test case 4: Out of threshold alignment")
    shooter_orientation = (0, 0, math.sin(math.pi/8), 1)  # 22.5 degrees rotation around Z-axis
    target_position = (1, 1, 0)
    assert not is_aligned(node, shooter_position, shooter_orientation, target_position, target_padding, cone_range, 0.05, verbose), "Test case 4 failed"

    # Test case 5: Shooter aims too far above the target (should fail)
    print("Test case 5: Shooter aims too far above the target")
    shooter_position = (0, 0, 0)
    shooter_orientation = (0, -math.sin(math.pi/4), 0, 1)  # 45 degrees rotation around Y-axis
    target_position = (1, 0, 0)
    assert not is_aligned(node, shooter_position, shooter_orientation, target_position, target_padding, cone_range, base_radius, verbose), "Test case 5 failed"

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