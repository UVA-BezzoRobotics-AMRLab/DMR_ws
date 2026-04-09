import rclpy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

class PathVisualizer:
    def __init__(self, node, topic_name='/global_plan'):
        # this node is the Controller
        self.node = node
        self.publisher = self.node.create_publisher(Path, topic_name, 10)

    def publish_path(self, path_coords):
        """
        path_coords: list of [x, y] or np.array of shape (N, 2)
        """
        if path_coords is None:
            return

        msg = Path()
        msg.header = Header()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = 'map'  # Ensure this matches your Fixed Frame in RViz

        for coord in path_coords:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = float(coord[0])
            pose.pose.position.y = float(coord[1])
            pose.pose.position.z = 0.0
            # No orientation needed for a simple line, but valid quats are good practice
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)

        self.publisher.publish(msg)
