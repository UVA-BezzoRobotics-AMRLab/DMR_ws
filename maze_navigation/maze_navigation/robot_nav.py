import rclpy
import numpy as np

from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

from maze_navigation.Map import Map
from maze_navigation.Visualizer import PathVisualizer

class Controller(Node):
    def __init__(self):
        super().__init__('controller_node')

        qos_map = rclpy.qos.QoSProfile(
            durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            qos_map
        )

        # Setup subscriber for ground truth robot pose
        self.odom_sub = self.create_subscription(
            Odometry,
            '/base_pose_ground_truth',
            self.odom_callback,
            1
        )

        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/laser1',
            self.lidar_callback,
            1
        )

        self.cmd_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        self.map_data = None
        self.get_logger().info("Controller Node started. Waiting for map...")

        # control timer
        self.timer = self.create_timer(0.1, self.control_loop)

        # robot pose
        self.pose = None

        # lidar slices
        self.lidar_slices = 8
        self.lidar_data = None
        self.lidar_angles = None
        
        # define the navigation goal
        self.goal = np.array([18.0, 7.0])

        # Initialize the path visualizer, this object will
        # publish the global plan to a topic that RViz can visualize
        self.path_visualizer = PathVisualizer(self)

    def odom_callback(self, msg):
        """
        Triggered when odometry data is received.
        msg.pose.pose contains the robot's current position and orientation.
        """
        quat = msg.pose.pose.orientation
        # Convert quaternion to yaw
        siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        self.pose = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, yaw])

    def map_callback(self, msg):
        """
        Triggered when a map is received.
        msg.data is a 1D tuple of signed chars (int8).
        """
        self.map_data = Map(msg)
        self.get_logger().info("Map received and processed.")

    def lidar_callback(self, msg):
        self.lidar_angle_min = msg.angle_min
        self.lidar_angle_increment = (msg.angle_max - msg.angle_min) / len(msg.ranges)
        
        slice_size = len(msg.ranges) // self.lidar_slices
        self.lidar_data = []
        for i in range(self.lidar_slices):
            slice_ranges = msg.ranges[i*slice_size:(i+1)*slice_size]
            valid_ranges = [r for r in slice_ranges if not np.isinf(r)]
            self.lidar_data.append(np.average(valid_ranges) if valid_ranges else float('inf'))

        self.lidar_angles = [
            msg.angle_min + (i + 0.5) * (msg.angle_max - msg.angle_min) / self.lidar_slices
            for i in range(self.lidar_slices)
        ]

    def control_loop(self):
        if self.map_data is None or self.pose is None or self.lidar_data is None:
            return

        msg = Twist()
        msg.linear.x = 0.5
        msg.angular.z = 0.2
        if self.lidar_data[len(self.lidar_data)//2] < 1.5:
            msg.linear.x = 0.0
            msg.angular.z = 0.0

        self.cmd_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()