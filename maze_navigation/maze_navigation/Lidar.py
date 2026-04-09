import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan, PointCloud2
from nav_msgs.msg import Odometry
import sensor_msgs_py.point_cloud2 as pc2

import math


class ScanProjector(Node):
    def __init__(self):
        super().__init__('scan_projector')

        self.scan_sub = self.create_subscription(
            LaserScan, '/laser1', self.scan_cb, 10)

        self.pose_sub = self.create_subscription(
            Odometry, '/base_pose_ground_truth', self.pose_cb, 10)

        self.pub = self.create_publisher(
            PointCloud2, '/scan_points', 10)

        self.pose = None

    def pose_cb(self, msg):
        self.pose = msg.pose.pose

    def scan_cb(self, scan):
        if self.pose is None:
            return

        points = []

        # Extract pose
        px = self.pose.position.x
        py = self.pose.position.y

        q = self.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

        angle = scan.angle_min

        for r in scan.ranges:
            if math.isinf(r) or math.isnan(r):
                angle += scan.angle_increment
                continue

            lx = r * math.cos(angle)
            ly = r * math.sin(angle)

            wx = px + (lx * math.cos(yaw) - ly * math.sin(yaw))
            wy = py + (lx * math.sin(yaw) + ly * math.cos(yaw))

            points.append([wx, wy, 0.0])

            angle += scan.angle_increment

        cloud = pc2.create_cloud_xyz32(scan.header, points)
        cloud.header.frame_id = 'odom'  # ground truth odom

        self.pub.publish(cloud)


def main():
    rclpy.init()
    node = ScanProjector()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()