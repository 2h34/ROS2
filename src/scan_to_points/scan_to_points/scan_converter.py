import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class ScanConverter(Node):
    def __init__(self):
        super().__init__("scan_converter")
        self.subscription = self.create_subscription(
            LaserScan, "/scan", self.scan_callback, 10
        )
        self.publisher = self.create_publisher(PointCloud2, "/scan_points", 10)

    def scan_callback(self, msg):
        points = []
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r):
                continue

            theta = msg.angle_min + i * msg.angle_increment
            x = r * math.cos(theta)
            y = r * math.sin(theta)
            z = 0.0
            points.append([x, y, z])
        cloud = point_cloud2.create_cloud_xyz32(msg.header, points)
        self.publisher.publish(cloud)


def main():
    rclpy.init()
    node = ScanConverter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
