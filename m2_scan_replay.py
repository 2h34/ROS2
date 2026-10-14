
import csv
import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

# M2-A 实际采集数据
DATA_DIR = (
    "/mnt/c/Users/glance/Desktop/2027venue/"
    "experiments/lidar_m2a/run02"
)

# 本轮重点验证的扫描
SELECTED = {43, 44, 51, 52, 53, 54, 55, 56, 97}


def main():
    with open(f"{DATA_DIR}/metadata.json") as f:
        meta = json.load(f)

    with open(f"{DATA_DIR}/scans.csv", newline="") as f:
        rows = [
            row for row in csv.DictReader(f)
            if int(row["scan_id"]) in SELECTED
        ]

    rclpy.init()
    node = Node("m2_scan_replay")
    pub = node.create_publisher(
        LaserScan, "/scan", qos_profile_sensor_data
    )

    try:
        print(f"Loaded {len(rows)} selected scans")

        # 等待 Phase 4 检测节点订阅 /scan
        while pub.get_subscription_count() == 0:
            time.sleep(0.1)

        for row in rows:
            msg = LaserScan()

            timestamp_ns = round(float(row["time_s"]) * 1e9)
            msg.header.stamp.sec = timestamp_ns // 1_000_000_000
            msg.header.stamp.nanosec = timestamp_ns % 1_000_000_000
            msg.header.frame_id = meta["frame_id"]

            msg.angle_min = float(meta["angle_min"])
            msg.angle_max = float(meta["angle_max"])
            msg.angle_increment = float(meta["angle_increment"])

            msg.range_min = float(meta["range_min"])
            msg.range_max = float(meta["range_max"])
            msg.scan_time = float(meta["scan_time"])
            msg.time_increment = float(meta["time_increment"])

            n = int(meta["horizontal_resolution"])
            msg.ranges = [float(row[f"r{i}"]) for i in range(n)]

            pub.publish(msg)
            print(f"Published scan {row['scan_id']}")

            # 留出处理和日志输出时间
            time.sleep(0.5)

        time.sleep(1.0)

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
