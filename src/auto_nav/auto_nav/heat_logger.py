# heat_logger.py
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import numpy as np
import math
import threading

class HeatLogger(Node):
    def __init__(self):
        super().__init__('heat_logger')

        self.lock = threading.Lock()
        self.latest_scan = None
        self.latest_pose = None
        self.heat_threshold = 29.0  # adjust as needed
        self.fov_deg = 90.0
        self.cols = 16
        self.angle_per_col = self.fov_deg / self.cols
        self.angle_offset = -self.fov_deg / 2.0

        self.hot_points = []  # store (x, y) poses
        self.hot_clusters = []

        self.create_subscription(Float32MultiArray, '/temperature_map', self.thermal_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.get_logger().info("HeatLogger initialized.")

    def scan_callback(self, msg):
        with self.lock:
            self.latest_scan = msg

    def odom_callback(self, msg):
        with self.lock:
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            self.latest_pose = (x, y)

    def thermal_callback(self, msg):
        if len(msg.data) != 64:
            self.get_logger().warn("Unexpected thermal data length.")
            return

        temp_array = np.array(msg.data).reshape((8, 8))
        upsampled = self.interpolate(temp_array)
        col_means = upsampled.mean(axis=0)

        hot_columns = np.where(col_means > self.heat_threshold)[0]

        with self.lock:
            if self.latest_scan is None or self.latest_pose is None:
                return

            for col in hot_columns:
                angle_deg = self.angle_offset + (col + 0.5) * self.angle_per_col
                angle_rad = math.radians(angle_deg)
                lidar_angle_index = int((angle_deg - math.degrees(self.latest_scan.angle_min)) / math.degrees(self.latest_scan.angle_increment))

                if 0 <= lidar_angle_index < len(self.latest_scan.ranges):
                    dist = self.latest_scan.ranges[lidar_angle_index]
                    if self.latest_scan.range_min < dist < self.latest_scan.range_max:
                        dx = dist * math.cos(angle_rad)
                        dy = dist * math.sin(angle_rad)
                        x_world = self.latest_pose[0] + dx
                        y_world = self.latest_pose[1] + dy
                        self.hot_points.append((x_world, y_world))
                        self.get_logger().info(f"Logged hot point at ({x_world:.2f}, {y_world:.2f})")

    def interpolate(self, array_8x8):
        from scipy.ndimage import zoom
        return zoom(array_8x8, 2, order=1)

    def get_clusters(self, radius=0.3, min_points=3):
        from sklearn.cluster import DBSCAN
        if not self.hot_points:
            return []

        X = np.array(self.hot_points)
        clustering = DBSCAN(eps=radius, min_samples=min_points).fit(X)
        labels = clustering.labels_

        clusters = []
        for label in set(labels):
            if label == -1:
                continue
            points = X[labels == label]
            x_mean = np.mean(points[:, 0])
            y_mean = np.mean(points[:, 1])
            clusters.append((x_mean, y_mean))

        self.hot_clusters = clusters
        return clusters

    def save_clusters_to_file(self, path="/tmp/hot_clusters.txt"):
        self.get_logger().info(f"Saving clusters to {path}")
        with open(path, "w") as f:
            for x, y in self.get_clusters():
                f.write(f"{x},{y}\n")

def main(args=None):
    rclpy.init(args=args)
    node = HeatLogger()

    def save_on_exit():
        node.save_clusters_to_file()

    import atexit
    atexit.register(save_on_exit)

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
