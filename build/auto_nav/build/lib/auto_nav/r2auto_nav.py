#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient

from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose, ComputePathToPose
from geometry_msgs.msg import PoseStamped, Twist
from tf2_ros import Buffer, TransformListener
from tf_transformations import quaternion_from_euler
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import LaserScan
from sklearn.cluster import DBSCAN

import numpy as np
import math
import threading
import time
import subprocess
import random

from .map_utils import detect_frontiers


def run_fire_script_over_ssh():
    rpi_ip = "192.168.162.224"
    rpi_user = "ubuntu"
    script_to_run = "/home/ubuntu/fire.py"
    ssh_cmd = f"ssh {rpi_user}@{rpi_ip} 'source /opt/ros/humble/setup.bash && python3 {script_to_run}'"
    try:
        subprocess.run(ssh_cmd, shell=True, check=True)
    except subprocess.CalledProcessError as e:
        print(f"[SSH] fire.py failed on RPi: {e}")


class NavigationNode(Node):
    def __init__(self):
        super().__init__('explore_and_fire_node')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.lock = threading.Lock()

        self.occdata = np.array([], dtype=np.int8)
        self.resolution = 0.05
        self.origin = (0.0, 0.0)
        self.width = 0
        self.height = 0

        self.all_hot_positions = []
        self.heat_goals = []
        self.exploration_start_time = time.time()
        self.exploration_time_limit_s = 300
        self.exploration_elapsed_s = 0

        self.goal_attempt_counter = {}

        self.laser_ranges = np.array([])
        self.lidar_angle_min = 0.0
        self.lidar_angle_increment = 0.0

        self.publisher_ = self.create_publisher(Twist, "/cmd_vel", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "/heat_goals_marker", 10)
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.path_client = ActionClient(self, ComputePathToPose, "/compute_path_to_pose")

        self.create_subscription(OccupancyGrid, "/map", self.map_callback, qos_profile_sensor_data)
        self.create_subscription(Float32MultiArray, "/temperature_map", self.thermal_callback, qos_profile_sensor_data)
        self.create_subscription(LaserScan, "/scan", self.scan_callback, qos_profile_sensor_data)

        self.marker_timer = self.create_timer(1.0, self.publish_all_hot_positions)

    def get_robot_pose(self):
        try:
            now = rclpy.time.Time()
            transform = self.tf_buffer.lookup_transform('map', 'base_footprint', now, timeout=rclpy.duration.Duration(seconds=0.5))
            t = transform.transform.translation
            return (t.x, t.y)
        except Exception:
            return None

    def map_callback(self, msg):
        with self.lock:
            self.width = msg.info.width
            self.height = msg.info.height
            self.resolution = msg.info.resolution
            self.origin = (msg.info.origin.position.x, msg.info.origin.position.y)
            self.occdata = np.array(msg.data, dtype=np.int8)

    def scan_callback(self, msg):
        self.laser_ranges = np.array(msg.ranges)
        self.lidar_angle_min = msg.angle_min
        self.lidar_angle_increment = msg.angle_increment

    def thermal_callback(self, msg):
        if self.laser_ranges.size == 0:
            return
        temps = np.array(msg.data).reshape((8, 8))
        hot_pixel_counts = np.sum(temps > 2000, axis=0)
        hot_columns = np.where(hot_pixel_counts > 3)[0]
        if len(hot_columns) == 0:
            return

        angle_per_col = 60.0 / 7.0
        angles_rad = [math.radians(-30 + col * angle_per_col) for col in hot_columns]
        pos = self.get_robot_pose()
        if pos is None:
            return
        rx, ry = pos
        hot_positions = []
        for ang in angles_rad:
            global_angle = ang % (2 * math.pi)
            index = int((global_angle - self.lidar_angle_min) / self.lidar_angle_increment)
            if 0 <= index < len(self.laser_ranges):
                dist = self.laser_ranges[index]
                if not np.isnan(dist) and not np.isinf(dist) and dist <= 10.0:
                    x = rx + dist * math.cos(global_angle)
                    y = ry + dist * math.sin(global_angle)
                    hot_positions.append((x, y))
                    self.get_logger().info(f"[HEAT] Hot position detected: ({x:.2f}, {y:.2f})")
        with self.lock:
            self.all_hot_positions.extend(hot_positions)

    def publish_all_hot_positions(self):
        marker_array = MarkerArray()
        for i, (x, y) in enumerate(self.all_hot_positions):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "hot_positions"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.2
            marker.scale.x = marker.scale.y = marker.scale.z = 0.15
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker_array.markers.append(marker)
        self.marker_pub.publish(marker_array)

    def cluster_heat_goals(self):
        if not self.all_hot_positions:
            return
        X = np.array(self.all_hot_positions)
        clustering = DBSCAN(eps=0.3, min_samples=3).fit(X)
        goals = []
        for label in set(clustering.labels_):
            if label == -1:
                continue
            cluster_pts = X[clustering.labels_ == label]
            cx, cy = np.mean(cluster_pts, axis=0)
            goals.append((cx, cy))
        self.heat_goals = goals

    def explore(self):
        while rclpy.ok():
            self.exploration_elapsed_s = time.time() - self.exploration_start_time
            if self.exploration_elapsed_s > self.exploration_time_limit_s:
                self.get_logger().info("[EXPLORE] Time limit reached. Stopping.")
                break
            clusters = detect_frontiers(self.occdata, self.width, self.height, dilation_iterations=4)
            if not clusters:
                self.move_randomly()
                continue
            for c in clusters:
                x = c['x'] * self.resolution + self.origin[0]
                y = c['y'] * self.resolution + self.origin[1]
                key = (round(x, 2), round(y, 2))
                self.goal_attempt_counter[key] = self.goal_attempt_counter.get(key, 0) + 1
                if self.goal_attempt_counter[key] > 5:
                    self.get_logger().warn(f"[EXPLORE] Skipping repeated goal {key}")
                    continue
                if self.is_goal_reachable(x, y):
                    self.get_logger().info(f"[NAV] Moving to frontier: ({x:.2f}, {y:.2f})")
                    if self.send_goal_and_wait(x, y):
                        break
                else:
                    self.get_logger().info(f"[EXPLORE] Skipping unreachable goal ({x:.2f}, {y:.2f})")

    def is_goal_reachable(self, x, y):
        if not self.path_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('[REACHABILITY] Path planner not available')
            return False

        goal_msg = ComputePathToPose.Goal()
        goal_msg.goal.header.frame_id = 'map'
        goal_msg.goal.header.stamp = self.get_clock().now().to_msg()
        goal_msg.goal.pose.position.x = x
        goal_msg.goal.pose.position.y = y
        goal_msg.goal.pose.orientation.w = 1.0

        future = self.path_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        return len(result.path.poses) > 0

    def move_randomly(self, max_attempts=20):
        pos = self.get_robot_pose()
        if pos is None:
            return
        rx, ry = pos
        for _ in range(max_attempts):
            angle = random.uniform(0, 2 * math.pi)
            dist = 1.0 + random.random() * 1.5
            nx = rx + dist * math.cos(angle)
            ny = ry + dist * math.sin(angle)
            if self.is_goal_reachable(nx, ny):
                self.get_logger().info(f"[NAV] Moving randomly to: ({nx:.2f}, {ny:.2f})")
                self.send_goal_and_wait(nx, ny, timeout_s=15.0)
                return
        self.get_logger().warn("[RANDOM] No reachable random goal found after several attempts.")

    def send_goal_and_wait(self, x, y, yaw=0.0, timeout_s=30):
        qx, qy, qz, qw = quaternion_from_euler(0, 0, yaw)
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.x = qx
        goal.pose.pose.orientation.y = qy
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            return False
        future = self.nav_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()
        if not handle or not handle.accepted:
            return False
        result_future = handle.get_result_async()
        start_time = time.time()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if result_future.done():
                return result_future.result().status == 4
            if time.time() - start_time > timeout_s:
                handle.cancel_goal_async()
                return False

    def fire_at_heat_goals(self):
        for i, (x, y) in enumerate(self.heat_goals):
            self.get_logger().info(f"[FIRE] Moving to heat goal {i+1}: ({x:.2f}, {y:.2f})")
            success = self.send_goal_and_wait(x, y, timeout_s=90)
            if success:
                self.get_logger().info("[FIRE] Goal reached. Executing fire.")
            else:
                self.get_logger().warn("[FIRE] Failed to reach goal. Firing anyway.")
            run_fire_script_over_ssh()


def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        node.explore()
        node.cluster_heat_goals()
        node.fire_at_heat_goals()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        thread.join()


if __name__ == '__main__':
    main()
