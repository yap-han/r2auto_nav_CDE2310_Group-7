import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import MultiThreadedExecutor
from rclpy.duration import Duration

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException

import numpy as np
import math
import threading
import time
import random


class FrontierExplorer(Node):
    # Configurable parameters for speeds
    LINEAR_SPEED = 0.3  # Linear speed for navigation and random walk
    ANGULAR_SPEED = 0.6  # Angular speed for rotation

    # Configurable parameters for timeouts
    OBSTACLE_TIMEOUT = 3.0  # Timeout for obstacle avoidance (seconds)
    RANDOM_WALK_TIMEOUT = 15.0  # Timeout for random walk (seconds)
    FRONTIER_NAVIGATION_TIMEOUT = 45.0  # Timeout for navigating to a frontier (seconds)

    # Configurable parameters for obstacle detection
    MIN_OBSTACLE_DISTANCE = 0.25  # Minimum distance to detect an obstacle (meters)
    LIDAR_ANGLE_RANGE = 45  # Angle range for obstacle detection (degrees)

    # Configurable parameters for path smoothing
    ANGLE_TOLERANCE = 0.2  # Tolerance for angle alignment (radians)
    DISTANCE_TOLERANCE = 0.25  # Tolerance for reaching the goal (meters)

    def __init__(self):
        super().__init__('frontier_explorer_node')
        self.lock = threading.Lock()
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, qos_profile_sensor_data)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)

        self.occdata = None
        self.visited_map = None
        self.map_resolution = 0.05
        self.map_origin = (0.0, 0.0)
        self.latest_pose = None
        self.latest_yaw = 0.0
        self.laser_ranges = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def map_callback(self, msg):
        with self.lock:
            self.map_resolution = msg.info.resolution
            self.map_origin = (msg.info.origin.position.x, msg.info.origin.position.y)
            self.occdata = np.array(msg.data).reshape((msg.info.height, msg.info.width))
            if self.visited_map is None:
                self.visited_map = np.zeros_like(self.occdata, dtype=bool)

    def scan_callback(self, msg):
        with self.lock:
            self.laser_ranges = np.array(msg.ranges)
            self.laser_ranges[self.laser_ranges == 0] = np.nan

    def update_pose_from_tf(self):
        try:
            now = rclpy.time.Time()
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', now, timeout=Duration(seconds=0.5)
            )
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            self.latest_pose = transform.transform.translation

            q = transform.transform.rotation
            siny = 2.0 * (q.w * q.z + q.x * q.y)
            cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            self.latest_yaw = math.atan2(siny, cosy)

            mx = int((x - self.map_origin[0]) / self.map_resolution)
            my = int((y - self.map_origin[1]) / self.map_resolution)
            if self.visited_map is not None:
                if 0 <= my < self.visited_map.shape[0] and 0 <= mx < self.visited_map.shape[1]:
                    self.visited_map[my, mx] = True

        except (LookupException, ConnectivityException, ExtrapolationException):
            self.get_logger().warn("TF transform unavailable (map → base_footprint)")

    def stopbot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher.publish(twist)

    def detect_frontiers(self):
        frontiers = []
        if self.occdata is None:
            return frontiers
        height, width = self.occdata.shape
        for y in range(1, height - 1):
            for x in range(1, width - 1):
                if self.occdata[y, x] == 0:
                    neighbors = self.occdata[y-1:y+2, x-1:x+2]
                    if -1 in neighbors:
                        wx = x * self.map_resolution + self.map_origin[0]
                        wy = y * self.map_resolution + self.map_origin[1]
                        frontiers.append((wx, wy))
        return frontiers

    def navigate_to_point(self, target_x, target_y, timeout=45.0):
        start_time = time.time()
        while rclpy.ok():
            elapsed = time.time() - start_time
            if elapsed > timeout:
                self.get_logger().warn(f"[NAV] Timeout reached while navigating to ({target_x:.2f}, {target_y:.2f}).")
                self.stopbot()
                return

            self.update_pose_from_tf()

            with self.lock:
                if self.latest_pose is None:
                    continue
                x = self.latest_pose.x
                y = self.latest_pose.y
                yaw = self.latest_yaw

            dx = target_x - x
            dy = target_y - y
            distance = math.hypot(dx, dy)
            if distance < self.DISTANCE_TOLERANCE:
                self.stopbot()
                return

            target_yaw = math.atan2(dy, dx)
            angle_diff = (target_yaw - yaw + math.pi) % (2 * math.pi) - math.pi

            twist = Twist()

            # 360-degree obstacle avoidance
            if self.laser_ranges is not None:
                front = np.nanmin(np.concatenate([self.laser_ranges[330:360], self.laser_ranges[0:30]]))
                left = np.nanmin(self.laser_ranges[60:120])
                right = np.nanmin(self.laser_ranges[240:300])

                if front < self.MIN_OBSTACLE_DISTANCE:
                    self.get_logger().warn(f"[NAV] Obstacle detected in front at {front:.2f}m. Stopping.")
                    twist.linear.x = 0.0
                    twist.angular.z = self.ANGULAR_SPEED
                elif left < self.MIN_OBSTACLE_DISTANCE:
                    self.get_logger().warn(f"[NAV] Obstacle detected on the left at {left:.2f}m. Steering right.")
                    twist.angular.z = -self.ANGULAR_SPEED
                elif right < self.MIN_OBSTACLE_DISTANCE:
                    self.get_logger().warn(f"[NAV] Obstacle detected on the right at {right:.2f}m. Steering left.")
                    twist.angular.z = self.ANGULAR_SPEED
                else:
                    if abs(angle_diff) > self.ANGLE_TOLERANCE:
                        twist.angular.z = self.ANGULAR_SPEED * np.sign(angle_diff)
                    else:
                        twist.linear.x = self.LINEAR_SPEED

            self.publisher.publish(twist)
            time.sleep(0.05)

    def random_walk(self, timeout=15.0):
        self.get_logger().info('[RANDOM WALK] No frontiers. Performing random exploration.')
        start_time = time.time()
        while time.time() - start_time < timeout:
            angle = random.uniform(-math.pi, math.pi)
            duration = abs(angle) / self.ANGULAR_SPEED

            twist = Twist()
            twist.angular.z = self.ANGULAR_SPEED * np.sign(angle)
            rotation_start = time.time()
            while time.time() - rotation_start < duration:
                if self.laser_ranges is not None:
                    min_distance = np.nanmin(self.laser_ranges)
                    if min_distance < self.MIN_OBSTACLE_DISTANCE:
                        self.get_logger().warn(f"[RANDOM WALK] Obstacle detected at {min_distance:.2f}m during rotation. Stopping.")
                        self.stopbot()
                        return
                self.publisher.publish(twist)
                time.sleep(0.05)

            self.stopbot()
            time.sleep(0.5)

            twist = Twist()
            twist.linear.x = self.LINEAR_SPEED
            forward_start = time.time()
            while time.time() - forward_start < 1.5:
                if self.laser_ranges is not None:
                    min_distance = np.nanmin(self.laser_ranges)
                    if min_distance < self.MIN_OBSTACLE_DISTANCE:
                        self.get_logger().warn(f"[RANDOM WALK] Obstacle detected at {min_distance:.2f}m during forward movement. Stopping.")
                        self.stopbot()
                        return
                self.publisher.publish(twist)
                time.sleep(0.05)

            self.stopbot()

    def main_loop(self):
        while rclpy.ok():
            self.update_pose_from_tf()
            frontiers = self.detect_frontiers()
            if frontiers:
                goal = random.choice(frontiers)
                self.get_logger().info(f"[FRONTIER] Navigating to {goal}")
                self.navigate_to_point(*goal, timeout=self.FRONTIER_NAVIGATION_TIMEOUT)
            else:
                self.random_walk(timeout=self.RANDOM_WALK_TIMEOUT)
            time.sleep(0.1)


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    try:
        node.main_loop()
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt - Shutting down")
    finally:
        node.stopbot()
        executor.shutdown()
        executor_thread.join()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()