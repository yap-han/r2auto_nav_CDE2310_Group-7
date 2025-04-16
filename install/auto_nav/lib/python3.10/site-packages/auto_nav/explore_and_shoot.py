#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient

from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose, ComputePathToPose
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Float32MultiArray

import numpy as np
import math
import time
import random
import threading
import subprocess

from .map_utils import detect_frontiers

EXCLUSION_RADIUS = 0.5
GOAL_REPEAT_LIMIT = 5

class NavigationNode(Node):
    def __init__(self):
        super().__init__('explore_and_shoot_node')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.lock = threading.Lock()
        self.occdata = np.array([], dtype=np.int8)
        self.width = 0
        self.height = 0
        self.resolution = 0.05
        self.origin = (0.0, 0.0)

        self.visited_frontiers = []
        self.firing_exclusion_zones = []
        self.goal_attempt_counter = {}
        self.firing_count = 0
        self.max_fires = 3
        self.min_front_dist = 0.35

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, qos_profile_sensor_data)
        self.scan_subscription = self.create_subscription(LaserScan, 'scan', self.scan_callback, qos_profile_sensor_data)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.path_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')

        self.thermal_display = None
        self.latest_odom = None
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, qos_profile_sensor_data)
        self.current_goal_handle = None
        self.temp_map_sub = self.create_subscription(
            Float32MultiArray, '/temperature_map', self.temperature_callback, qos_profile_sensor_data
        )
        self.temperature_array = np.zeros((8, 8))
        self.hot_column = None
        self.heat_detection_threshold = 1800  # for hot column detection
        self.firing_confirmation_threshold = 2800  # for final firing confirmation


        self.latest_pose = None
        self.pose_sub = self.create_subscription(PoseStamped, '/pose', self.pose_callback, qos_profile_sensor_data)

    
    def get_robot_pose(self):
        with self.lock:
            if self.latest_pose:
                p = self.latest_pose.position
                o = self.latest_pose.orientation
                yaw = euler_from_quaternion([o.x, o.y, o.z, o.w])[2]
                return (p.x, p.y), yaw

            if self.latest_odom:
                p = self.latest_odom.position
                o = self.latest_odom.orientation
                yaw = euler_from_quaternion([o.x, o.y, o.z, o.w])[2]
                return (p.x, p.y), yaw

        return None, None



    def map_callback(self, msg):
        with self.lock:
            self.width = msg.info.width
            self.height = msg.info.height
            self.resolution = msg.info.resolution
            self.origin = (msg.info.origin.position.x, msg.info.origin.position.y)
            self.occdata = np.array(msg.data, dtype=np.int8)

    def point_within_map_bounds(self, x, y):
        with self.lock:
            mx = int((x - self.origin[0]) / self.resolution)
            my = int((y - self.origin[1]) / self.resolution)
            return 0 <= mx < self.width and 0 <= my < self.height

    def scan_callback(self, msg):
        with self.lock:
            self.laser_range = np.array(msg.ranges)
            self.laser_range[self.laser_range == 0] = np.nan
    
    def pose_callback(self, msg):
        """
        Callback for the /pose topic. Updates the robot's latest pose.
        """
        with self.lock:
            self.latest_pose = msg.pose
            self.get_logger().info(f"[POSE] Updated pose: Position=({self.latest_pose.position.x:.2f}, {self.latest_pose.position.y:.2f}), "
                                f"Orientation=({self.latest_pose.orientation.x:.2f}, {self.latest_pose.orientation.y:.2f}, "
                                f"{self.latest_pose.orientation.z:.2f}, {self.latest_pose.orientation.w:.2f})")

    def odom_callback(self, msg):
        with self.lock:
            self.latest_odom = msg.pose.pose

    def position_in_exclusion_zone(self, position):
        for ex in self.firing_exclusion_zones:
            if math.hypot(position[0] - ex[0], position[1] - ex[1]) < EXCLUSION_RADIUS:
                self.get_logger().info(f"[EXCLUSION CHECK] Position is within exclusion zone at ({ex[0]:.2f}, {ex[1]:.2f}).")
                return True
        return False

    def get_hot_column_from_display(self):
        with self.lock:
            return self.hot_column

    def rotate_until_centered_hot(self):
        self.get_logger().info("[ROTATE] Starting blocking rotation to center hot column.")
        twist = Twist()
        speed = 0.1
        start_time = time.time()
        timeout = 15.0
        grace_period = 2.0  # Allow 2 seconds of intermittent signal loss
        last_detected_time = time.time()

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            hot_col = self.get_hot_column_from_display()
            self.get_logger().info(f"[ROTATE] Current hot column: {hot_col}.")
            
            if hot_col is not None:
                last_detected_time = time.time()
                if hot_col in [3, 4]:
                    self.get_logger().info(f"[ROTATE] Centered on column {hot_col}.")
                    break
                elif hot_col < 3:
                    twist.angular.z = +abs(speed)
                else:
                    twist.angular.z = -abs(speed)
            else:
                # Check if grace period has expired
                if time.time() - last_detected_time > grace_period:
                    self.get_logger().warn("[ROTATE] No hot column detected for grace period. Stopping rotation.")
                    break

            self.publisher_.publish(twist)
            if time.time() - start_time > timeout:
                self.get_logger().warn("[ROTATE] Timeout while rotating.")
                break
        self.stopbot()

    def move_forward_until_obstacle(self):
        self.get_logger().info("[FORWARD] Starting blocking forward movement.")
        twist = Twist()
        twist.linear.x = 0.1
        start_time = time.time()
        timeout = 15.0

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            front_dist = self.get_front_distance()
            self.get_logger().info(f"[FORWARD] Current front distance: {front_dist:.2f}m.")
            if front_dist < self.min_front_dist:
                self.get_logger().info(f"[FORWARD] Obstacle at {front_dist:.2f}m. Stopping.")
                break
            self.publisher_.publish(twist)
            if time.time() - start_time > timeout:
                self.get_logger().warn("[FORWARD] Timeout.")
                break
        self.stopbot()

    def get_front_distance(self):
        with self.lock:
            if not hasattr(self, 'laser_range') or self.laser_range.size == 0:
                return float('inf')
            front_data = np.concatenate((self.laser_range[0:20], self.laser_range[340:359]))
            if front_data.size == 0 or np.all(np.isnan(front_data)):
                return float('inf')
            return np.nanmin(front_data)

    def perform_firing_sequence(self):
        if not self.confirm_firing_target():
            self.get_logger().warn("[FIRE] Firing aborted — no confirmed heat source.")
            return

        position, _ = self.get_robot_pose()
        if position and not self.position_in_exclusion_zone(position):
            self.firing_exclusion_zones.append(position)
        self.get_logger().info("[FIRE] Starting firing sequence.")
        run_fire_script_over_ssh()
        self.firing_count += 1
        self.get_logger().info(f"[FIRE] Firing done. Count={self.firing_count}")

    def frontier_explore_loop(self):
        while rclpy.ok() and self.firing_count < self.max_fires:
            with self.lock:
                clusters = detect_frontiers(self.occdata, self.width, self.height, dilation_iterations=4)

            if len(clusters) == 0:
                self.get_logger().info("[FRONTIER] No frontiers detected. Moving randomly.")
                self.move_randomly()
                continue

            for cluster in clusters:
                cx, cy = cluster['x'], cluster['y']
                world_x = cx * self.resolution + self.origin[0]
                world_y = cy * self.resolution + self.origin[1]
                goal_key = (round(world_x, 2), round(world_y, 2))

                if not self.point_within_map_bounds(world_x, world_y):
                    self.get_logger().warn(f"[BOUNDS] Skipping out-of-bounds frontier: ({world_x:.2f}, {world_y:.2f})")
                    continue

                self.goal_attempt_counter[goal_key] = self.goal_attempt_counter.get(goal_key, 0) + 1
                if self.goal_attempt_counter[goal_key] > GOAL_REPEAT_LIMIT:
                    self.get_logger().warn(f"[GOAL] Repeated goal {goal_key} too many times. Switching to random walk.")
                    self.move_randomly()
                    break

                if not self.is_goal_reachable(world_x, world_y):
                    self.get_logger().warn(f"[GOAL] Pose {goal_key} is not reachable. Skipping.")
                    continue

                self.get_logger().info(f"[FRONTIER] Moving to: {goal_key}")
                success = self.send_goal_and_wait(world_x, world_y, yaw=0.0)

                if success:
                    self.visited_frontiers.append(goal_key)
                    break
                else:
                    self.get_logger().warn("[GOAL] Frontier goal timed out.")
                    continue
            
    def move_randomly(self, max_attempts=20):
        position, _ = self.get_robot_pose()
        if position is None:
            return

        rx, ry = position
        for _ in range(max_attempts):
            angle = random.uniform(0, 2 * math.pi)
            dist = 1.0 + random.random() * 1.5
            nx = rx + dist * math.cos(angle)
            ny = ry + dist * math.sin(angle)
            if self.point_within_map_bounds(nx, ny) and self.is_goal_reachable(nx, ny):
                self.get_logger().info(f"[RANDOM] Moving to: ({nx:.2f}, {ny:.2f})")
                self.send_goal_and_wait(nx, ny, yaw=0.0, timeout_s=15.0)
                return
        self.get_logger().warn("[RANDOM] No reachable random goal found after several attempts.")

    def send_goal_and_wait(self, x, y, yaw=0.0, timeout_s=30.0):
        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, yaw)
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.x = qx
        goal_msg.pose.pose.orientation.y = qy
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw

        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn('[NAV] Nav2 server not available')
            return False

        future = self.nav_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            return False

        self.current_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        start = time.time()

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

            # 🔥 NEW: If hot column detected mid-navigation, interrupt
            hot_col = self.get_hot_column_from_display()
            robot_pos, _ = self.get_robot_pose()
            if hot_col is not None and robot_pos is not None and not self.position_in_exclusion_zone(robot_pos):
                self.get_logger().warn("[NAV] Hot column detected during navigation. Cancelling goal.")
                self.cancel_current_goal()
                self.rotate_until_centered_hot()
                self.move_forward_until_obstacle()
                self.perform_firing_sequence()
                return True  # start new loop

            if result_future.done():
                status = result_future.result().status
                return status == 4

            if time.time() - start > timeout_s:
                self.cancel_current_goal()
                return False

        return False


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

    def cancel_current_goal(self):
        if self.current_goal_handle:
            cancel_future = self.current_goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=1.0)
            self.current_goal_handle = None
        self.latest_pose = None


    def stopbot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher_.publish(twist)

    def temperature_callback(self, msg):
        with self.lock:
            try:
                self.temperature_array = np.array(msg.data).reshape((8, 8))
                self.hot_column = self.compute_hot_column(self.temperature_array)
                self.thermal_display = self.generate_display(self.hot_column)

                # 🔥 Only show display when a hot column is detected
                if self.hot_column is not None:
                    self.get_logger().info(f"[THERMAL DISPLAY] Hot Column: {self.hot_column}\n{self.thermal_display}")
            except Exception as e:
                self.get_logger().error(f"[TEMP] Error processing temperature map: {e}")


    def compute_hot_column(self, temp_array, threshold=None, min_hot_pixels=3):
        hot_column = None
        max_count = 0
        max_avg = 0.0
        if threshold is None:
            threshold = self.heat_detection_threshold

        for col in range(8):
            col_vals = temp_array[:, col]
            hot_vals = col_vals[col_vals > threshold]
            if len(hot_vals) >= min_hot_pixels:
                avg = np.mean(hot_vals)
                if len(hot_vals) > max_count or (len(hot_vals) == max_count and avg > max_avg):
                    max_count = len(hot_vals)
                    max_avg = avg
                    hot_column = col
        return hot_column

    def confirm_firing_target(self):
        with self.lock:
            if self.temperature_array is None or self.temperature_array.shape != (8, 8):
                self.get_logger().warn("[FIRE CHECK] No valid temperature data.")
                return False

            for col in range(2, 7):  # center columns: 2, 3, 4, 5, 6
                col_vals = self.temperature_array[:, col]
                strong_hits = col_vals[col_vals > self.firing_confirmation_threshold]
                if len(strong_hits) >= 3:
                    self.get_logger().info(f"[FIRE CHECK] Confirmed in column {col} with {len(strong_hits)} hot pixels.")
                    return True

            self.get_logger().warn("[FIRE CHECK] No strong heat source in center columns.")
            return False


    def generate_display(self, hot_col):
        lines = []
        for _ in range(8):
            line = list("........")
            if hot_col is not None and 0 <= hot_col < 8:
                line[hot_col] = 'X'
            lines.append("".join(line))
        return "\n".join(lines)


def run_fire_script_over_ssh():
    rpi_ip = "192.168.162.224"
    rpi_user = "ubuntu"
    script_to_run = "/home/ubuntu/fire.py"
    ssh_cmd = f"ssh {rpi_user}@{rpi_ip} 'source /opt/ros/humble/setup.bash && python3 {script_to_run}'"
    try:
        print("[SSH] Running fire.py on RPi...")
        subprocess.run(ssh_cmd, shell=True, check=True)
        print("[SSH] fire.py completed successfully on RPi.")
    except subprocess.CalledProcessError as e:
        print(f"[SSH] fire.py failed on RPi: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()

    start = time.time()
    timeout = 10.0
    while rclpy.ok() and (time.time() - start < timeout):
        rclpy.spin_once(node, timeout_sec=0.5)
        if node.width > 0 and node.height > 0:
            node.get_logger().info("[STARTUP] Map initialized. Starting exploration.")
            break
        else:
            node.get_logger().info("[WAIT] Waiting for SLAM to publish map...")
    else:
        node.get_logger().warn("Timeout waiting for /map. Proceeding anyway.")

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    try:
        node.frontier_explore_loop()
    except KeyboardInterrupt:
        node.stopbot()
    finally:
        node.stopbot()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        thread.join()

if __name__ == '__main__':
    main()