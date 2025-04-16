import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import Float32MultiArray
import numpy as np
import random
from map_utils import detect_frontiers
from astar_utils import AStarPlanner

class AutoNavController:
    def __init__(self, node):
        self.node = node
        self.mode = 'exploration'
        self.state = 'RANDOM_WALK'
        self.map_data = None
        self.goal_locations = []
        self.exploration_pass = 1
        self.occ_grid = None
        self.grid_resolution = 0.05  # meters per cell
        self.current_pose = None

    def map_callback(self, msg):
        self.map_data = msg
        self.occ_grid = np.array(msg.data).reshape((msg.info.height, msg.info.width))

    def goal_callback(self, msg):
        self.goal_locations = np.array(msg.data).reshape(-1, 2).tolist()

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose.position

    def control_loop(self):
        if self.mode == 'exploration':
            self.run_multi_pass_exploration()
        elif self.mode == 'navigation':
            self.navigate_to_heat_targets()

    def run_multi_pass_exploration(self):
        if self.state == 'RANDOM_WALK':
            self.node.get_logger().info('Exploration - Random Walk')
            self.moveforward()
            if self.check_mapping_progress():
                self.state = 'FRONTIER_EXPLORE'

        elif self.state == 'FRONTIER_EXPLORE':
            self.node.get_logger().info('Exploration - Frontier Explore')
            self.frontier_explore()
            if self.check_mapping_progress():
                self.state = 'VALIDATION_PASS'

        elif self.state == 'VALIDATION_PASS':
            self.node.get_logger().info('Exploration - Final Refinement')
            self.final_refinement()
            self.mode = 'navigation'

    def navigate_to_heat_targets(self):
        if not self.goal_locations or self.occ_grid is None:
            self.node.get_logger().info('No heat targets or map not ready.')
            return

        start = self.get_current_grid_position()
        if start is None:
            self.node.get_logger().warn('Current position unknown. Cannot plan path.')
            return

        goal = self.closest_goal(start)
        planner = AStarPlanner(self.occ_grid)
        path = planner.plan(start, goal)

        if path:
            self.node.get_logger().info(f'Path found to goal: {goal}. Moving along path.')
            self.execute_path(path)
        else:
            self.node.get_logger().warn('No path found to goal.')

    def moveforward(self):
        twist = Twist()
        twist.linear.x = 0.2
        twist.angular.z = 0.0
        self.node.cmd_pub.publish(twist)

    def frontier_explore(self):
        if self.occ_grid is None:
            return
        frontiers = detect_frontiers(self.occ_grid.flatten(), self.occ_grid.shape[1], self.occ_grid.shape[0])
        if frontiers:
            self.node.get_logger().info(f'{len(frontiers)} frontier cells detected.')
            self.moveforward()
        else:
            self.node.get_logger().info('No frontiers found.')

    def final_refinement(self):
        twist = Twist()
        twist.linear.x = 0.1
        twist.angular.z = 0.0
        self.node.cmd_pub.publish(twist)

    def check_mapping_progress(self):
        return random.random() > 0.7

    def get_current_grid_position(self):
        if self.current_pose is None:
            return None
        x = self.current_pose.x
        y = self.current_pose.y
        gx = int(x / self.grid_resolution)
        gy = int(y / self.grid_resolution)
        return (gy, gx)  # Grid index in (row, col)

    def closest_goal(self, start):
        distances = [np.linalg.norm(np.array(start) - np.array(g)) for g in self.goal_locations]
        return self.goal_locations[np.argmin(distances)]

    def execute_path(self, path):
        for step in path:
            twist = Twist()
            twist.linear.x = 0.2
            twist.angular.z = 0.0
            self.node.cmd_pub.publish(twist)

class R2AutoNav(Node):
    def __init__(self):
        super().__init__('r2auto_nav')
        self.controller = AutoNavController(self)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.controller.map_callback, 10)
        self.goal_sub = self.create_subscription(Float32MultiArray, '/goal_locations', self.controller.goal_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.controller.odom_callback, 10)
        self.timer = self.create_timer(1.0, self.controller.control_loop)

def main(args=None):
    rclpy.init(args=args)
    node = R2AutoNav()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()