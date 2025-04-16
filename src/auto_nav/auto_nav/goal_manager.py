import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np

class GoalManager(Node):
    def __init__(self):
        super().__init__('goal_manager')

        # Subscriber to thermal data
        self.sub_thermal = self.create_subscription(
            Float32MultiArray,
            '/temperature_map',
            self.thermal_callback,
            10
        )

        # Publisher for goal locations
        self.pub_goals = self.create_publisher(
            Float32MultiArray,
            '/goal_locations',
            10
        )

        # Threshold and goal list
        self.heat_threshold = 30.0  # Celsius
        self.goal_locations = []

    def thermal_callback(self, msg):
        thermal_grid = np.array(msg.data).reshape((8, 8))
        new_goals = []

        for i in range(8):
            for j in range(8):
                temp = thermal_grid[i][j]
                if temp >= self.heat_threshold:
                    # Assume each cell corresponds to a localized region in map
                    new_goals.append([i, j])

        if new_goals:
            goal_msg = Float32MultiArray()
            goal_msg.data = np.array(new_goals).flatten().tolist()
            self.pub_goals.publish(goal_msg)
            self.get_logger().info(f'Published {len(new_goals)} heat goal(s).')


def main(args=None):
    rclpy.init(args=args)
    node = GoalManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
