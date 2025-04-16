import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np

class HeatLogger(Node):
    def __init__(self):
        super().__init__('heat_logger')

        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/temperature_map',
            self.thermal_callback,
            10
        )

        self.heat_threshold = 30.0
        self.frame_count = 0

    def thermal_callback(self, msg):
        self.frame_count += 1
        thermal_data = np.array(msg.data).reshape((8, 8))
        max_temp = np.max(thermal_data)
        hot_cells = np.argwhere(thermal_data >= self.heat_threshold)

        self.get_logger().info(f'Thermal Frame {self.frame_count}: Max Temp = {max_temp:.2f}°C, Hot Cells = {len(hot_cells)}')
        if hot_cells.any():
            for cell in hot_cells:
                i, j = cell
                temp = thermal_data[i][j]
                self.get_logger().info(f'  - Cell ({i},{j}): {temp:.2f}°C')


def main(args=None):
    rclpy.init(args=args)
    node = HeatLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
