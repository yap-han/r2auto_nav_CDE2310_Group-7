import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float32MultiArray
import numpy as np

class R2Occupancy2(Node):
    def __init__(self):
        super().__init__('r2occupancy2')

        # Subscriptions
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        self.thermal_sub = self.create_subscription(
            Float32MultiArray,
            '/temperature_map',
            self.thermal_callback,
            10
        )

        # Internal map containers
        self.latest_map = None
        self.latest_temperature = None

    def map_callback(self, msg):
        self.latest_map = msg
        self.get_logger().info('SLAM map received.')

        # Optionally, overlay heatmap here if temperature data is available
        if self.latest_temperature:
            self.overlay_heatmap()

    def thermal_callback(self, msg):
        self.latest_temperature = np.array(msg.data).reshape((8, 8))
        self.get_logger().info('Thermal data received.')

    def overlay_heatmap(self):
        # Placeholder logic: this function represents where SLAM map would
        # be enhanced with thermal map information (e.g., color overlay, heat scoring grid, etc.)

        # Real implementation requires spatial correlation between robot pose and temperature readings
        heat_max = np.max(self.latest_temperature)
        heat_min = np.min(self.latest_temperature)

        self.get_logger().info(f'Overlaying heatmap (range {heat_min:.1f}°C to {heat_max:.1f}°C) on SLAM map.')
        # In future: publish composite visualization, goal marking, etc.


def main(args=None):
    rclpy.init(args=args)
    node = R2Occupancy2()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
