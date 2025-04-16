import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np
from scipy.ndimage import zoom
import random

class SimulatedAMG8833Node(Node):
    def __init__(self):
        super().__init__('sim_amg8833_node')

        # Setup Publisher
        self.publisher = self.create_publisher(Float32MultiArray, '/temperature_map', 10)

        # Timer to simulate periodic data
        self.timer = self.create_timer(0.5, self.read_publish_temperature)
        self.get_logger().info('Simulated AMG8833 Node running (no I2C hardware).')

    def read_publish_temperature(self):
        try:
            # Simulate 8x8 thermal grid with one hot spot
            base_temp = 25.0 + np.random.normal(0, 0.3, size=(8, 8))
            hot_col = random.randint(0, 7)
            base_temp[:, hot_col] += np.random.uniform(4, 8)  # simulate heat source

            temperature_array = base_temp
            upsampled = zoom(temperature_array, 2, order=1)

            col_means = upsampled.mean(axis=0)
            max_col_index = np.argmax(col_means)

            # Display ASCII heat map
            rows, cols = upsampled.shape
            char_map = np.full((rows, cols), '.', dtype='<U1')
            char_map[:, max_col_index] = 'X'

            print("\n16x16 Interpolated Heat Map (column with highest average = X):")
            for row in char_map:
                print(" ".join(row))

            # Calculate approximate angle
            fov_degrees = 90.0
            angle_per_col = fov_degrees / cols
            angle_offset = -fov_degrees / 2.0
            angle = angle_offset + (max_col_index + 0.5) * angle_per_col

            print(f"Column with highest avg temp = {max_col_index} (Angle ~ {angle:.2f} degrees)")

            # Publish original 64 values
            msg = Float32MultiArray()
            msg.data = temperature_array.flatten().tolist()
            self.publisher.publish(msg)
            self.get_logger().info(f'Simulated thermal frame published: {len(msg.data)} points')

        except Exception as e:
            self.get_logger().error(f'Simulation error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = SimulatedAMG8833Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
