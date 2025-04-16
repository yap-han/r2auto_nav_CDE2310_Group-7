import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import smbus2
import time

class AMG8833Node(Node):
    def __init__(self):
        super().__init__('amg8833_node')

        # I2C Configuration
        self.bus = smbus2.SMBus(1)  # I2C Bus 1 on RPi4
        self.address = 0x69  # Default AMG8833 I2C address

        # Setup Publisher
        self.publisher = self.create_publisher(Float32MultiArray, '/temperature_map', 10)

        # Timer to read sensor data periodically
        self.timer = self.create_timer(0.5, self.read_publish_temperature)
        self.get_logger().info('AMG8833 Node initialized and reading temperatures...')

    def read_publish_temperature(self):
        try:
            # AMG8833 stores 64 temperature values (8x8) at registers 0x80 to 0xFF
            data = []
            for i in range(0x80, 0x80 + 128, 2):
                raw = self.bus.read_word_data(self.address, i)

                # Convert to temperature using datasheet method
                temp = self.convert_raw_temperature(raw)
                data.append(temp)

            msg = Float32MultiArray()
            msg.data = data
            self.publisher.publish(msg)
            self.get_logger().info(f'Thermal frame published: {len(data)} points')

        except Exception as e:
            self.get_logger().error(f'AMG8833 read error: {e}')

    def convert_raw_temperature(self, raw):
        # Swap bytes due to little-endian format
        raw_swapped = ((raw & 0xFF) << 8) | ((raw >> 8) & 0xFF)
        temp = raw_swapped * 0.25
        # Optional: Sign correction for negative temperatures
        if temp > 2047:
            temp -= 4096
        return temp


def main(args=None):
    rclpy.init(args=args)
    node = AMG8833Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
