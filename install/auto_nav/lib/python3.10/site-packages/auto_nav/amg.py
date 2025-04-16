import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import smbus2
import time
import numpy as np

class AMG8833Node(Node):
    def __init__(self):
        super().__init__('amg8833_node')

        # I2C Configuration
        self.bus = smbus2.SMBus(1)  # I2C Bus 1 on RPi4
        self.address = 0x69  # Default AMG8833 I2C address

        # Setup Publisher
        self.publisher = self.create_publisher(Float32MultiArray, '/temperature_map', 10)

        # Timer to read sensor data periodically
        self.timer = self.create_timer(0.1, self.read_publish_temperature)
        self.get_logger().info('AMG8833 Node initialized and reading temperatures...')

        # self.fig, self.ax = plt.subplots()
        # self.im = self.ax.imshow(np.zeros((8, 8)), cmap='hot', interpolation='nearest')
        # plt.colorbar(self.im, ax=self.ax)
        # plt.ion()  # Turn on interactive mode
        # plt.show()

    def read_publish_temperature(self):
        try:
            # AMG8833 stores 64 temperature values (8x8) at registers 0x80 to 0xFF
            data = []
            for i in range(0x80, 0x80 + 128, 2):
                raw = self.bus.read_word_data(self.address, i)

                # Convert to temperature using datasheet method
                temp = self.convert_raw_temperature(raw)
                data.append(temp)

            print("Temperature Array:")
            for i in range(8):
                print(data[i*8:(i+1)*8])

            # Convert the temperature array to a numpy array and reshape it to 8x8
            temperature_array = np.array(data).reshape((8, 8))

            # self.im.set_array(temperature_array)
            # self.ax.set_title('Thermal Image')
            # plt.draw()
            # plt.pause(0.01)  # Pause to update the plot
                     
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
    import rclpy
    rclpy.init(args=args)
    node = AMG8833Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
