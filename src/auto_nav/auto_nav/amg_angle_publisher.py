#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
import smbus2
import time
import numpy as np

class AMG_Angle_Publisher(Node):
    def __init__(self):
        super().__init__('amg8833_angle_publisher')

        # I2C Configuration
        self.bus = smbus2.SMBus(1)  # I2C Bus 1 on Raspberry Pi
        self.address = 0x69  # Default AMG8833 I2C address

        # Publishers for raw temperature data and hot angles.
        self.temp_publisher = self.create_publisher(Float32MultiArray, '/temperature_map', 10)
        self.angle_publisher = self.create_publisher(Float32MultiArray, '/hot_angles', 10)
        # NEW: Publisher for an 8x8 display of the thermal frame (dots everywhere except a column of X’s)
        self.display_publisher = self.create_publisher(String, '/thermal_display', 10)

        # Timer: Read sensor and publish data every 0.5 sec.
        self.timer = self.create_timer(0.5, self.read_publish_temperature)
        self.get_logger().info('AMG8833 Node initialized and reading temperatures...')

        # Thermal detection criteria (matching what your nav node uses)
        self.threshold = 1800   # Raw temperature threshold
        self.min_hot_pixels = 3  # Minimum hot pixels required in a column

    def read_publish_temperature(self):
        try:
            data = []
            # Read 128 bytes (64 values) from registers starting at 0x80.
            for i in range(0x80, 0x80 + 128, 2):
                raw = self.bus.read_word_data(self.address, i)
                temp = self.convert_raw_temperature(raw)
                data.append(temp)

            # Convert flat data array into an 8x8 numpy matrix.
            temperature_array = np.array(data).reshape((8, 8))

            # Print the raw temperature array.
            print("Temperature Array:")
            for i in range(8):
                print(temperature_array[i])

            # Determine hot pixels per column.
            hot_column = None
            max_count = 0
            max_avg = 0.0
            # For each column compute the number and average of hot pixels.
            for col in range(8):
                col_vals = temperature_array[:, col]
                hot_vals = col_vals[col_vals > self.threshold]
                if len(hot_vals) >= self.min_hot_pixels:
                    avg = np.mean(hot_vals)
                    if len(hot_vals) > max_count or (len(hot_vals) == max_count and avg > max_avg):
                        max_count = len(hot_vals)
                        max_avg = avg
                        hot_column = col

            # For the angle publisher (if needed) compute corresponding angles.
            angle_per_col = 60 / 7.0  # With 7 intervals spanning -30° to +30°
            angles = []
            if hot_column is not None:
                # Compute angle for the detected hottest column.
                angle = -30 + hot_column * angle_per_col
                angles = [angle]
            print(f"Hot Column: {hot_column}, Hot Angle: {angles}")

            # Create a display: an 8x8 grid of dots; if a hot column exists, show 'X' in that column.
            display_lines = []
            for row in range(8):
                # Start with a list of dots.
                line = list("........")
                if hot_column is not None:
                    line[hot_column] = 'X'
                display_lines.append("".join(line))
            display_str = "\n".join(display_lines)
            
            # Print the display.
            print("Thermal Display:")
            print(display_str)

            # Publish raw temperature array.
            temp_msg = Float32MultiArray()
            temp_msg.data = data
            self.temp_publisher.publish(temp_msg)
            self.get_logger().info(f'Thermal frame published: {len(data)} points')

            # Publish the hot angle information.
            angle_msg = Float32MultiArray()
            angle_msg.data = angles
            self.angle_publisher.publish(angle_msg)

            # Publish the display string.
            display_msg = String()
            display_msg.data = display_str
            self.display_publisher.publish(display_msg)
            self.get_logger().info('Thermal display published.')

        except Exception as e:
            self.get_logger().error(f'AMG8833 read error: {e}')

    def convert_raw_temperature(self, raw):
        # Swap bytes due to little-endian format.
        raw_swapped = ((raw & 0xFF) << 8) | ((raw >> 8) & 0xFF)
        temp = raw_swapped * 0.25
        if temp > 2047:
            temp -= 4096
        return temp


def main(args=None):
    rclpy.init(args=args)
    node = AMG_Angle_Publisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
