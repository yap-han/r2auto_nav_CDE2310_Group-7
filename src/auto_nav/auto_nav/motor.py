#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from gpiozero import Motor
from time import sleep
import atexit

class AutomaticMotorNode(Node):
    def __init__(self):
        super().__init__('automatic_motor_node')

        # Initialize two motors with non-conflicting GPIO pins:
        # Motor1: uses GPIO pins 9 (forward) and 11 (backward)
        # Motor2: uses GPIO pins 10 (forward) and 12 (backward)
        self.motor1 = Motor(forward=1, backward=6)
        self.motor2 = Motor(forward=17, backward=27)

        self.get_logger().info('Automatic Motor Node with two motors has started.')
        self.run_motor_loop()

    def run_motor_loop(self):
        try:
            while rclpy.ok():
                # Run both motors simultaneously:
                self.get_logger().info('Motor1 rotating anticlockwise, Motor2 rotating clockwise.')
                # For motor1, using backward() rotates it anticlockwise.
                # For motor2, using forward() rotates it clockwise.
                self.motor1.forward()
                self.motor2.backward()    
                sleep(5)  # Run for 5 seconds

                # # Stop both motors
                # self.get_logger().info('Stopping both motors.')
                # self.motor1.stop()
                # self.motor2.stop()
                # sleep(2)  # Pause for 2 seconds

                # # Optionally reverse directions
                # self.get_logger().info('Reversing directions: Motor1 clockwise, Motor2 anticlockwise.')
                # self.motor1.backward()
                # self.motor2.forward()
                # sleep(5)  # Run for 5 seconds

                # self.get_logger().info('Stopping both motors.')
                # self.motor1.stop()
                # self.motor2.stop()
                # sleep(2)  # Pause for 2 seconds

        except KeyboardInterrupt:
            self.get_logger().info('Motor operation interrupted by user.')
            self.shutdown_motors()

    def shutdown_motors(self):
        # Stop both motors
        self.get_logger().info('Shutting down: Stopping both motors.')
        self.motor1.stop()
        self.motor2.stop()

    def destroy_node(self):
        self.shutdown_motors()
        self.get_logger().info('Automatic Motor Node is shutting down.')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AutomaticMotorNode()

    # Register an atexit callback to ensure motors are stopped when the program exits
    atexit.register(node.shutdown_motors)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt detected.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
