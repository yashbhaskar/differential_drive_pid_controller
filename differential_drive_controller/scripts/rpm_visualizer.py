#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt
from std_msgs.msg import Float64
import time

class RPMSubscriber(Node):
    def __init__(self):
        super().__init__('rpm_subscriber')

        self.create_subscription(Float64, '/left_wheel_rpm', self.left_rpm_callback, 10)
        self.create_subscription(Float64, '/right_wheel_rpm', self.right_rpm_callback, 10)

        self.left_rpm_values = []
        self.right_rpm_values = []
        self.time_values = []

        # Initialize Matplotlib
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(10, 6))
        self.line1, = self.ax.plot([], [], 'b-o', label='Left RPM')
        self.line2, = self.ax.plot([], [], 'r-o', label='Right RPM')

        self.ax.set_xlabel('Time (iterations)')
        self.ax.set_ylabel('RPM')
        self.ax.set_title('Wheel RPMs Over Time')
        self.ax.legend()
        plt.show()

    def left_rpm_callback(self, msg):
        """Callback for left wheel RPM."""
        self.left_rpm_values.append(msg.data)
        self.time_values.append(len(self.left_rpm_values))
        self.get_logger().info(f'Left Wheel RPM: {msg.data}')
        self.update_plot()

    def right_rpm_callback(self, msg):
        """Callback for right wheel RPM."""
        self.right_rpm_values.append(msg.data)
        self.get_logger().info(f'Right Wheel RPM: {msg.data}')
        self.update_plot()

    def update_plot(self):
        """Update the Matplotlib graph dynamically."""
        self.line1.set_xdata(self.time_values)
        self.line1.set_ydata(self.left_rpm_values)

        self.line2.set_xdata(self.time_values[:len(self.right_rpm_values)])
        self.line2.set_ydata(self.right_rpm_values)

        self.ax.relim()
        self.ax.autoscale_view()

        plt.draw()
        plt.pause(0.1)

def main():
    rclpy.init()
    node = RPMSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down RPM subscriber node.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
