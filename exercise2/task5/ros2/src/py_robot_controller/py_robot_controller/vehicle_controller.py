#!/usr/bin/env python3

import rclpy
import random

from pynput import keyboard
from pynput.keyboard import KeyCode
from rclpy.node import Node
from sensor_msgs.msg import LaserScan  # Lidar message
from geometry_msgs.msg import Twist


class VehicleController(Node):

    def __init__(self):
        super().__init__('vehicle_controller')
        self.auto_mode = True
        self.data_to_pub = Twist()

        self.subscription = self.create_subscription(
            LaserScan,
            'lidar',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()
        self.get_logger().info('Automatic mode')

    def listener_callback(self, msg: LaserScan):
        if self.auto_mode:
            self.auto_control(msg)
        else:
            self.publisher_.publish(self.data_to_pub)

    def on_press(self, key):
        if key == keyboard.Key.enter:
            self.auto_mode = not self.auto_mode
            if self.auto_mode:
                self.get_logger().info('Automatic mode')
            else:
                self.get_logger().info('Manual mode - use arrows to move')

        self.keyboard_control(key)

    def on_release(self, key):
        self.data_to_pub = Twist()

    def keyboard_control(self, key):
        data_pub = Twist()
        if type(key) == KeyCode:
            return

        if key.name == 'left':
            data_pub.linear.x = 0.0
            data_pub.angular.z = 0.5
        elif key.name == 'right':
            data_pub.linear.x = 0.0
            data_pub.angular.z = -0.5
        elif key.name == 'up':
            data_pub.linear.x = 1.0
            data_pub.angular.z = 0.0
        elif key.name == 'down':
            data_pub.linear.x = -1.0
            data_pub.angular.z = 0.0

        self.data_to_pub = data_pub


    def kill_listener(self):
        self.listener.stop()

    def auto_control(self, msg: LaserScan):
        all_more = True
        for value in msg.ranges:
            if value < 1.0:
                all_more = False
                break

        data_pub = Twist()
        if all_more:
            data_pub.linear.x = 1.0
            data_pub.angular.z = 0.0
        else:
            data_pub.linear.x = 0.0
            data_pub.angular.z = 0.5

        self.publisher_.publish(data_pub)


def main(args=None):
    rclpy.init(args=args)

    vehicle_controller = VehicleController()

    rclpy.spin(vehicle_controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    vehicle_controller.kill_listener()
    vehicle_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
