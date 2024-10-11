#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan  # Lidar message
from geometry_msgs.msg import Twist

class VehicleController(Node):

    def __init__(self):
        super().__init__('vehicle_controller')
        self.subscription = self.create_subscription(
            LaserScan,
            'lidar',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)



    def listener_callback(self, msg: LaserScan):
        all_more = True
        for value in msg.ranges:
            if value < 1:
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
    vehicle_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
