#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import sys


class GoToPoint(Node):
    def __init__(self, goal_x, goal_y):
        super().__init__('go_to_point')

        self.goal_x = goal_x
        self.goal_y = goal_y

        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.timer = self.create_timer(0.5, self.move_to_goal)

        self.constant = 0.5

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_theta = math.atan2(self.current_y, self.current_x)
        # self.current_theta = math.acos(msg.pose.pose.orientation.w) * 2
        print(f"x: {self.current_x}, y: {self.current_y}")

    def angular_vel(self):
        steering_angle = math.atan2(self.goal_y - self.current_y, self.goal_x - self.current_x)
        return steering_angle - self.current_theta

    def linear_vel(self):
        return math.sqrt((self.goal_x - self.current_x) ** 2 + (self.goal_y - self.current_y) ** 2)

    def move_to_goal(self):
        dist_tolerance = 0.5

        distance = self.linear_vel()
        angular_vel = self.angular_vel()

        twist = Twist()
        # print(f"distance {distance} curr_x {self.current_x} curr_y {self.current_y} ")
        if distance >= dist_tolerance:
            twist.linear.x = self.constant * distance
            twist.angular.z = self.constant * angular_vel
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.get_logger().info("Reached the goal.")
            self.publisher.publish(twist)
            quit()

        self.publisher.publish(twist)

def main(args=None):
    if len(sys.argv) != 3:
        print("Incorrect arguments - expected go_to_point.py <goal_x> <goal_y>")
        return
    try:
        x = float(sys.argv[1])
        y = float(sys.argv[2])
    except ValueError:
        print("Incorrect arguments value types - expected floats")
        return

    rclpy.init(args=args)
    node = GoToPoint(x, y)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
