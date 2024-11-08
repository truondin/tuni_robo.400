#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import sys


class GoToPoint(Node):
    def __init__(self, goal_x, goal_y):
        super().__init__('go_to_point2')

        self.goal_x = goal_x
        self.goal_y = goal_y

        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.coords_set = False
        self.timer = self.create_timer(0.5, self.move_to_goal)
        # self.timer = self.create_timer(0.5, self.test)

        self.constant = 1

    def test(self):
        print(f"pos {self.current_x} {self.current_y}, theta {self.current_theta}")
        angle = round(math.atan2(self.goal_y - self.current_y, self.goal_x - self.current_x), 4)
        print(f"angle {angle}")

    def odom_callback(self, msg):
        self.current_x = round(msg.pose.pose.position.x, 4)
        self.current_y = round(msg.pose.pose.position.y, 4)
        self.current_theta = round(math.acos(msg.pose.pose.orientation.w) * 2, 4)
        if not self.coords_set:
            self.coords_set = True

        # print(f"x: {self.current_x}, y: {self.current_y}")

    def compute_angle(self):
        return math.atan2(self.goal_y - self.current_y, self.goal_x - self.current_x)

    def linear_vel(self):
        return math.sqrt((self.goal_x - self.current_x) ** 2 + (self.goal_y - self.current_y) ** 2)

    def move_to_goal(self):
        if not self.coords_set:
            return

        dist_tolerance = 0.1
        angle_tolerance = 0.01

        distance = self.linear_vel()
        angle = round(math.atan2(self.goal_y - self.current_y, self.goal_x - self.current_x), 4)

        angle_err = angle - self.current_theta

        twist = Twist()
        if abs(angle_err) > angle_tolerance:
            twist.angular.z = self.constant * angle_err
            print(f"angular velocity: {twist.angular.z}, current theta: {self.current_theta}, angle: {angle}")
        else:
            if distance >= dist_tolerance:
                twist.linear.x = self.constant * distance
                twist.angular.z = 0.0
                print(f"linear velocity: {twist.linear.x}")
            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.get_logger().info("Goal reached")
                self.publisher.publish(twist)
                self.is_in_goal = True
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
