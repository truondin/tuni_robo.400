#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import sys

class GoToPose(Node):
    def __init__(self, goal_x, goal_y, theta):
        super().__init__('go_to_pose')

        self.goal_x = goal_x
        self.goal_y = goal_y
        self.goal_theta = theta

        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.timer = self.create_timer(0.5, self.control_robot)

        self.kp = 0.7
        self.is_in_goal = False
        self.coords_set = False

        self.dist_tolerance = 0.1
        self.angle_tolerance = 0.01
        self.goal_angle_tolerance = 0.1

    def odom_callback(self, msg):
        self.current_x = round(msg.pose.pose.position.x, 4)
        self.current_y = round(msg.pose.pose.position.y, 4)
        self.current_theta = round(math.acos(msg.pose.pose.orientation.w) * 2, 4)
        if not self.coords_set:
            self.coords_set = True

    def compute_angle(self):
        return math.atan2(self.goal_y - self.current_y, self.goal_x - self.current_x)

    def linear_vel(self):
        return math.sqrt((self.goal_x - self.current_x) ** 2 + (self.goal_y - self.current_y) ** 2)

    def control_robot(self):
        if self.is_in_goal:
            twist = self.rotate()
        else:
            twist = self.move()

        self.publisher.publish(twist)

    def rotate(self):
        twist = Twist()
        if self.goal_theta > self.current_theta:
            goal_angle_err = self.goal_theta - self.current_theta
        else:
            goal_angle_err = self.current_theta - self.goal_theta

        print(f"goal angle err: {goal_angle_err}, goal_theta {self.goal_theta}, curr_theta {self.current_theta}")
        if abs(goal_angle_err) > self.goal_angle_tolerance:
            twist.angular.z = self.kp * goal_angle_err
            return twist
        else:
            twist.angular.z = 0.0
            twist.linear.x = 0.0
            self.get_logger().info("Goal theta reached")
            self.publisher.publish(twist)
            quit()

    def move(self):
        twist = Twist()
        distance = self.linear_vel()
        angle = round(self.compute_angle(), 4)

        if angle < 0:
            angle_err = self.current_theta + angle
        else:
            angle_err = angle - self.current_theta

        if abs(angle_err) > self.angle_tolerance:
            twist.angular.z = min(self.kp * angle_err, 0.1)
            print(f"angular velocity: {twist.angular.z}, current theta: {self.current_theta}, angle: {angle}")
        else:
            if distance >= self.dist_tolerance:
                twist.linear.x = min(self.kp * distance, 1.0)
                twist.angular.z = 0.0
                print(f"linear velocity: {twist.linear.x}")
            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.get_logger().info("Goal reached")
                self.publisher.publish(twist)
                self.is_in_goal = True
        return twist


def main(args=None):
    if len(sys.argv) != 4:
        print("Incorrect arguments - expected go_to_point.py <goal_x> <goal_y> <theta>")
        return
    try:
        x = float(sys.argv[1])
        y = float(sys.argv[2])
        theta = float(sys.argv[3])
    except ValueError:
        print("Incorrect arguments value types - expected floats")
        return

    rclpy.init(args=args)
    node = GoToPose(x, y, theta)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
