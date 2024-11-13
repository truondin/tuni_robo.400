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
        self.yaw = 0.0
        self.timer = self.create_timer(0.5, self.move_to_goal)

        self.constant = 1.4
        self.is_in_goal = False

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        # self.current_theta = math.atan2(self.current_y, self.current_x)
        self.current_theta = math.acos(msg.pose.pose.orientation.w) * 2
        orientation_list = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z,msg.pose.pose.orientation.w]
        # (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(orientation_list)

        print(f"x: {self.current_x}, y: {self.current_y}")


    def linear_vel(self):
        return math.sqrt((self.goal_x - self.current_x) ** 2 + (self.goal_y - self.current_y) ** 2)

    def move_to_goal(self):
        dist_tolerance = 0.1
        angle_tolerance = 0.01

        distance = self.linear_vel()
        angle = math.atan2(self.goal_y - self.current_y, self.goal_x - self.current_x)

        if angle < 0:
            angle_err = self.current_theta + angle
        else:
            angle_err = angle - self.current_theta

        twist = Twist()
        if self.is_in_goal:
            goal_angle_err = self.goal_theta - self.current_theta
            if abs(goal_angle_err) > angle_tolerance:
                twist.angular.z = min(self.constant * angle_err, 0.1)
            else:
                twist.angular.z = 0.0
                twist.linear.x = 0.0
                self.get_logger().info("Goal theta reached")
                self.publisher.publish(twist)
                quit()
        else:
            if abs(angle_err) > angle_tolerance:
                twist.angular.z = min(self.constant * angle_err, 0.1)
            else:
                if distance >= dist_tolerance:
                    twist.linear.x = min(self.constant * distance, 1.0)
                    twist.angular.z = 0.0
                else:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self.get_logger().info("Goal reached")
                    self.publisher.publish(twist)
                    self.is_in_goal = True

        self.publisher.publish(twist)

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
