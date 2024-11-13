#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import sys


class GoByPoints(Node):
    def __init__(self, points):
        super().__init__('go_by_points')
        self.checkpoints = points
        self.goal_x = points[0][0]
        self.goal_y = points[0][1]
        self.next_index = 1

        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.coords_set = False
        self.timer = self.create_timer(0.5, self.move_by_path)
        self.kp = 0.7

        self.dist_tolerance = 0.1
        self.angle_tolerance = 0.01

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

    def move_to_goal(self):
        distance = self.linear_vel()
        angle = round(self.compute_angle(), 4)

        if angle < 0:
            angle_err = self.current_theta + angle
        else:
            angle_err = angle - self.current_theta

        twist = Twist()
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
                return True

        self.publisher.publish(twist)
        return False

    def move_by_path(self):
        if not self.coords_set:
            return

        in_goal = self.move_to_goal()
        if in_goal:
            if self.next_index < len(self.checkpoints):
                next_goal_point = self.checkpoints[self.next_index]
                self.goal_x = next_goal_point[0]
                self.goal_y = next_goal_point[1]
                self.next_index += 1
            else:
                self.publisher.publish(Twist())
                self.get_logger().info("Final goal reached")
                quit()

def parse_input_coords(input):
    if input[0] != "[" or input[-1] != "]":
        print("Invalid input format, coordinates should be in brackets [] and without white spaces - expected example: go_to_point.py [0,0] [1,1] ...")
        quit()

    str_coords = input[1:-1].split(",")
    if len(str_coords) != 2:
        print("Invalid coordinates format - expected [x1,y1]")
        quit()
    try:
        coords = [float(str_coords[0]), float(str_coords[1])]
        return coords
    except ValueError:
        print("Incorrect arguments value types - expected is number")
        quit()

def main(args=None):
    if len(sys.argv) < 2:
        print("Incorrect arguments - expected go_to_point.py <[x1,y1] [x2,y2] ...> ")
        return

    points = []
    for i in range(1, len(sys.argv)):
        coords = parse_input_coords(sys.argv[i])
        points.append(coords)

    rclpy.init(args=args)
    node = GoByPoints(points)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
