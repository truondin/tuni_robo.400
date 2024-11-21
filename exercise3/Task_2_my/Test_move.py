#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry  # Updated to use Odometry for Gazebo
from math import pow, atan2, sqrt

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')

        # Publisher to the topic '/cmd_vel' to control the robot's velocity
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber to the topic '/odom' to get the robot's current position
        self.odom_subscription = self.create_subscription(Odometry, '/odom', self.update_pose, 10)

        self.pose = None  # Initially, pose is not set until we get a message
        self.rate = self.create_rate(10)
        self.goal_pose = None
        self.distance_tolerance = 0.1  # Default tolerance, can be modified

    def update_pose(self, msg):
        """Callback function which updates the robot's current pose based on Odometry data."""
        # Update pose with x, y, and theta (yaw)
        self.pose = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'theta': self.get_yaw_from_quaternion(msg.pose.pose.orientation)
        }

    def get_yaw_from_quaternion(self, orientation):
        """Convert quaternion to yaw (theta) for 2D navigation."""
        import math
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y**2 + orientation.z**2)
        return math.atan2(siny_cosp, cosy_cosp)

    def euclidean_distance(self, goal_pose):
        """Euclidean distance between current pose and the goal."""
        return sqrt(pow((goal_pose['x'] - self.pose['x']), 2) + pow((goal_pose['y'] - self.pose['y']), 2))

    def linear_vel(self, goal_pose, constant=1.5):
        """Calculate linear velocity towards the goal."""
        return constant * self.euclidean_distance(goal_pose)

    def steering_angle(self, goal_pose):
        """Calculate the angle towards the goal."""
        return atan2(goal_pose['y'] - self.pose['y'], goal_pose['x'] - self.pose['x'])

    def angular_vel(self, goal_pose, constant=6):
        """Calculate angular velocity to align with the goal."""
        return constant * (self.steering_angle(goal_pose) - self.pose['theta'])

    def move_to_goal(self):
        """Moves the robot to the goal."""
        self.goal_pose = {
            'x': float(input("Set your x goal: ")),
            'y': float(input("Set your y goal: "))
        }
        self.distance_tolerance = float(input("Set your tolerance: "))

        vel_msg = Twist()

        while self.pose is None:  # Wait for the first odom message
            rclpy.spin_once(self)

        while self.euclidean_distance(self.goal_pose) >= self.distance_tolerance:
            # Linear velocity in the x-axis
            vel_msg.linear.x = self.linear_vel(self.goal_pose)
            vel_msg.linear.y = 0.0
            vel_msg.linear.z = 0.0

            # Angular velocity in the z-axis
            vel_msg.angular.x = 0.0
            vel_msg.angular.y = 0.0
            vel_msg.angular.z = self.angular_vel(self.goal_pose)

            # Publish velocity
            self.velocity_publisher.publish(vel_msg)

            # Sleep to maintain loop rate
            self.rate.sleep()

        # Stop the robot once goal is reached
        vel_msg.linear.x = 0.0
        vel_msg.angular.z = 0.0
        self.velocity_publisher.publish(vel_msg)

def main(args=None):
    rclpy.init(args=args)
    robot_controller = RobotController()

    try:
        robot_controller.move_to_goal()
    except KeyboardInterrupt:
        pass

    # Destroy the node explicitly (optional)
    robot_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
