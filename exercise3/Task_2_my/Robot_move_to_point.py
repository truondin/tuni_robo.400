import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import sys

class MotionControlNode(Node):
    def __init__(self):
        super().__init__('motion_control_node')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.goal_x = None
        self.goal_y = None
        self.goal_theta = None
        self.goal_reached = False  # To track if the goal has been reached

        self.declare_parameters(
            namespace='',
            parameters=[
                ('goal_x', None),
                ('goal_y', None),
                ('goal_theta', None),
            ]
        )

    def odom_callback(self, msg):
        # Extract current position and orientation
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        theta = self.euler_from_quaternion(msg.pose.pose.orientation)

        if not self.goal_reached and self.goal_x is not None and self.goal_y is not None:
            self.go_to_goal(x, y, theta)

    def go_to_goal(self, x, y, theta):
        goal_theta = self.goal_theta if self.goal_theta is not None else theta
        distance = math.sqrt((self.goal_x - x)**2 + (self.goal_y - y)**2)
        angle_to_goal = math.atan2(self.goal_y - y, self.goal_x - x)
        angle_error = angle_to_goal - theta

        # Control for position
        linear_speed = 0.5 * distance
        angular_speed = 0.5 * angle_error

        # If the robot is close enough to the goal, stop moving
        if distance < 0.5:
            linear_speed = 0.0
            angular_speed = 0.0
            self.goal_reached = True
            self.get_logger().info("Goal reached! Stopping the robot.")
            self.stop_robot()  # Stop the robot and shutdown
        

        self.publish_command(linear_speed, angular_speed)

    def euler_from_quaternion(self, orientation):
        # Calculate yaw from quaternion
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y**2 + orientation.z**2)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

    def publish_command(self, linear, angular):
        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        self.publisher_.publish(cmd)

    def stop_robot(self):
        # Send zero velocities to stop the robot
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.publisher_.publish(cmd)

        # Shutdown the node after stopping
        self.get_logger().info("Shutting down node.")
        quit()

def main(args=None):
    rclpy.init(args=args)
    node = MotionControlNode()

    # Accept goal arguments for Task 1 and Task 2
    if len(sys.argv) >= 3:
        node.goal_x = float(sys.argv[1])
        node.goal_y = float(sys.argv[2])
        if len(sys.argv) == 4:
            node.goal_theta = float(sys.argv[3])

    rclpy.spin(node)
    node.destroy_node()  # Ensure node is properly destroyed after stopping
    rclpy.shutdown()

if __name__ == '__main__':
    main()
