import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import argparse
import sys

class RobotController(Node):
    def __init__(self, goal_type, goal):
        super().__init__('motion_control')

        # Set up publisher and subscriber
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Robot state
        self.current_pose = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        self.goal_type = goal_type  # 'point', 'pose', or 'path'
        self.goal = goal

        # Proportional gains
        self.linear_kp = 0.3
        self.angular_kp = 1

        self.timer = self.create_timer(0.1, self.control_loop)

        # Goal progress
        self.goal_reached = False

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        self.current_pose['x'] = position.x
        self.current_pose['y'] = position.y

        # Directly use the yaw value from the odometry message
        self.current_pose['theta'] = self.get_yaw_from_odom(msg)

    def get_yaw_from_odom(self, msg):
        """Extract yaw directly from the orientation in the odometry message."""
        orientation = msg.pose.pose.orientation
        siny_cosp = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def control_to_point(self, goal_x, goal_y):
        dx = goal_x - self.current_pose['x']
        dy = goal_y - self.current_pose['y']
        distance = math.sqrt(dx**2 + dy**2)
        goal_pose = math.atan2(dy, dx)

        curr_pose = self.current_pose['theta']
        print(f'goal_pose: {goal_pose} self_pose: {curr_pose}')

        angle_to_goal = self.wrap_angle(goal_pose - self.current_pose['theta'])


        # if curr_pose > 0:
            # angle_to_goal = goal_pose - self.current_pose['theta']
        # else:
            # angle_to_goal = self.current_pose['theta'] - (goal_pose + self.current_pose['theta'])
        # 
        # self.get_logger().info(str(angle_to_goal))

        # Stop if close to the goal
        if distance < 0.03:
            self.publish_velocity(0.0, 0.0)
            self.goal_reached = True
            self.get_logger().info(f"Reached point [{goal_x}, {goal_y}]")
            # sys.exit()
            return

        # Proportional control
        linear_velocity = min(self.linear_kp * distance, 0.2) 
        angular_velocity = max(-1.0, min(self.angular_kp * angle_to_goal, 1.0))
       
       
        # if abs(angle_to_goal) > 0.1:  # Alignment threshold
        #     # Rotate to face the goal
        #     linear_velocity = 0.0
        #     angular_velocity = max(-1.0, min(self.angular_kp * angle_to_goal, 1.0))
        # else:
        #     # Move forward
        #     linear_velocity = min(self.linear_kp * distance, 0.2)  # Max TurtleBot3 speed: 0.22 m/s
        #     angular_velocity = 0.0

        # angular_velocity = self.angular_kp * angle_to_goal

        self.publish_velocity(linear_velocity, angular_velocity)

    def wrap_angle(self, angle):
        """Wrap an angle to the range [-pi, pi]."""
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def control_to_pose(self, goal_x, goal_y, goal_theta):
        dx = goal_x - self.current_pose['x']
        dy = goal_y - self.current_pose['y']
        distance = math.sqrt(dx**2 + dy**2)
        angle_to_goal = math.atan2(dy, dx) - self.current_pose['theta']

        # Stop if close to the goal position and orientation
        theta_error = goal_theta - self.current_pose['theta']
        if distance < 0.05 and abs(theta_error) < 0.05:
            self.publish_velocity(0.0, 0.0)
            self.goal_reached = True
            self.get_logger().info(f"Reached pose [{goal_x}, {goal_y}, {goal_theta}]")
            sys.exit()
            return

        # Proportional control
        if distance > 0.05:
            linear_velocity = min(self.linear_kp * distance, 0.2)
            angular_velocity = max(-2.5, min(self.angular_kp * angle_to_goal, 2.5))
        else:
            linear_velocity = 0.0
            angular_velocity = max(-2.5, min(self.angular_kp * theta_error, 2.5))
        self.publish_velocity(linear_velocity, angular_velocity)

    def follow_path(self, waypoints):
        if not waypoints:
            self.goal_reached = True
            self.get_logger().info("Completed path")
            sys.exit()
            return

        # Follow the first waypoint
        goal_x, goal_y = waypoints[0]
        self.control_to_point(goal_x, goal_y)
        if self.goal_reached:
            self.goal_reached = False
            waypoints.pop(0)  # Remove the reached waypoint
            self.goal = waypoints

    def publish_velocity(self, linear, angular):
        print(angular)
        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        self.vel_pub.publish(cmd)

    def control_loop(self):
        if self.goal_reached:
            return

        if self.goal_type == 'point':
            self.control_to_point(self.goal[0], self.goal[1])
        elif self.goal_type == 'pose':
            self.control_to_pose(self.goal[0], self.goal[1], self.goal[2])
        elif self.goal_type == 'path':
            self.follow_path(self.goal)

def main():
    rclpy.init()

    parser = argparse.ArgumentParser()
    parser.add_argument("--type", type=str, choices=["point", "pose", "path"], required=True,
                        help="Goal type: 'point', 'pose', or 'path'")
    parser.add_argument("--goal", type=float, nargs='+', required=True,
                        help="Goal coordinates [x y] for 'point', [x y theta] for 'pose', or a list of waypoints for 'path'")
    args = parser.parse_args()

    goal_type = args.type
    goal = args.goal

    if goal_type == "path":
        # Parse waypoints for the path
        if len(goal) % 2 != 0:
            print("Invalid path waypoints. Provide x, y pairs.")
            return
        goal = [(goal[i], goal[i + 1]) for i in range(0, len(goal), 2)]

    controller = RobotController(goal_type, goal)

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass

    controller.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
