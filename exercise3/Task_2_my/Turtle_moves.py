import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class TurtleBotMotion(Node):
    def __init__(self):
        super().__init__('turtlebot_motion')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        time.sleep(1)  # Allow time for the publisher to set up

    def move_square(self):
        twist = Twist()
        
        # Define commands for square pattern (forward and turn)
        motions = [
            (0.2, 0.0, 2.0),   # Move forward for 2 seconds
            (0.0, 1.57, 1.0),  # Turn 90 degrees (1.57 radians) in 1 second
            (0.2, 0.0, 2.0),   # Move forward for 2 seconds
            (0.0, 1.57, 1.0),  # Turn 90 degrees
            (0.2, 0.0, 2.0),   # Move forward for 2 seconds
            (0.0, 1.57, 1.0),  # Turn 90 degrees
            (0.2, 0.0, 2.0),   # Move forward for 2 seconds
            (0.0, 1.57, 1.0)   # Final 90-degree turn
        ]
        
        # Execute each motion
        for linear_x, angular_z, duration in motions:
            twist.linear.x = linear_x
            twist.angular.z = angular_z
            start_time = time.time()
            
            # Publish the command for the specified duration
            while time.time() - start_time < duration:
                self.publisher_.publish(twist)
                time.sleep(0.1)  # Publish every 100 ms

            # Stop the robot momentarily between motions
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher_.publish(twist)
            time.sleep(0.5)  # Short delay before next command

def main(args=None):
    rclpy.init(args=args)
    turtlebot_motion = TurtleBotMotion()
    turtlebot_motion.move_square()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
