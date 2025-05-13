import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

class DrawSpiral(Node):
    def __init__(self):
        super().__init__("draw_spiral")

        # Create publisher for cmd_vel topic
        self.publisher_ = self.create_publisher(
            Twist, "turtle1/cmd_vel", 10
        )

        # Create timer for movement control
        timer_period = 0.1  # 10Hz for smoother motion
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Initialize state variables
        self.radius = 0.5  # Initial radius
        self.angular_speed = 1.0  # Constant angular speed (rad/s)
        self.growth_rate = 0.1  # How much the radius grows per second
        self.elapsed_time = 0.0

    def timer_callback(self):
        # Create Twist message
        msg = Twist()

        # Calculate current radius
        self.elapsed_time += 0.1  # Add timer period
        current_radius = self.radius + (self.growth_rate * self.elapsed_time)

        # Set angular velocity (constant)
        msg.angular.z = self.angular_speed

        # Set linear velocity (increases with radius to maintain spiral)
        msg.linear.x = current_radius * self.angular_speed

        # Publish the message
        self.publisher_.publish(msg)
        self.get_logger().info(f'Drawing spiral: radius = {current_radius:.2f}')

def main(args=None):
    rclpy.init(args=args)
    draw_spiral = DrawSpiral()
    rclpy.spin(draw_spiral)
    draw_spiral.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main() 
