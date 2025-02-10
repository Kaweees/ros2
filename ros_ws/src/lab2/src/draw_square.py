import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time


class DrawSquare(Node):
    def __init__(self):
        # Init the node with a name (this is the name that appears when running)
        # 'ros2 node list'
        super().__init__("draw_square")

        # Create publisher for cmd_vel topic
        self.publisher_ = self.create_publisher(
            Twist, "turtle1/cmd_vel", 10
        )  # Message Type | Topic Name | Queue Length

        # Create timer for movement control
        timer_period = 1.0  # Time between function calls (1Hz)
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Initialize state variables
        self.state = "forward"
        self.start_time = time.time()
        self.side_length_time = 2.0  # Time to move forward
        self.turn_time = 1.57  # Time to turn (approximately 90 degrees)

    def move_turtle(self):
        msg = Twist()
        current_time = time.time() - self.start_time

        if self.state == "forward":
            # Move forward
            msg.linear.x = 1.0
            msg.angular.z = 0.0
            if current_time >= self.side_length_time:
                self.state = "turn"
                self.start_time = time.time()

        elif self.state == "turn":
            # Turn 90 degrees
            msg.linear.x = 0.0
            msg.angular.z = 1.0
            if current_time >= self.turn_time:
                self.state = "forward"
                self.start_time = time.time()

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    draw_square = DrawSquare()

    # Spin function *important*
    # makes sure that the program does not terminate
    # immediately
    rclpy.spin(draw_square)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    draw_square.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
