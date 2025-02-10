import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import math


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

        # Think of this flag as a FSM,
        # or that the turtle has two modes of operation.
        # The robot is either turning in place, or not turning in place
        # i.e. moving forward.
        self.turning: bool = False

        # Let's create two messages to send to the robot depending
        # on the mode its in.
        # What should their type be (should the same as 'Import' above)

        # If I want the robot to move "1m forward" what should
        # the speed be, given the timer is running at 1hz?
        # (Note that values are in m/s)
        # Along which axis should I move in?
        self.forward_msg = Twist()  # move 1m forward
        self.forward_msg.linear.x = 1.0
        self.forward_msg.linear.y = 0.0
        self.forward_msg.linear.z = 0.0
        self.forward_msg.angular.x = 0.0
        self.forward_msg.angular.y = 0.0
        self.forward_msg.angular.z = 0.0

        # What if I want the robot to turn 90 degrees?
        # Along which axis?
        self.turn_msg = Twist()  # turn 90 degrees
        self.turn_msg.linear.x = 0.0
        self.turn_msg.linear.y = 0.0
        self.turn_msg.linear.z = 0.0
        self.turn_msg.angular.x = 0.0
        self.turn_msg.angular.y = 0.0
        self.turn_msg.angular.z = math.pi / 2  # (π/2 rad/s)

    # Callback for the events
    def timer_callback(self):
        # If robot is turning
        if self.turning:
            # Call publisher here
            self.get_logger().info("Robot is Turning!")
            self.publisher_.publish(self.turn_msg)
        else:
            # Call publisher here
            self.get_logger().info("Robot is Moving Forward!")
            self.publisher_.publish(self.forward_msg)

        # Get logger function call similar to 'cout' in C++
        # or 'print()' in python

        # Flip the mode of the robot
        self.turning = not self.turning


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
