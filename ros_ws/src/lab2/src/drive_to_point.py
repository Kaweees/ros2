import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
import time


class DriveToPoint(Node):
    def __init__(self):
        super().__init__("drive_to_point")

        # Create publisher for cmd_vel topic
        self.publisher_ = self.create_publisher(Twist, "turtle1/cmd_vel", 10)

        # Create timer for movement control
        timer_period = 0.1  # 10Hz for smoother motion
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # State machine states
        self.ROTATE_TO_POINT = 0
        self.MOVE_TO_POINT = 1
        self.ROTATE_LEFT = 2
        self.FINISHED = 3

        # Initialize state
        self.current_state = self.ROTATE_TO_POINT
        self.start_time = time.time()

        # Target point
        self.target_x = 1.0
        self.target_y = 1.0

        # Calculate initial rotation angle (towards point 1,1)
        # arctan(1/1) = 45 degrees = pi/4 radians
        self.initial_rotation = math.pi / 4

        # Distance to move (sqrt(1^2 + 1^2) = sqrt(2))
        self.distance = math.sqrt(2)

    def timer_callback(self):
        msg = Twist()
        elapsed_time = time.time() - self.start_time

        if self.current_state == self.ROTATE_TO_POINT:
            # Rotate 45 degrees (pi/4 radians) to face the point
            if elapsed_time < 0.5:  # Time to complete rotation
                msg.angular.z = self.initial_rotation * 2  # Complete in 0.5 seconds
                self.get_logger().info("Rotating towards point")
            else:
                self.current_state = self.MOVE_TO_POINT
                self.start_time = time.time()

        elif self.current_state == self.MOVE_TO_POINT:
            # Move to point (1,1)
            if elapsed_time < self.distance:  # Move at 1 m/s
                msg.linear.x = 1.0
                self.get_logger().info("Moving to point")
            else:
                self.current_state = self.ROTATE_LEFT
                self.start_time = time.time()

        elif self.current_state == self.ROTATE_LEFT:
            # Rotate to face left (-135 degrees from current position)
            if elapsed_time < 1.35:  # Time to rotate -135 degrees
                msg.angular.z = -2.0  # Rotate at 2 rad/s
                self.get_logger().info("Rotating to face left")
            else:
                self.current_state = self.FINISHED

        elif self.current_state == self.FINISHED:
            # Stop all motion
            self.get_logger().info("Finished movement")

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    drive_to_point = DriveToPoint()
    rclpy.spin(drive_to_point)
    drive_to_point.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
