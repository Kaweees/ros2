import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32


class IntegerPublisher(Node):
    def __init__(self):
        super().__init__("integer_publisher")
        self.publisher_ = self.create_publisher(Int32, "integer_topic", 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Int32()
        msg.data = self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%d"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)
    integer_publisher = IntegerPublisher()
    rclpy.spin(integer_publisher)
    integer_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
