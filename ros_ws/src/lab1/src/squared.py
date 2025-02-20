import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32


class NumberSquarer(Node):
    def __init__(self):
        super().__init__("number_squarer")
        # Subscribe to input numbers
        self.subscription = self.create_subscription(
            Int32, "integer_topic", self.input_callback, 10
        )
        # Publish squared numbers
        self.publisher_ = self.create_publisher(Int32, "squared_topic", 10)
        self.subscription  # prevent unused variable warning

    def input_callback(self, msg):
        # Square the received number
        input_number = msg.data
        squared = input_number * input_number

        # Publish the squared number
        squared_msg = Int32()
        squared_msg.data = squared
        self.publisher_.publish(squared_msg)
        self.get_logger().info(
            "Received: %d, Publishing squared: %d" % (input_number, squared)
        )


def main(args=None):
    rclpy.init(args=args)
    number_squarer = NumberSquarer()
    rclpy.spin(number_squarer)
    number_squarer.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
