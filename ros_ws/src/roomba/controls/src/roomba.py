import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import LaserScan
import math

class Roomba(Node):

    def __init__(self) -> None:
        super().__init__('roomba')
        
        self.publisher_ = self.create_publisher(TwistStamped, 'cmd_vel', 10)
        self.subscriber_ = self.create_subscription(LaserScan, 'scan', self.lidar_callback, 10)

        self.turning = False

        self.forward_msg = TwistStamped()
        self.forward_msg.twist.linear.x = 0.2

        self.turn_msg = TwistStamped()
        self.turn_msg.twist.angular.z = 0.2

        self.stop_msg = TwistStamped()

    def lidar_callback(self, laser_scan) -> None:
        front = self.front_reading(laser_scan, math.radians(20))

        if front < 1.0:
            print("something's close!")
            self.publisher_.publish(self.turn_msg)
        else:
            print("forward!")
            self.publisher_.publish(self.forward_msg)


    def front_reading(self, laser_scan, angle):
        num_readings = int(angle // laser_scan.angle_increment)
        middle = len(laser_scan.ranges) // 2
        ranges = laser_scan.ranges[middle - num_readings : middle + num_readings]

        filtered_ranges = []

        for r in ranges:
            if r != float("inf") and r > laser_scan.range_min and r < laser_scan.range_max:
                filtered_ranges.append(r)
        
        if len(filtered_ranges) == 0:
            return 0

        return min(filtered_ranges)


def main(args=None) -> None:
    rclpy.init(args=args)

    roomba = Roomba()
    rclpy.spin(roomba)

    roomba.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
