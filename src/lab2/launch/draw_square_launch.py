from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="turtlesim",
                executable="turtlesim_node",
                name="turtlesim",
            ),
            Node(
                package="lab2",
                executable="draw_square",
                name="draw_square",
            ),
        ]
    )
