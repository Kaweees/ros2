from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="lab1",
                executable="listener",
                name="listener",
            ),
            Node(
                package="lab1",
                executable="talker",
                name="talker",
            ),
            Node(
                package="lab1",
                executable="squared",
                name="squared",
            ),
        ]
    )
