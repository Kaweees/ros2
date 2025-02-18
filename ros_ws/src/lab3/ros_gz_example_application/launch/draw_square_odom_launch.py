from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Include the simulation launch file
    pkg_ros_gz_example_bringup = get_package_share_directory("ros_gz_example_bringup")
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_example_bringup, "launch", "diff_drive.launch.py")
        )
    )

    # Launch our square drawing node
    draw_square_node = Node(
        package="ros_gz_example_application",
        executable="draw_square_odom",
        name="draw_square_odom",
    )

    return LaunchDescription([simulation, draw_square_node])
