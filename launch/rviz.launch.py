import launch
import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    rviz_file = os.path.join(get_package_share_directory(
        "trajectory_tracking_control"), "rviz", "my.rviz")

    return launch.LaunchDescription([
        Node(package="rviz2",
             executable="rviz2",
             name="rviz2",
             arguments=['-d', rviz_file],
             output="screen")
    ])
