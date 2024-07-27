import launch
import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    # Parameters
    lifecycle_nodes = ['map_server']
    use_sim_time = True
    autostart = True

    map_file = os.path.join(get_package_share_directory(
        "trajectory_tracking_control"), "maps", "map.yaml")

    return launch.LaunchDescription([
        Node(package="nav2_map_server",
             executable="map_server",
             name="map_server",
             output="screen",
             parameters=[{"use_sim_time": use_sim_time},
                         {"yaml_filename": map_file}],
             emulate_tty=True),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            output='screen',
            emulate_tty=True,
            parameters=[{'use_sim_time': True},
                        {'autostart': autostart},
                        {'node_names': lifecycle_nodes}])
    ])
