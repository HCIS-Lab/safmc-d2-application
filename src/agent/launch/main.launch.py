import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('agent')
    params_path = os.path.join(pkg_dir, 'config', 'params.yaml')

    return LaunchDescription([
        Node(
            package='agent',
            executable='agent',
            name='agent',
            output='screen',
            parameters=[params_path]
        )
    ])
