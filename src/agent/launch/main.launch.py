from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    agent_dir = get_package_share_directory('agent')
    params_path = os.path.join(agent_dir, 'config', 'params.yaml')

    return LaunchDescription([
        Node(
            package='agent',
            executable='agent',
            name='agent',
            output='screen',
            parameters=[params_path]
        )
    ])