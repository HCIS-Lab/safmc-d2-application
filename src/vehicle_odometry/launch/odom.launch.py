import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('vehicle_odometry')
    params_path = os.path.join(pkg_dir, 'config', 'params.yaml')

    return LaunchDescription([
        Node(
            package='vehicle_odometry',
            executable='vehicle_odometry',
            name='vehicle_odometry',
            output='screen',
            parameters=[params_path]
        ),
    ])

