import launch
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource


import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # TODO: 驗證 drone_id <= num_drones

    launch_dir = os.path.join(get_package_share_directory('agent'), 'launch')

    declared_args = [
        DeclareLaunchArgument(
            "num_drones",
            default_value="4",
            description='Select the number of drones'
        ),
        DeclareLaunchArgument(
            'drone_id',
            default_value='1',
            description='Select the drone id (1-4)'
        ),
        DeclareLaunchArgument(
            'view_mode',
            default_value='lidar',
            choices=['lidar', 'camera'],
            description='Choose view mode: lidar or camera'
        ),
    ]

    return launch.LaunchDescription(
        declared_args +
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(launch_dir, 'sim_bridge.launch.py'))
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(launch_dir, 'sim_agent.launch.py'))
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(launch_dir, 'sim_rviz.launch.py'))
            ),
        ]
    )
