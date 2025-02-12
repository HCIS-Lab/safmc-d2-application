import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def prepare_multiple_nodes(context, *args, **kwargs):
    num_drones_lc = LaunchConfiguration("num_drones")
    num_drones = int(num_drones_lc.perform(context))

    if num_drones < 1 or num_drones > 4:
        message = LogInfo(
            msg="ERROR: Number of drones must be between 1 and 4, not launching any."
        )
        agent_nodes = []

    else:
        message = LogInfo(msg=f"Starting {num_drones} agent node(s)")

        # Nodes for agent
        agent_nodes = [
            Node(
                package='agent',
                executable='agent',
                namespace=f'agent_{i+1}',
                output='screen',
                parameters=[os.path.join(get_package_share_directory('agent'), 'config',
                                         f'params.yaml.{i+1}')]  # TODO: 如果 yaml 找不到, ERROR
            )
            for i in range(num_drones)
        ]

    return agent_nodes + [message]


def generate_launch_description():

    declared_args = [
        DeclareLaunchArgument(
            "num_drones",
            default_value="4",
            description='Select the number of drones'
        )
    ]

    return LaunchDescription(
        declared_args +
        [OpaqueFunction(function=prepare_multiple_nodes)])
