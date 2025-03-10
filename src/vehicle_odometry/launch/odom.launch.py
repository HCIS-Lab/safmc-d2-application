import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription(
        [
            Node(
                package="vehicle_odometry",
                executable="vehicle_odometry",
                name="vehicle_odometry",
                output="screen",
            ),
        ]
    )
