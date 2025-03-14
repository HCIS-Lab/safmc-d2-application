import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    drone_id = int(os.environ["DRONE_ID"])

    return LaunchDescription(
        [
            Node(
                package="agent",
                executable="agent",
                namespace=f"px4_{drone_id}",
                output="screen",
                parameters=[{"drone_id": drone_id}],
            ),
            Node(
                package="agent",
                executable="aruco_tracker",
                namespace=f"px4_{drone_id}",
                output="screen",
                parameters=[{"drone_id": drone_id}],
            ),
        ]
    )
