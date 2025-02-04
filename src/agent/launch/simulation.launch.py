import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

NUM_DRONES = 4


def generate_launch_description():
    pkg_dir = get_package_share_directory('agent')

    # Nodes for agent
    agent_nodes = [
        Node(
            package='agent',
            executable='agent',
            name=f'agent_{i + 1}',
            output='screen',
            parameters=[os.path.join(pkg_dir, 'config', f'params.yaml.{i+1}')]
        )
        for i in range(NUM_DRONES)
    ]

    # Nodes for Gazebo-ROS2 bridge
    bridge_nodes = [
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name=f'gz_bridge_{i + 1}',
            output='screen',
            arguments=[
                f'/world/safmc_d2/model/x500_safmc_d2_{i+1}/link/lidar_2d_link/sensor/lidar_2d_sensor/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                f'/world/safmc_d2/model/x500_safmc_d2_{i+1}/link/lidar_2d_link/sensor/lidar_2d_sensor/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
                f'/world/safmc_d2/model/x500_safmc_d2_{i+1}/link/pi3_cam_link/sensor/pi3_cam_sensor/image@sensor_msgs/msg/Image[gz.msgs.Image',
                f'/world/safmc_d2/model/x500_safmc_d2_{i+1}/link/pi3_cam_link/sensor/pi3_cam_sensor/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            ]
        ) for i in range(NUM_DRONES)
    ]

    return LaunchDescription(agent_nodes + bridge_nodes)
