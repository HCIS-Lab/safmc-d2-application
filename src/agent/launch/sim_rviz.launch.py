import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def prepare_rviz_node(context, *args, **kwargs):

    drone_id_lc = LaunchConfiguration("drone_id")
    view_mode_lc = LaunchConfiguration("view_mode")
    drone_id = int(drone_id_lc.perform(context))
    view_mode = str(view_mode_lc.perform(context))

    rviz_config_dir = os.path.join(
        get_package_share_directory('agent'), 'config',
    )
    original_rviz_config_path = os.path.join(rviz_config_dir, 'rviz2_config.rviz')
    modified_rviz_config_path = os.path.join(rviz_config_dir, 'rviz2_config_modified.rviz')

    if drone_id < 1 or drone_id > 4:
        message = LogInfo(
            msg="ERROR: Drone ID must be between 1 and 4, not launching any."
        )
    elif view_mode not in ['lidar', 'camera']:
        message = LogInfo(
            msg="ERROR: View mode must be lidar or camera, not launching any."
        )
    else:
        message = LogInfo(msg=f"Starting {view_mode} rviz node for drone {drone_id}")

        with open(original_rviz_config_path, 'r') as file:
            rviz_config = yaml.safe_load(file)

        if view_mode == "camera":
            fixed_frame = f"x500_safmc_d2_{drone_id}/pi3_cam_link/pi3_cam_sensor"
            lidar_enabled = False
            camera_enabled = True
        else:
            fixed_frame = f"x500_safmc_d2_{drone_id}/lidar_2d_link/lidar_2d_sensor"
            lidar_enabled = True
            camera_enabled = False

        rviz_config["Visualization Manager"]["Global Options"]["Fixed Frame"] = fixed_frame
        for display in rviz_config["Visualization Manager"]["Displays"]:
            if display["Class"] == "rviz_default_plugins/LaserScan":
                display["Topic"]["Value"] = f"/world/safmc_d2/model/x500_safmc_d2_{drone_id}/link/lidar_2d_link/sensor/lidar_2d_sensor/scan"
                display["Enabled"] = lidar_enabled
            elif display["Class"] == "rviz_default_plugins/Camera":
                display["Topic"]["Value"] = f"/world/safmc_d2/model/x500_safmc_d2_{drone_id}/link/pi3_cam_link/sensor/pi3_cam_sensor/image"
                display["Enabled"] = camera_enabled

        with open(modified_rviz_config_path, 'w') as file:
            yaml.dump(rviz_config, file)

        # Nodes for rviz2
        rviz_node = [
            Node(
                package='rviz2',
                executable='rviz2',
                namespace=f'agent_{drone_id}',
                output='screen',
                arguments=['-d', modified_rviz_config_path, '--ros-args', '--log-level', 'fatal'])
        ]

        return rviz_node + [message]


def generate_launch_description():

    declared_args = [
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

    return LaunchDescription(
        declared_args +
        [OpaqueFunction(function=prepare_rviz_node)])
