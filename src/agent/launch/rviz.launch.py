import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node

def modify_rviz_config(context):
    config_dir = os.path.join('/workspace/safmc-d2-application/src/agent/config')
    original_rviz_config = os.path.join(config_dir, 'rviz2_config.rviz')
    modified_rviz_config = os.path.join(config_dir, 'rviz2_config_modified.rviz')

    drone_id = context.launch_configurations['drone_id']
    view_mode = context.launch_configurations['view_mode']

    with open(original_rviz_config, 'r') as file:
        rviz_config = yaml.safe_load(file)
    
    camera_topic = f"/world/safmc_d2/model/x500_safmc_d2_{drone_id}/link/pi3_cam_link/sensor/pi3_cam_sensor/image"
    lidar_topic = f"/world/safmc_d2/model/x500_safmc_d2_{drone_id}/link/lidar_2d_link/sensor/lidar_2d_sensor/scan"

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
            display["Topic"]["Value"] = lidar_topic
            display["Enabled"] = lidar_enabled
        elif display["Class"] == "rviz_default_plugins/Camera":
            display["Topic"]["Value"] = camera_topic
            display["Enabled"] = camera_enabled

    with open(modified_rviz_config, 'w') as file:
        yaml.dump(rviz_config, file)

    return [Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', modified_rviz_config]
    )]

def generate_launch_description():
    drone_id_arg = DeclareLaunchArgument(
        'drone_id',
        default_value='1',
        description='Select the drone number (1-4)'
    )
    view_mode_arg = DeclareLaunchArgument(
        'view_mode',
        default_value='lidar',
        choices=['lidar', 'camera'],
        description='Choose view mode: lidar or camera'
    )

    return LaunchDescription([
        drone_id_arg,
        view_mode_arg,
        OpaqueFunction(function=modify_rviz_config)
    ])
