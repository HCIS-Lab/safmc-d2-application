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
        bridge_nodes = []

    else:
        message = LogInfo(msg=f"Starting {num_drones} ros_gz_bridge node(s)")

        # Nodes for Gazebo-ROS2 bridge
        bridge_nodes = [
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                namespace=f"px4_{i+1}",
                output="screen",
                arguments=[
                    f"/world/safmc_d2/model/x500_safmc_d2_{i+1}/link/lidar_2d_link/sensor/lidar_2d_sensor/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
                    f"/world/safmc_d2/model/x500_safmc_d2_{i+1}/link/lidar_2d_link/sensor/lidar_2d_sensor/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
                    f"/world/safmc_d2/model/x500_safmc_d2_{i+1}/link/pi3_cam_link/sensor/pi3_cam_sensor/image@sensor_msgs/msg/Image[gz.msgs.Image",
                    f"/world/safmc_d2/model/x500_safmc_d2_{i+1}/link/pi3_cam_link/sensor/pi3_cam_sensor/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
                ],
            )
            for i in range(num_drones)
        ]

    return bridge_nodes + [message]


def generate_launch_description():

    declared_args = [
        DeclareLaunchArgument(
            "num_drones",
            default_value="4",
        )
    ]

    return LaunchDescription(
        declared_args + [OpaqueFunction(function=prepare_multiple_nodes)]
    )
