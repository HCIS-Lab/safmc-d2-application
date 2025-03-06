# SAFMC D2 Application

ROS 2-based control system for the SAFMC D2 competition, implementing the main flight logic.

## Prerequisite

```bash
pip install -r requirements.txt
git submodule update --init # px4_msgs
```

## Simulation

```bash
colcon build
source install/setup.bash

# for agent
ros2 launch agent sim_all.launch.py         num_drones:=4 drone_id:=1 view_mode:=lidar
# ros2 launch agent sim_bridge.launch.py    num_drones:=4
# ros2 launch agent sim_agent.launch.py     num_drones:=4
# ros2 launch agent sim_rviz.launch.py      drone_id:=1 view_mode:=lidar

# for mediator
ros2 run mediator mediator
```

---

## 補充

### Gazebo-ROS2 bridge

> [!IMPORTANT]  
> 如果是使用前面提到的 launch file 執行, 就不需要跑這些指令了!

```bash
# ref: https://github.com/gazebosim/ros_gz/blob/ros2/ros_gz_bridge/README.md

# single drone
ros2 run ros_gz_bridge parameter_bridge \
/world/safmc_d2/model/x500_safmc_d2_1/link/lidar_2d_link/sensor/lidar_2d_sensor/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan \
/world/safmc_d2/model/x500_safmc_d2_1/link/lidar_2d_link/sensor/lidar_2d_sensor/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked \
/world/safmc_d2/model/x500_safmc_d2_1/link/pi3_cam_link/sensor/pi3_cam_sensor/image@sensor_msgs/msg/Image[gz.msgs.Image \
/world/safmc_d2/model/x500_safmc_d2_1/link/pi3_cam_link/sensor/pi3_cam_sensor/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo

# 4 drones (i = 1 ~ 4)
ros2 run ros_gz_bridge parameter_bridge \
/world/safmc_d2/model/x500_safmc_d2_1/link/lidar_2d_link/sensor/lidar_2d_sensor/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan \
/world/safmc_d2/model/x500_safmc_d2_1/link/lidar_2d_link/sensor/lidar_2d_sensor/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked \
/world/safmc_d2/model/x500_safmc_d2_1/link/pi3_cam_link/sensor/pi3_cam_sensor/image@sensor_msgs/msg/Image[gz.msgs.Image \
/world/safmc_d2/model/x500_safmc_d2_1/link/pi3_cam_link/sensor/pi3_cam_sensor/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo \
/world/safmc_d2/model/x500_safmc_d2_2/link/lidar_2d_link/sensor/lidar_2d_sensor/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan \
/world/safmc_d2/model/x500_safmc_d2_2/link/lidar_2d_link/sensor/lidar_2d_sensor/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked \
/world/safmc_d2/model/x500_safmc_d2_2/link/pi3_cam_link/sensor/pi3_cam_sensor/image@sensor_msgs/msg/Image[gz.msgs.Image \
/world/safmc_d2/model/x500_safmc_d2_2/link/pi3_cam_link/sensor/pi3_cam_sensor/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo \
/world/safmc_d2/model/x500_safmc_d2_3/link/lidar_2d_link/sensor/lidar_2d_sensor/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan \
/world/safmc_d2/model/x500_safmc_d2_3/link/lidar_2d_link/sensor/lidar_2d_sensor/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked \
/world/safmc_d2/model/x500_safmc_d2_3/link/pi3_cam_link/sensor/pi3_cam_sensor/image@sensor_msgs/msg/Image[gz.msgs.Image \
/world/safmc_d2/model/x500_safmc_d2_3/link/pi3_cam_link/sensor/pi3_cam_sensor/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo \
/world/safmc_d2/model/x500_safmc_d2_4/link/lidar_2d_link/sensor/lidar_2d_sensor/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan \
/world/safmc_d2/model/x500_safmc_d2_4/link/lidar_2d_link/sensor/lidar_2d_sensor/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked \
/world/safmc_d2/model/x500_safmc_d2_4/link/pi3_cam_link/sensor/pi3_cam_sensor/image@sensor_msgs/msg/Image[gz.msgs.Image \
/world/safmc_d2/model/x500_safmc_d2_4/link/pi3_cam_link/sensor/pi3_cam_sensor/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo
```

## 參考

- [Launch file 撰寫](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Using-ROS2-Launch-For-Large-Projects.html)
