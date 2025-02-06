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
ros2 launch agent simulation.launch.py

# for mediator
ros2 run mediator mediator
```

## Rviz2

### Arguments
- drone_id: 1~4
- view_mode: lidar/camera

```bash
colcon build
source install/setup.bash

# for lidar
ros2 launch agent rviz.launch.py drone_id:=1 view_mode:=lidar

# for camera
ros2 launch agent rviz.launch.py drone_id:=1 view_mode:=camera
```

---

## 補充

```bash
colcon build
source install/setup.bash
ros2 run agent agent
ros2 run mediator mediator
```

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
/world/safmc_d2/model/x500_safmc_d2_4/link/pi3_cam_link/sensor/pi3_cam_sensor/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo \
```
