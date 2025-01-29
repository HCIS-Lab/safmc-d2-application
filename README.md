# SAFMC D2 Application

ROS 2-based control system for the SAFMC D2 competition, implementing the main flight logic.

## RUN

```bash
git submodule update --init # px4_msgs

colcon build
source install/setup.bash
ros2 run agent agent
```
