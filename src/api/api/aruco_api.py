from rclpy.node import Node
from rclpy.time import Time

from safmc_msgs.msg import ArucoPose
from common.coordinate import Coordinate
from common.qos import sensor_qos_profile

from .api import Api


class ArucoApi(Api):

    _target_marker_id: int = None

    _is_marker_detected = False
    _detected_time: Time = None
    _detected_marker_position_diff: Coordinate = None

    def __init__(self, node: Node):
        self._clock = node.get_clock()

        # Subscriptions
        node.create_subscription(
            ArucoPose, f"aruco_pose", self.__aruco_pose_callback, sensor_qos_profile
        )

    def reset(self):
        self._is_marker_detected = False
        self._detected_time = None
        self._detected_marker_position_diff = None

    def set_target_marker_id(self, target_marker_id):
        self._target_marker_id = target_marker_id

    @property
    def is_marker_detected(self) -> bool:
        return self._is_marker_detected

    @property
    def elapsed_time(self) -> Time:
        return self._clock.now() - self._detected_time

    @property
    def marker_position_diff(self) -> Coordinate:
        """
        marker_position_diff 是 aruco marker 相對飛機的位置

        換言之, local_position + marker_position_diff 就是 marker 在飛機 local 座標系下的位置
        """
        return self._detected_marker_position_diff

    def __aruco_pose_callback(self, msg):
        marker_id = msg.aruco_marker_id

        if marker_id != self._target_marker_id:
            return

        self._is_marker_detected = True
        self._detected_time = self._clock.now()
        self._detected_marker_position_diff.x = msg.position.x
        self._detected_marker_position_diff.y = msg.position.y
        self._detected_marker_position_diff.z = msg.position.z
