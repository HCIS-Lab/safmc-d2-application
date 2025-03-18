from rclpy.node import Node
from rclpy.time import Time

from safmc_msgs.msg import ArucoPose
from common.coordinate import Coordinate
from common.qos import sensor_qos_profile
from scipy.spatial.transform import Rotation
from common.logger import Logger
from .api import Api
import numpy as np
import transforms3d


def quaternion_to_yaw(q: list):
    R = transforms3d.quaternions.quat2mat(q)
    yaw = np.arctan2(R[1, 0], R[0, 0])
    return yaw


HEADING_ARUCO_ID = 69


class ArucoApi(Api):

    _target_marker_id: int = None

    _is_marker_detected = False
    _detected_time: Time = None
    _detected_marker_position: Coordinate = None

    _heading: float

    def __init__(self, node: Node):
        self._clock = node.get_clock()

        # Subscriptions
        node.create_subscription(
            ArucoPose, f"aruco_pose", self._aruco_pose_callback, sensor_qos_profile
        )
        node.create_subscription(
            ArucoPose,
            f"aruco_pose_for_heading",
            self._aruco_pose_for_heading_callback,
            sensor_qos_profile,
        )

    def reset(self):
        self._is_marker_detected = False
        self._detected_time = None
        self._detected_marker_position = None

    def set_target_marker_id(self, target_marker_id):
        self._target_marker_id = target_marker_id

    @property
    def heading(self) -> float:
        return self._heading

    @property
    def is_marker_detected(self) -> bool:
        return self._is_marker_detected

    @property
    def elapsed_time(self) -> Time:
        return self._clock.now() - self._detected_time

    @property
    def marker_position(self) -> Coordinate:
        return self._detected_marker_position

    def _aruco_pose_for_heading_callback(self, msg):
        marker_id = msg.aruco_marker_id

        # Logger.info(f"RECEIVE ARUCO = {marker_id}")

        # TODO[lnfu] refactor
        if marker_id == HEADING_ARUCO_ID:
            self._heading = quaternion_to_yaw(
                [
                    msg.orientation.x,
                    msg.orientation.y,
                    msg.orientation.z,
                    msg.orientation.w,
                ]
            )
            Logger.info(
                f"orientation: x={msg.orientation.x:.3f}, y={msg.orientation.y:.3f}, z={msg.orientation.z:.3f}, w={msg.orientation.w:.3f}",
            )
            Logger.info(f"HEADING = {self._heading}")
            return

    def _aruco_pose_callback(self, msg):
        marker_id = msg.aruco_marker_id
        # Logger.info(f"RECEIVE ARUCO = {marker_id}")

        if marker_id != self._target_marker_id:
            return

        self._is_marker_detected = True
        self._detected_time = self._clock.now()
        self._detected_marker_position.x = msg.position.x
        self._detected_marker_position.y = msg.position.y
        self._detected_marker_position.z = msg.position.z
