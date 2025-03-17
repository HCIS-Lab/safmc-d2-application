from rclpy.node import Node
from rclpy.time import Time

from safmc_msgs.msg import ArucoPose
from common.coordinate import Coordinate
from common.qos import sensor_qos_profile
from common.logger import Logger
from .api import Api
import numpy as np
import transforms3d


def quaternion_to_yaw(q: list) -> float:
    """Converts a quaternion to a yaw angle."""
    R = transforms3d.quaternions.quat2mat(q)
    return np.arctan2(R[1, 0], R[0, 0])


HEADING_ARUCO_ID = 69


class ArucoApi(Api):
    def __init__(self, node: Node):
        self._clock = node.get_clock()
        self._target_marker_id: int = None
        self._is_marker_detected = False
        self._detected_time: Time = None
        self._detected_marker_position: Coordinate = Coordinate()
        self._heading: float = 0.0

        # Subscriptions
        node.create_subscription(
            ArucoPose, "aruco_pose", self._aruco_pose_callback, sensor_qos_profile
        )
        node.create_subscription(
            ArucoPose,
            "aruco_pose_for_heading",
            self._aruco_pose_for_heading_callback,
            sensor_qos_profile,
        )

    def reset(self) -> None:
        """Resets the detected marker state."""
        self._is_marker_detected = False
        self._detected_time = None
        self._detected_marker_position = Coordinate()

    def set_target_marker_id(self, target_marker_id: int) -> None:
        """Sets the target marker ID for detection."""
        self._target_marker_id = target_marker_id

    @property
    def heading(self) -> float:
        return self._heading

    @property
    def is_marker_detected(self) -> bool:
        return self._is_marker_detected

    @property
    def elapsed_time(self) -> Time:
        return (
            self._clock.now() - self._detected_time if self._detected_time else Time()
        )

    @property
    def marker_position(self) -> Coordinate:
        return self._detected_marker_position

    def _aruco_pose_for_heading_callback(self, msg: ArucoPose) -> None:
        """Handles incoming ArucoPose messages for heading calculation."""
        if msg.aruco_marker_id == HEADING_ARUCO_ID:
            self._heading = quaternion_to_yaw(
                [
                    msg.orientation.x,
                    msg.orientation.y,
                    msg.orientation.z,
                    msg.orientation.w,
                ]
            )
            Logger.info(f"HEADING = {self._heading:.3f}")

    def _aruco_pose_callback(self, msg: ArucoPose) -> None:
        """Handles incoming ArucoPose messages for marker detection."""
        if msg.aruco_marker_id == self._target_marker_id:
            self._is_marker_detected = True
            self._detected_time = self._clock.now()
            self._detected_marker_position = Coordinate(
                msg.position.x, msg.position.y, msg.position.z
            )
