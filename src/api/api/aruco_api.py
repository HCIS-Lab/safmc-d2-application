from rclpy.node import Node
from rclpy.time import Time

from safmc_msgs.msg import ArucoPose
from common.coordinate import Coordinate
from common.qos import sensor_qos_profile
from scipy.spatial.transform import Rotation

from .api import Api

HEADING_ARUCO_IDS = [0, 1, 2, 3, 4, 5, 6, 7]


def quaternion_to_yaw(q: list):
    r = Rotation.from_quat([q[0], q[1], q[2], q[3]])  # (x, y, z, w)
    euler = r.as_euler("zyx", degrees=False)  # 取得 (yaw, pitch, roll)
    return euler[0]  # Yaw 角


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
            ArucoPose, f"aruco_pose", self.__aruco_pose_callback, sensor_qos_profile
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

    def __aruco_pose_callback(self, msg):
        marker_id = msg.aruco_marker_id

        # TODO[lnfu] refactor
        if marker_id in HEADING_ARUCO_IDS:
            self._heading = quaternion_to_yaw(
                [
                    msg.orientation.x,
                    msg.orientation.y,
                    msg.orientation.z,
                    msg.orientation.w,
                ]
            )

        if marker_id != self._target_marker_id:
            return

        self._is_marker_detected = True
        self._detected_time = self._clock.now()
        self._detected_marker_position.x = msg.position.x
        self._detected_marker_position.y = msg.position.y
        self._detected_marker_position.z = msg.position.z
