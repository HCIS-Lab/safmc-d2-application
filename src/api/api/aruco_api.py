from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from rclpy.time import Time

from agent_msgs.msg import ArucoInfo
from common.coordinate import Coordinate

from .api import Api


class ArucoApi(Api):

    _target_marker_id = 6
    _is_marker_detected = False
    _marker_position = Coordinate(0, 0, 0)

    def __init__(self, node: Node):
        # Initial Values
        self._clock = node.get_clock()
        self._latest_msg_time = self._clock.now()

        # QoS
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Subscriptions
        node.create_subscription(
            ArucoInfo, f"aruco_info", self.__aruco_info_callback, qos_profile
        )

    def reset(self):
        self._is_marker_detected = False

    def set_target_marker_id(self, target_marker_id):
        self._target_marker_id = target_marker_id

    @property
    def is_marker_detected(self) -> bool:
        return self._is_marker_detected

    @property
    def marker_position(self) -> Coordinate:
        return self._marker_position

    @property
    def idle_time(self) -> Time:
        return self._clock.now() - self._latest_msg_time

    @property
    def latest_timestamp(self) -> Time:
        return self._latest_msg_time

    def __aruco_info_callback(self, aruco_info_msg):
        id = aruco_info_msg.id

        if id != self._target_marker_id:
            return

        self._is_marker_detected = True
        self._marker_position.x = aruco_info_msg.position.x
        self._marker_position.y = aruco_info_msg.position.y
        self._marker_position.z = aruco_info_msg.position.z
        self._latest_msg_time = self._clock.now()
