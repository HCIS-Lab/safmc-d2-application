from rclpy.node import Node
from rclpy.time import Time

from agent_msgs.msg import ArucoPose
from common.coordinate import Coordinate
from common.qos import sensor_qos_profile

from .api import Api


class ArucoApi(Api):

    _target_marker_id = 6
    _is_marker_detected = False
    _marker_position = Coordinate(0, 0, 0)

    def __init__(self, node: Node):
        # Initial Values
        self._clock = node.get_clock()
        self._latest_msg_time = self._clock.now()

        # Subscriptions
        node.create_subscription(
            ArucoPose, f"aruco_pose", self.__aruco_pose_callback, sensor_qos_profile
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

    def __aruco_pose_callback(self, msg):
        marker_id = msg.aruco_marker_id

        if marker_id != self._target_marker_id:
            return

        self._is_marker_detected = True
        self._marker_position.x = msg.position.x
        self._marker_position.y = msg.position.y
        self._marker_position.z = msg.position.z
        self._latest_msg_time = self._clock.now()
