# TODO 之後可以用 timestamp 的方式去偵測是否長時間沒收到 aruco marker 的資訊

from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from rclpy.time import Time

from agent_msgs.msg import ArucoInfo
from common.ned_coordinate import NEDCoordinate

from .api import Api


class ArucoApi(Api):
    def __init__(self, node: Node):

        # Initial Values
        self.__target_marker_id = 6
        self.__is_marker_detected = False
        self.__marker_position = NEDCoordinate(0, 0, 0)

        # QoS
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscriptions
        node.create_subscription(
            ArucoInfo,
            '/aruco_info',
            self.__aruco_info_callback,
            qos_profile
        )

        self.clock = node.get_clock()
        self.__latest_msg_time = self.clock.now()

    def reset(self):
        self.__is_marker_detected = False

    def set_target_marker_id(self, target_marker_id):
        self.__target_marker_id = target_marker_id

    @property
    def is_marker_detected(self) -> bool:
        return self.__is_marker_detected

    @property
    def marker_position(self) -> NEDCoordinate:
        return self.__marker_position
    
    @property
    def idle_time(self) -> Time:
        return self.clock.now() - self.__latest_msg_time

    @property
    def latest_timestamp(self) -> Time:
        return self.__latest_msg_time

    def __aruco_info_callback(self, aruco_info_msg):
        id = aruco_info_msg.id

        if id != self.__target_marker_id:
            return

        self.__is_marker_detected = True
        self.__marker_position.x = aruco_info_msg.position.x
        self.__marker_position.y = aruco_info_msg.position.y
        self.__marker_position.z = aruco_info_msg.position.z
        self.__latest_msg_time = self.clock.now()
        
