from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from std_msgs.msg import Bool

from agent_msgs.msg import MagnetControl

from .api import Api


class MagnetApi(Api):
    def __init__(self, node: Node):

        # Initial Values
        self.__is_loaded = False

        # TODO: qos_policy (Copied from autositter repo, might not fit this project)
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscriptions
        self.is_loaded_sub = node.create_subscription(
            Bool,
            "is_loaded",
            self.__set_is_loaded,
            qos_profile
        )

        # Publishers
        self.magnet_control_pub = node.create_publisher(
            MagnetControl,
            # payload system subscribe to /drone_{i}/magnet_control, for i from 0 to 3
            "magnet_control",
            qos_profile
        )

    @property
    def is_loaded(self) -> bool:
        return self.__is_loaded

    def __set_is_loaded(self, is_loaded_msg: Bool):
        self.__is_loaded = is_loaded_msg.data

    def activate_magnet(self) -> None:
        magnet_control_msg = MagnetControl()
        magnet_control_msg.magnet1 = True
        magnet_control_msg.magnet2 = False
        magnet_control_msg.magnet3 = False
        self.magnet_control_pub.publish(magnet_control_msg)

    def deactivate_magnet(self) -> None:
        magnet_control_msg = MagnetControl()
        magnet_control_msg.magnet1 = False
        magnet_control_msg.magnet2 = False
        magnet_control_msg.magnet3 = False
        self.magnet_control_pub.publish(magnet_control_msg)
