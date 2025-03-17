from rclpy.node import Node
from std_msgs.msg import Bool

from safmc_msgs.msg import Magnet, Payload
from common.qos import cmd_qos_profile

from .api import Api


class MagnetApi(Api):
    def __init__(self, node: Node):

        self.__is_loaded = False

        # Subscriptions
        # TODO[lnfu] ????????????? 微動 ?????????????
        self.is_loaded_sub = node.create_subscription(
            Payload, f"payload/out/payload", self.__set_is_loaded, cmd_qos_profile
        )

        # Publishers
        self.magnet_control_pub = node.create_publisher(
            Magnet,
            # payload system subscribe to /drone_{i}/magnet_control, for i from 0 to 3
            f"payload/in/magnet",
            cmd_qos_profile,
        )

    @property
    def is_loaded(self) -> bool:
        return self.__is_loaded

    def __set_is_loaded(self, is_loaded_msg: Bool):
        self.__is_loaded = is_loaded_msg.payload1

    def activate_magnet(self) -> None:
        magnet_msg = Magnet()
        magnet_msg.magnet1 = True
        magnet_msg.magnet2 = False
        magnet_msg.magnet3 = False
        self.magnet_control_pub.publish(magnet_msg)

    def deactivate_magnet(self) -> None:
        magnet_msg = Magnet()
        magnet_msg.magnet1 = False
        magnet_msg.magnet2 = False
        magnet_msg.magnet3 = False
        self.magnet_control_pub.publish(magnet_msg)
