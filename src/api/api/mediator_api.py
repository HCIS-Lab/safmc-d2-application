from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from std_msgs.msg import Empty, Int8

from .api import Api


class MediatorApi(Api):
    def __init__(self, node: Node, drone_id: int, group_id: int):

        self.drone_id = drone_id
        self.group_id = group_id

        # TODO: qos_policy (Copied from autositter repo, might not fit this project)
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscriptions
        self.__signal = False
        self.signal_sub = node.create_subscription(
            Empty, f'/group{self.group_id}/signal', self.__set_signal, qos_profile)

        # Publishers
        self.wait_pub = node.create_publisher(
            Int8, f'/group{self.group_id}/wait', qos_profile)

    def wait_to_drop(self):
        signal_msg = Int8()
        signal_msg.data = self.drone_id
        self.wait_pub.publish(signal_msg)

    @property
    def signal(self):
        return self.__signal

    def __set_signal(self, msg: Empty):
        self.__signal = True
