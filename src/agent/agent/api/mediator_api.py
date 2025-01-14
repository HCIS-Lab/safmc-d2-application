from .api import Api

from rclpy.node import Node
from std_msgs.msg import Empty, Int8
from agent.common.parameters import get_parameter

from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy
)


class MediatorApi(Api):
    def __init__(self, node: Node):

        self.drone_id = get_parameter(node, 'drone_id', 0)
        self.group_id = get_parameter(node, 'group_id', 0)

        # TODO: check param (drone_id, group_id) is unique

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
