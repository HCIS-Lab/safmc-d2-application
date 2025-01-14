from .api import Api

from agent.config import Config
from rclpy.node import Node
from std_msgs.msg import Empty, Int8

from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy
)


class MediatorApi(Api):
    def __init__(self, node: Node, config: Config):

        # TODO: read from params.yaml
        self.drone_id = config.get_int_parameter('drone_id')
        self.group_id = config.get_int_parameter('group_id')

        if self.drone_id == -1 or self.group_id == -1:
            node.get_logger().error("Failed to import parameters from YAML!")

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
