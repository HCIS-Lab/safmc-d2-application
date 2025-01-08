from rclpy.node import Node
from std_msgs.msg import Empty, Int8

from .api import Api


class MediatorApi(Api):
    def __init__(self, node: Node):

        # TODO: read from params.yaml
        node.declare_parameters(
            parameters=[
                ('drone_id', -1),
                ('group_id', -1)
            ])
        self.drone_id = node.get_parameter('drone_id').get_parameter_value().integer_value
        self.group_id = node.get_parameter('group_id').get_parameter_value().integer_value
        if self.drone_id == -1 or self.group_id == -1:
            node.get_logger().error("Failed to import parameters from YAML!")

        # subscriptions
        self.__signal = False
        self.signal_subscription = node.create_subscription(
            Empty, f'/group{self.group_id}/signal', self.__set_signal, 10)

        # publishers
        self.wait_publisher = node.create_publisher(
            Int8, f'/group{self.group_id}/wait', 10)

    def wait_to_drop(self):
        signal_msg = Int8()
        signal_msg.data = self.drone_id
        self.wait_publisher.publish(signal_msg)

    def __set_signal(self, msg: Empty):
        self.__signal = True

    def get_signal(self):
        return self.__signal
