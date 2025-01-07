from rclpy.node import Node
from transitions import Machine
from std_msgs.msg import Empty, Int8

from .states import States, transitions


class Drone(Machine):
    def __init__(self, node: Node):
        super().__init__(self, states=States,
                         transitions=transitions, initial=States.IDLE)
        
        self.group_id = 0

        node.wait_publisher = node.create_publisher(Int8, f'/pair{self.group_id}/wait', 10)
        node.wait_signal_subscription = node.create_subscription(
                Empty, f'/pair{self.group_id}/signal', self.__set_signal, 10)
    
    def publish_wait(self):
        signal_msg = Int8()
        signal_msg.data = self.drone_id
        self.wait_publisher.publish(signal_msg)

    def __set_signal(self, msg: Empty):
        self.__signal = True

    def get_drop_signal(self):
        return self.__signal
    