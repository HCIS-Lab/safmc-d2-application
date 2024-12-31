from rclpy.node import Node
from transitions import Machine
from agent_msgs.msg import Drop

from .states import States, transitions


class Drone(Machine):
    def __init__(self, node: Node):
        super().__init__(self, states=States,
                         transitions=transitions, initial=States.IDLE)
        self.drop_signal_subscription = node.create_subscription(
            Drop, 'drop_signal', self.__set_drop_signal, 10)
        
    def __set_drop_signal(self, drop_msg: Drop):
        self.__drop_signal = (drop_msg.drop)

    def get_drop_signal(self):
        return self.__drop_signal

    def camera_blocked(self):
        pass

    def drop_payload(self):
        pass