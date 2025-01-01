from rclpy.node import Node
from transitions import Machine
from agent_msgs.msg import Wait

from .states import States, transitions


class Drone(Machine):
    def __init__(self, node: Node):
        super().__init__(self, states=States,
                         transitions=transitions, initial=States.IDLE)
        self.wait_signal_subscription = node.create_subscription(
            Wait, 'drop_signal_metiator', self.__set_can_drop, 10)
    
    def __set_can_drop(self, msg: Wait):
        self.__can_drop = (msg.wait)

    def get_drop_signal(self):
        return self.__can_drop
    