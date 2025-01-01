from rclpy.node import Node
from transitions import Machine
from agent_msgs.msg import CanDrop

from .states import States, transitions


class Drone(Machine):
    def __init__(self, node: Node):
        super().__init__(self, states=States,
                         transitions=transitions, initial=States.IDLE)
        if self.get_namespace() == "drone0" or self.get_namespace() == "drone1":
            self.wait_signal_subscription = node.create_subscription(
                CanDrop, '/pair0/drop_signal_metiator', self.__set_can_drop, 10)
        else:
            self.wait_signal_subscription = node.create_subscription(
                CanDrop, '/pair1/drop_signal_metiator', self.__set_can_drop, 10)
    
    def __set_can_drop(self, msg: CanDrop):
        self.__can_drop = (msg.can_drop)

    def get_drop_signal(self):
        return self.__can_drop
    