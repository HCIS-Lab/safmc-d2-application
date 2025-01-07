from rclpy.node import Node
from transitions import Machine
from std_msgs.msg import Empty

from .states import States, transitions


class Drone(Machine):
    def __init__(self, node: Node):
        super().__init__(self, states=States,
                         transitions=transitions, initial=States.IDLE)
        
        self.group_id = 0

        node.wait_publisher = node.create_publisher(Empty, 'Wait', 10)
        node.wait_signal_subscription = node.create_subscription(
                Empty, f'/pair{self.group_id}/Signal', self.__set_signal, 10)
    
    def __set_signal(self, msg: Empty):
        self.__signal = True

    def get_drop_signal(self):
        return self.__signal
    
    def publish_wait(self):
        signal_msg = Empty()
        self.wait_publisher.publish(signal_msg)
    