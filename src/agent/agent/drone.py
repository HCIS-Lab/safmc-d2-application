from rclpy.node import Node
from transitions import Machine
from agent_msgs.msg import Wait, Signal

from .states import States, transitions


class Drone(Machine):
    def __init__(self, node: Node):
        super().__init__(self, states=States,
                         transitions=transitions, initial=States.IDLE)
        
        self.wait_publisher = self.create_publisher(Wait, 'Wait', 10)
        if self.get_namespace() == "drone0" or self.get_namespace() == "drone1":
            self.wait_signal_subscription = node.create_subscription(
                Signal, '/pair0/Signal', self.__set_signal, 10)
        else:
            self.wait_signal_subscription = node.create_subscription(
                Signal, '/pair1/Signal', self.__set_signal, 10)
    
    def __set_signal(self, msg: Signal):
        self.__signal = (msg.signal)

    def get_drop_signal(self):
        return self.__signal
    
    def publish_wait(self):
        signal_msg = Wait()
        signal_msg.data = True
        self.wait_publisher.publish(signal_msg)
    