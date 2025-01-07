import rclpy
from rclpy.node import Node
from std_msgs.msg import Wait, Signal

class Metiator(Node):
    def __init__(self):
        super().__init__('metiator')
        self.wait_signal = [False, False, False, False]

        self.subscriber_0 = self.create_subscription(Wait, '/drone0/Wait', lambda msg: self.signal_callback(msg, 0), 10)
        self.subscriber_1 = self.create_subscription(Wait, '/drone1/Wait', lambda msg: self.signal_callback(msg, 1), 10)
        self.subscriber_2 = self.create_subscription(Wait, '/drone2/Wait', lambda msg: self.signal_callback(msg, 2), 10)
        self.subscriber_3 = self.create_subscription(Wait, '/drone3/Wait', lambda msg: self.signal_callback(msg, 3), 10)
        self.publisher_0 = self.create_publisher(Signal, '/pair0/Signal', 10)
        self.publisher_1 = self.create_publisher(Signal, '/pair1/Signal', 10)

    def signal_callback(self, msg, signal_type):
        if msg.data:
            self.wait_signal[signal_type] = True
            self.get_logger().info(f'Received signal from drone{signal_type}')
            self.check_wait_publish_drop()

    def check_wait_publish_drop(self):
        if self.wait_signal[0] and self.wait_signal[1]:
            self.get_logger().info('Received wait signals from drone0 and drone1. Sending drop signals.')
            signal_msg = Signal()
            signal_msg.data = True
            self.publisher_0.publish(signal_msg)
            self.wait_signal[0] = False
            self.wait_signal[1] = False
        elif self.wait_signal[2] and self.wait_signal[3]:
            self.get_logger().info('Received wait signals from drone2 and drone3. Sending drop signals.')
            signal_msg = Signal()
            signal_msg.data = True
            self.publisher_1.publish(signal_msg)
            self.wait_signal[2] = False
            self.wait_signal[3] = False


def main(args=None):

    rclpy.init(args=args)
    metiator = Metiator()

    try:
        rclpy.spin(metiator)
    finally:
        metiator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()