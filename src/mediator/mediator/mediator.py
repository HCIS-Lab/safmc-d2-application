import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, Int8


class Metiator(Node):
    def __init__(self):
        super().__init__('metiator')
        self.wait_signal = [False, False, False, False]

        self.subscriber_0 = self.create_subscription(
            Int8, '/pair0/wait', self.signal_callback, 10)
        self.subscriber_1 = self.create_subscription(
            Int8, '/pair1/wait', self.signal_callback, 10)
        self.publisher_0 = self.create_publisher(Empty, '/pair0/signal', 10)
        self.publisher_1 = self.create_publisher(Empty, '/pair1/signal', 10)

    def signal_callback(self, msg):
        if msg.data >= 0 and msg.data <= 3:
            self.wait_signal[msg.data] = True
            self.get_logger().info(f'Received signal from drone{msg.data}')
            self.check_wait_publish_drop()

    def check_wait_publish_drop(self):
        if self.wait_signal[0] and self.wait_signal[1]:
            self.get_logger().info(
                'Received wait signals from drone0 and drone1. Sending drop signals.')
            signal_msg = Empty()
            self.publisher_0.publish(signal_msg)
            self.wait_signal[0] = False
            self.wait_signal[1] = False
        elif self.wait_signal[2] and self.wait_signal[3]:
            self.get_logger().info(
                'Received wait signals from drone2 and drone3. Sending drop signals.')
            signal_msg = Empty()
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
