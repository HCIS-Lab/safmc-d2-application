import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleOdometry
from uwb_msgs.msg import TagPosition
import time
from common.parameters import get_parameter

class VisualOdometryPublisher(Node):
    def __init__(self):
        super().__init__('vehicle_odometry_publisher')
        self.eui = get_parameter(self, 'eui', '01:01')  # eui is a string from '01:01' to '04:04'

        # Subscriptions
        self.vehicle_local_position_sub = self.create_subscription(
            TagPosition,
            "/tag_position",
            self.__set_vehicle_odometry,
            10
        )
        # Publisher
        self.publisher_ = self.create_publisher(VehicleOdometry, '/fmu/in/vehicle_vehicle_odometry', 10)
        self.timestamp = int(time.time() * 1e6)

    def __set_vehicle_odometry(self, msg: TagPosition):
        if msg.eui == self.eui:
            self.publish_odometry([msg.x, msg.y, msg.z])

    def publish_odometry(self, global_position):
        msg = VehicleOdometry()
        msg.timestamp = self.timestamp
        msg.timestamp_sample = self.timestamp
        msg.position = global_position
        # 發布消息
        self.publisher_.publish(msg)
        # self.get_logger().info(f'Published visual odometry: {msg.timestamp}, {msg.position}')
        self.timestamp = int(time.time() * 1e6)


def main(args=None):
    rclpy.init(args=args)
    node = VisualOdometryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
