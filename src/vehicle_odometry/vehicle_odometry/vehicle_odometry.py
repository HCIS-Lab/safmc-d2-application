# This ROS node should be run on mediator

import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleOdometry
from uwb_msgs.msg import TagPosition
import time

class VehicleVisualOdometry(Node):
    def __init__(self):
        super().__init__('vehicle_visual_odometry')

        # Subscriptions
        self.vehicle_global_position_sub = self.create_subscription(
            TagPosition,
            "/tag_position",
            self.__set_vehicle_odometry,
            10
        )
        # Publisher
        self.publisher_ = [ 
            self.create_publisher(VehicleOdometry, f'/px4_1/fmu/in/vehicle_visual_odometry', 10),
            self.create_publisher(VehicleOdometry, f'/px4_2/fmu/in/vehicle_visual_odometry', 10),
            self.create_publisher(VehicleOdometry, f'/px4_3/fmu/in/vehicle_visual_odometry', 10),
            self.create_publisher(VehicleOdometry, f'/px4_4/fmu/in/vehicle_visual_odometry', 10)
        ]
        self.timestamp = int(time.time() * 1e6)

    def __set_vehicle_odometry(self, msg: TagPosition):
        # msg.eui is a string from '01:01' to '04:04'
        self.publish_odometry([msg.x, msg.y, msg.z], int(msg.eui[-1]))

    def publish_odometry(self, global_position, publisher_num):
        msg = VehicleOdometry()
        msg.timestamp = self.timestamp
        msg.timestamp_sample = self.timestamp
        msg.position = global_position
        # 發布消息
        self.publisher_[publisher_num - 1].publish(msg)
        self.get_logger().info(f'px4_{publisher_num} Published visual odometry: {msg.position}')
        self.timestamp = int(time.time() * 1e6)


def main(args=None):
    rclpy.init(args=args)
    node = VehicleVisualOdometry()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
