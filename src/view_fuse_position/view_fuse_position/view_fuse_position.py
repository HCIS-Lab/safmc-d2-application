import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleOdometry

class OdometrySubscriber(Node):
    def __init__(self):
        super().__init__('odometry_subscriber')
        self.subscription = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.odometry_callback,
            10)
        self.subscription  # 避免被垃圾回收

    def odometry_callback(self, msg):
        position = msg.position  # [y, x, z] in meters
        position = [position[1], position[0], position[2]] # [y, x, z] to [x, y, z]
        self.get_logger().info(f"Position: {position}")

def main(args=None):
    rclpy.init(args=args)
    node = OdometrySubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
