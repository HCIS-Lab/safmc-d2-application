import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleVisualOdometry
import time

class VisualOdometryPublisher(Node):
    def __init__(self):
        super().__init__('visual_odometry_publisher')
        self.publisher_ = self.create_publisher(VehicleVisualOdometry, '/fmu/in/vehicle_visual_odometry', 10)
        self.timer = self.create_timer(0.1, self.publish_odometry)  # 10 Hz
        self.timestamp = int(time.time() * 1e6)

    def publish_odometry(self):
        msg = VehicleVisualOdometry()
        msg.timestamp = self.timestamp
        msg.timestamp_sample = self.timestamp
        msg.pose_frame = 0
        msg.position.x = 1.0
        msg.position.y = 2.0
        msg.position.z = -3.0
        msg.q.x = 0.0
        msg.q.y = 0.0
        msg.q.z = 0.0
        msg.q.w = 1.0
        msg.velocity.x = 0.0
        msg.velocity.y = 0.0
        msg.velocity.z = 0.0
        msg.angular_velocity.x = 0.0
        msg.angular_velocity.y = 0.0
        msg.angular_velocity.z = 0.0
        msg.position_variance.x = 0.01
        msg.position_variance.y = 0.01
        msg.position_variance.z = 0.01
        msg.velocity_variance.x = 0.01
        msg.velocity_variance.y = 0.01
        msg.velocity_variance.z = 0.01
        msg.reset_counter = 0
        msg.quality = 255
        self.publisher_.publish(msg)
        self.timestamp += 100000  # 10 Hz 更新

def main(args=None):
    rclpy.init(args=args)
    node = VisualOdometryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
