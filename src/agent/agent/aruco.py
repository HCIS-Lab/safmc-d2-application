import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import cv2
import numpy as np
from cv_bridge import CvBridge

class ArucoTracker(Node):
    def __init__(self):
        super().__init__('aruco_tracker')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.bridge = CvBridge()
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50)
        self.parameters = cv2.aruco.DetectorParameters()

        # 相機資訊 之後要校正
        self.camera_matrix = np.array([[1400, 0, 640], [0, 1400, 360], [0, 0, 1]])
        self.dist_coeffs = np.zeros((5, 1))

        # aruco實際大小
        self.aruco_marker_size = 0.1

        # 距離誤差閾值
        self.target_tolerance = 0.1 

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.dictionary, parameters=self.parameters)

        if ids is not None:
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.aruco_marker_size, self.camera_matrix, self.dist_coeffs)
            target_x, target_y = tvecs[0][0][0], tvecs[0][0][1]
            twist_msg = Twist()
            
            # 閾值內移動
            if abs(target_x) > self.target_tolerance:
                twist_msg.linear.x = -target_x  
            if abs(target_y) > self.target_tolerance:
                twist_msg.linear.y = -target_y
            
            self.publisher.publish(twist_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
