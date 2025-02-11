import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
import cv2
import numpy as np
from cv_bridge import CvBridge

from agent_msgs.msg import ArucoInfo

class ArucoTracker(Node):
    def __init__(self):
        super().__init__('aruco_tracker')
        
        # use calibration or not
        use_calibration = False 
        if use_calibration:
            self.subscription = self.create_subscription(
                Image,
                '/camera/image_rect',
                self.image_callback,
                10)
        else:
            self.subscription = self.create_subscription(
                Image,
                '/camera/image_raw',
                self.image_callback,
                10)
            
        self.publisher = self.create_publisher(ArucoInfo, '/cmd_vel', 10)
        self.bridge = CvBridge()
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50)
        self.parameters = cv2.aruco.DetectorParameters()

        # 相機資訊 之後要校正
        self.camera_matrix = np.array([[1400, 0, 640], [0, 1400, 360], [0, 0, 1]])
        self.dist_coeffs = np.zeros((5, 1))

        # aruco實際大小
        self.aruco_marker_size = 0.1

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.dictionary, parameters=self.parameters)

        aruco_msg = ArucoInfo()
        aruco_msg.id = -1
        aruco_msg.x = 0
        aruco_msg.y = 0

        if ids is not None:
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.aruco_marker_size, self.camera_matrix, self.dist_coeffs)
            target_x, target_y = tvecs[0][0][0], tvecs[0][0][1]
            
            aruco_msg.id = ids[0][0]
            aruco_msg.x = -target_x  
            aruco_msg.y = -target_y
            
            self.publisher.publish(aruco_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
