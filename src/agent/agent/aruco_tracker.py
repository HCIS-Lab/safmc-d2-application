import cv2
import cv2.aruco as aruco
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from sensor_msgs.msg import Image

from agent.constants import ARUCO_DICT, ARUCO_MARKER_SIZE
from agent_msgs.msg import ArucoInfo


class ArucoTracker(Node):
    def __init__(self):
        super().__init__('aruco_tracker')

        self.bridge = CvBridge()
        self.dictionary = aruco.getPredefinedDictionary(ARUCO_DICT)
        self.parameters = aruco.DetectorParameters()

        # 相機資訊 之後要校正
        self.camera_matrix = np.array([[1400, 0, 640], [0, 1400, 360], [0, 0, 1]])
        self.dist_coeffs = np.zeros((5, 1))

        # aruco實際大小
        self.aruco_marker_size = ARUCO_MARKER_SIZE

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # use calibration or not
        # ref: http://wiki.ros.org/image_proc?distro=noetic
        use_calibration = False 
        if use_calibration:
            self.subscription = self.create_subscription(
                Image,
                '/camera/image_rect',
                self.image_callback,
                qos_profile
            )
        else:
            self.subscription = self.create_subscription(
                Image,
                '/camera/image_raw',
                self.image_callback,
                qos_profile
            )
            
        self.publisher = self.create_publisher(ArucoInfo, '/aruco_info', 10)
        

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, self.dictionary, parameters=self.parameters)
                    
        aruco_msg = ArucoInfo()
        aruco_msg.id = -1
        aruco_msg.x = 0
        aruco_msg.y = 0

        if ids is not None:
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, self.aruco_marker_size, self.camera_matrix, self.dist_coeffs)
            
            for i, id in enumerate(ids):
                tvec = tvecs[i][0]  # position (x, y, z)
                rvec = rvecs[i][0]  # rotation
                
                aruco_msg = ArucoInfo()
                aruco_msg.id = id
                aruco_msg.position.x = tvec[0]
                aruco_msg.position.y = tvec[1]
                aruco_msg.position.z = tvec[2]
                self.publisher.publish(aruco_msg)
        else:
            print("No Aruco markers detected.") # TODO

def main(args=None):
    
    rclpy.init(args=args)
    aruco_tracker = ArucoTracker()

    try:
        rclpy.spin(aruco_tracker)
    finally:
        aruco_tracker.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
