import cv2
import cv2.aruco as aruco
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Vector3
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from sensor_msgs.msg import Image

from agent.constants import ARUCO_DICT, ARUCO_MARKER_SIZE
from agent_msgs.msg import ArucoInfo
from common.parameters import get_parameter


class ArucoTracker(Node):
    def __init__(self):
        super().__init__("aruco_tracker")

        self.__drone_id = get_parameter(self, "drone_id", 1)

        self.bridge = CvBridge()
        self.dictionary = aruco.getPredefinedDictionary(ARUCO_DICT)
        self.parameters = aruco.DetectorParameters_create()

        # 相機資訊 之後要校正
        self.camera_matrix = np.array([[1400, 0, 640], [0, 1400, 360], [0, 0, 1]])
        self.dist_coeffs = np.zeros((5, 1))

        # aruco實際大小
        self.aruco_marker_size = ARUCO_MARKER_SIZE

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        image_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # use calibration or not
        # ref: http://wiki.ros.org/image_proc?distro=noetic
        use_calibration = True
        if use_calibration:
            self.subscription = self.create_subscription(
                Image,
                "camera/image_rect",  # TODO topic name
                self.image_callback,
                qos_profile,
            )

            self.detected_image_pub = self.create_publisher(
                Image, f"detected/aruco", image_qos
            )

        else:
            self.subscription = self.create_subscription(
                Image,
                f"/world/safmc_d2/model/x500_safmc_d2_{self.__drone_id}/link/pi3_cam_link/sensor/pi3_cam_sensor/image",
                self.image_callback,
                image_qos,
            )
        self.publisher = self.create_publisher(ArucoInfo, "aruco_info", qos_profile)

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(
            gray, self.dictionary, parameters=self.parameters
        )

        aruco_msg = ArucoInfo()
        aruco_msg.id = -1
        aruco_msg.position = Vector3()

        if ids is not None:
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                corners, self.aruco_marker_size, self.camera_matrix, self.dist_coeffs
            )

            for rvec, tvec, marker_corner, id in zip(
                rvecs, tvecs, list(corners)[0], ids
            ):
                # tvec = tvecs[i][0]  # position (x, y, z)
                # rvec = rvecs[i][0]  # rotation
                print(frame.shape)

                marked_frame = frame
                cv2.line(
                    marked_frame,
                    (int(marker_corner[0][0]), int(marker_corner[0][1])),
                    (int(marker_corner[1][0]), int(marker_corner[1][1])),
                    (0, 255, 0),
                    6,
                )
                cv2.line(
                    marked_frame,
                    (int(marker_corner[1][0]), int(marker_corner[1][1])),
                    (int(marker_corner[2][0]), int(marker_corner[2][1])),
                    (0, 255, 0),
                    6,
                )
                cv2.line(
                    marked_frame,
                    (int(marker_corner[2][0]), int(marker_corner[2][1])),
                    (int(marker_corner[3][0]), int(marker_corner[3][1])),
                    (0, 255, 0),
                    6,
                )
                cv2.line(
                    marked_frame,
                    (int(marker_corner[3][0]), int(marker_corner[3][1])),
                    (int(marker_corner[0][0]), int(marker_corner[0][1])),
                    (0, 255, 0),
                    6,
                )

                cv2.line(
                    marked_frame,
                    (
                        int((marker_corner[0][0] + marker_corner[2][0]) / 2),
                        int((marker_corner[0][1] + marker_corner[2][1]) / 2),
                    ),
                    (int(frame.shape[1] / 2), int(frame.shape[0] / 2)),
                    (0, 255, 0),
                    6,
                )

                cv2.putText(
                    marked_frame,
                    f"x:{tvec[0][1]:.3f}, y:{tvec[0][0]:.3f}",
                    (int(marker_corner[3][0]), int(marker_corner[3][1]) - 10),
                    fontScale=1.5,
                    thickness=3,
                    fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                    color=(0, 255, 0),
                    lineType=cv2.LINE_AA,
                )

                image_msg = self.bridge.cv2_to_imgmsg(marked_frame)
                self.detected_image_pub.publish(image_msg)

                aruco_msg = ArucoInfo()
                aruco_msg.id = int(id[0])
                aruco_msg.position.x = tvec[0][0]
                aruco_msg.position.y = tvec[0][1]
                aruco_msg.position.z = tvec[0][2]
                self.publisher.publish(aruco_msg)
        else:
            print("No Aruco markers detected.")  # TODO


def main(args=None):

    rclpy.init(args=args)
    aruco_tracker = ArucoTracker()

    try:
        rclpy.spin(aruco_tracker)
    finally:
        aruco_tracker.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
