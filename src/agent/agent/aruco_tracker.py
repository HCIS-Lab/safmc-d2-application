import cv2
import numpy as np
import yaml
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from common.qos import sensor_qos_profile, status_qos_profile
from agent.constants import ARUCO_DICT, ARUCO_DICT_FOR_HEADING
from safmc_msgs.msg import ArucoPose
import transforms3d


class ArucoTracker(Node):
    _cv_bridge = CvBridge()
    _detectors = {
        "pose": cv2.aruco.ArucoDetector(
            cv2.aruco.getPredefinedDictionary(ARUCO_DICT),
            cv2.aruco.DetectorParameters(),
        ),
        "heading": cv2.aruco.ArucoDetector(
            cv2.aruco.getPredefinedDictionary(ARUCO_DICT_FOR_HEADING),
            cv2.aruco.DetectorParameters(),
        ),
    }

    def __init__(self):
        super().__init__("aruco_tracker")

        # Parameters
        self.aruco_marker_size = self.get_parameter_or("aruco_marker_size", 0.05)
        camera_calibration_file_path = self.get_parameter_or(
            "camera_calibration_file_path",
            "/home/user/.ros/camera_info/imx708_wide__base_soc_i2c0mux_i2c_1_imx708_1a_320x240.yaml",
        )

        self._marker_points = self._get_aruco_marker_points(self.aruco_marker_size)
        self._marker_points_for_heading = self._get_aruco_marker_points(0.075)

        self._camera_matrix, self._dist_coeffs = self._load_camera_intrinsic_params(
            camera_calibration_file_path
        )
        self.get_logger().info(f"Loaded Camera Matrix: {self._camera_matrix}")
        self.get_logger().info(f"Loaded Distortion Coeffs: {self._dist_coeffs}")

        # ROS Interfaces
        self.create_subscription(
            Image, "camera/image_raw", self._image_callback, sensor_qos_profile
        )
        self._publishers = {
            "pose": self.create_publisher(ArucoPose, "aruco_pose", status_qos_profile),
            "heading": self.create_publisher(
                ArucoPose, "aruco_pose_for_heading", status_qos_profile
            ),
        }

    def _get_aruco_marker_points(self, size: float) -> np.ndarray:
        half_size = size / 2
        return np.array(
            [
                [-half_size, half_size, 0],
                [half_size, half_size, 0],
                [half_size, -half_size, 0],
                [-half_size, -half_size, 0],
            ],
            dtype=np.float32,
        )

    def _load_camera_intrinsic_params(self, file_path: str):
        with open(file_path, "r") as file:
            camera_data = yaml.safe_load(file)
        return (
            np.array(camera_data["camera_matrix"]["data"]).reshape(
                (
                    camera_data["camera_matrix"]["rows"],
                    camera_data["camera_matrix"]["cols"],
                )
            ),
            np.array(camera_data["distortion_coefficients"]["data"]).reshape(
                (
                    camera_data["distortion_coefficients"]["rows"],
                    camera_data["distortion_coefficients"]["cols"],
                )
            ),
        )

    def _image_callback(self, msg):
        frame = self._cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        for key, detector in self._detectors.items():
            corners, ids, _ = detector.detectMarkers(gray)
            if ids is not None:
                for i in range(len(ids)):
                    self._process_marker(ids[i], corners[i][0], key)

    def _process_marker(self, marker_id, corners, marker_type):
        success, rvec, tvec = cv2.solvePnP(
            (
                self._marker_points
                if marker_type == "pose"
                else self._marker_points_for_heading
            ),
            corners,
            self._camera_matrix,
            self._dist_coeffs,
        )

        if success:
            position = tvec.flatten()
            rotation_matrix, _ = cv2.Rodrigues(rvec)
            quaternion = transforms3d.quaternions.mat2quat(rotation_matrix)

            pose_msg = ArucoPose()
            pose_msg.aruco_marker_id = int(marker_id)
            pose_msg.position.x, pose_msg.position.y, pose_msg.position.z = position
            (
                pose_msg.orientation.x,
                pose_msg.orientation.y,
                pose_msg.orientation.z,
                pose_msg.orientation.w,
            ) = quaternion

            self._publishers[marker_type].publish(pose_msg)
            self.get_logger().debug(
                f"Detected ID {marker_id} ({marker_type}) Position: {position}"
            )
        else:
            self.get_logger().warn(
                f"solvePnP() failed for marker {marker_id} ({marker_type})"
            )


def main(args=None):
    rclpy.init(args=args)
    node = ArucoTracker()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
