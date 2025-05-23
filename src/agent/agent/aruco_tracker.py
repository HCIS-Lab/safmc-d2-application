import cv2
import numpy as np
import yaml
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from common.qos import sensor_qos_profile, status_qos_profile
from agent.constants import ARUCO_DICT, ARUCO_MARKER_SIZE
from safmc_msgs.msg import ArucoPose
import transforms3d


class ArucoTracker(Node):
    # use calibration or not (image_rect)
    # ref: http://wiki.ros.org/image_proc?distro=noetic
    _use_calibration = False

    _cv_bridge = CvBridge()
    _aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    _detector_parameters = cv2.aruco.DetectorParameters()
    _detector = cv2.aruco.ArucoDetector(_aruco_dict, _detector_parameters)
    _image_encoding = "bgr8"
    _camera_yaml_file = "/workspace/safmc-d2-bridge/camera/imx708_wide__base_soc_i2c0mux_i2c_1_imx708_1a_320x240.yaml"  # TODO[lnfu] better location?

    _marker_points = np.array(
        [
            [-ARUCO_MARKER_SIZE / 2, ARUCO_MARKER_SIZE / 2, 0],  # 左上角
            [ARUCO_MARKER_SIZE / 2, ARUCO_MARKER_SIZE / 2, 0],  # 右上角
            [ARUCO_MARKER_SIZE / 2, -ARUCO_MARKER_SIZE / 2, 0],  # 右下角
            [-ARUCO_MARKER_SIZE / 2, -ARUCO_MARKER_SIZE / 2, 0],  # 左下角
        ],
        dtype=np.float32,
    )

    def __init__(self):
        super().__init__("aruco_tracker")

        # 相機資訊 之後要校正
        self._camera_matrix, self._dist_coeffs = self._load_camera_intrinsic_params(
            self._camera_yaml_file
        )

        self.get_logger().info(f"camera matrix: {self._camera_matrix}")
        self.get_logger().info(f"dist coeffs: {self._dist_coeffs}")

        if self._use_calibration:
            self.create_subscription(
                Image,
                "camera/image_rect",  # TODO[lnfu] topic name
                self._image_callback,
                sensor_qos_profile,
            )
        else:
            self.create_subscription(
                Image, "camera/image_raw", self._image_callback, sensor_qos_profile
            )

        self._image_aruco_pub = self.create_publisher(
            Image, f"camera/image_aruco", sensor_qos_profile
        )

        self._aruco_pose_pub = self.create_publisher(
            ArucoPose, "aruco_pose", status_qos_profile
        )

    def _load_camera_intrinsic_params(self, yaml_file):
        with open(yaml_file, "r") as file:
            camera_data = yaml.safe_load(file)

        camera_matrix = np.array(camera_data["camera_matrix"]["data"]).reshape(
            (camera_data["camera_matrix"]["rows"], camera_data["camera_matrix"]["cols"])
        )

        dist_coeffs = np.array(camera_data["distortion_coefficients"]["data"]).reshape(
            (
                camera_data["distortion_coefficients"]["rows"],
                camera_data["distortion_coefficients"]["cols"],
            )
        )
        return camera_matrix, dist_coeffs

    def _image_callback(self, msg):
        self.get_logger().info("receive image")

        frame = self._cv_bridge.imgmsg_to_cv2(
            msg, desired_encoding=self._image_encoding
        )
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self._detector.detectMarkers(gray)

        if ids is not None:
            for i in range(len(ids)):
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)

                # 使用 `solvePnP()` 計算 3D 姿態
                success, rvec, tvec = cv2.solvePnP(
                    self._marker_points,
                    corners[i][0],
                    self._camera_matrix,
                    self._dist_coeffs,
                )

                if success:
                    # 繪製 3D 坐標軸
                    cv2.drawFrameAxes(
                        frame,
                        self._camera_matrix,
                        self._dist_coeffs,
                        rvec,
                        tvec,
                        0.03,
                        1,
                    )

                    # 顯示標記 ID 和座標
                    x, y, z = tvec.flatten()
                    cv2.putText(
                        img=frame,
                        text=f"ID: {ids[i][0]} Pos: ({x:.2f}, {y:.2f}, {z:.2f})",
                        org=(int(corners[i][0][0][0]), int(corners[i][0][0][1])),
                        fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                        fontScale=1.0,
                        color=(0, 255, 0),
                        thickness=1,
                    )

                    msg = self._cv_bridge.cv2_to_imgmsg(
                        frame, encoding=self._image_encoding
                    )

                    rotation_matrix, _ = cv2.Rodrigues(rvec)

                    quaternion = transforms3d.quaternions.mat2quat(rotation_matrix)

                    aruco_pose_msg = ArucoPose()
                    aruco_pose_msg.aruco_marker_id = int(ids[i])
                    aruco_pose_msg.position.x = x
                    aruco_pose_msg.position.y = y
                    aruco_pose_msg.position.z = z
                    aruco_pose_msg.orientation.x = quaternion[0]
                    aruco_pose_msg.orientation.y = quaternion[1]
                    aruco_pose_msg.orientation.z = quaternion[2]
                    aruco_pose_msg.orientation.w = quaternion[3]
                    self._aruco_pose_pub.publish(aruco_pose_msg)
                    self.get_logger().info(  # TODO[lnfu] debug
                        f"position: x={x:.3f}, y={y:.3f}, z={z:.3f}",
                    )
                    self.get_logger().info(  # TODO[lnfu] debug
                        f"orientation: x={quaternion[0]:.3f}, y={quaternion[1]:.3f}, z={quaternion[2]:.3f}, w={quaternion[3]:.3f}",
                    )
                else:
                    self.get_logger().info("solvePnP() error")
        else:
            self.get_logger().info("aruco marker not found")

        self._image_aruco_pub.publish(msg)


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
