import cv2

DELTA_TIME = 0.1
"""
Time step interval (seconds) for updates.
"""

TAKEOFF_HEIGHT = 0.9
"""
Altitude (meters) for takeoff.
"""

NAV_THRESHOLD = 0.1
"""
Maximum distance (meters) to target for navigation completion.
"""

HEIGHT_THRESHOLD = 0.05
"""
Maximum height difference (meters) to target for navigation completion.
"""

LOAD_HEIGHT = 0.15
"""
Altitude (meters) for the drone to descend to in order to pick up the payload.
"""


ARUCO_DICT = cv2.aruco.DICT_5X5_50

ARUCO_MARKER_SIZE = 0.05  # meter(s)

ARUCO_DIST_THRESHOLD = 0.015
"""
Distance threashold (meters) to the target Aruco Marker. Alignment is treated as complete if the distance is lower than this.
"""

CAMERA_CALIBRATION_FILE = "/workspace/safmc-d2-bridge/camera/imx708_wide__base_soc_i2c0mux_i2c_1_imx708_1a_800x600.yaml"
