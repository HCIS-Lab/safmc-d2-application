DELTA_TIME = 0.2
'''
Time step interval (seconds) for updates.
'''

TAKEOFF_HEIGHT = 1.5
'''
Altitude (meters) for takeoff.
'''

NAV_THRESHOLD = 0.2
'''
Minimum distance (meters) to target for navigation completion.
'''

LOAD_HEIGHT = 0.3
'''
Altitude (meters) for the drone to descend to in order to pick up the payload.
'''

import cv2

ARUCO_DICT = cv2.aruco.DICT_5X5_50

ARUCO_MARKER_SIZE = 0.1 # meter(s)

ARUCO_DIST_THRESHOLD = 0.1
'''
Distance threashold (meters) to the target Aruco Marker. Alignment is treated as complete if the distance is lower than this.
'''
