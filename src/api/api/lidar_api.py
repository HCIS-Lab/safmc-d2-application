import math

from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan

from .api import Api


class LidarApi(Api):
    def __init__(self, node: Node):
        self.__node = node
        self.__lidar_ranges = []
        self.__angle_min = 0.0
        self.__angle_increment = 0.0
        self.__range_min = 0.0
        self.__range_max = 0.0

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.lidar_sub = node.create_subscription(
            LaserScan,
            f"scan",
            self.__set_lidar_status,
            qos_profile
        )

    def __set_lidar_status(self, msg: LaserScan):
        self.__lidar_ranges = msg.ranges
        self.__angle_min = msg.angle_min
        self.__angle_increment = msg.angle_increment
        self.__range_min = msg.range_min
        self.__range_max = msg.range_max

    def get_obstacle_points_2d(self, max_distance: float = 3.0):
        """
        將 LiDAR 分佈轉成 2D 坐標點 (以機體/雷射座標為原點).
        只取距離小於 max_distance 的數據, 以過濾過遠或雜訊.
        回傳 [(x1, y1), (x2, y2), ...]
        右邊方向是-y
        x方向正確（往前）
        """
        obstacle_points = []
        if not self.__lidar_ranges:
            return obstacle_points

        angle = self.__angle_min
        for dist in self.__lidar_ranges:
            if self.__range_min < dist < max_distance:
                x = dist * math.cos(angle)
                y = dist * math.sin(angle)
                obstacle_points.append((x, y))
            angle += self.__angle_increment

        return obstacle_points
