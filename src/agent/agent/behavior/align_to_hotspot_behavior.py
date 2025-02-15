
from typing import Optional

from agent.constants import ARUCO_DIST_THRESHOLD
from api import ArucoApi, DroneApi
from common.logger import Logger

from .behavior import Behavior


class AlignToHotspotBehavior(Behavior):  # 精準定位

    def __init__(self, logger: Logger, drone_api: DroneApi, aruco_api: ArucoApi):
        super().__init__(logger)

        self.drone_api = drone_api
        self.aruco_api = aruco_api
        self.speed: float = 0.3  # 機器最大速度

    def execute(self):
        # 目前想法是依照 Aruco node 的資訊做 move with velocity
        # Aruco node 的回傳是無人機要移動到 Aruco marker 的距離

        vel = self.aruco_api.marker_position
        vel.z = self.drone_api.home_position - self.drone_api.local_position

        if vel.magnitude > self.speed:
            vel = self.speed * vel.normalized

        self.drone_api.move_with_velocity(vel)

    def get_next_state(self) -> Optional[str]:
        pos = self.aruco_api.marker_position
        pos.z = 0

        if pos.magnitude <= ARUCO_DIST_THRESHOLD :
            return "wait"  # 等待
        if self.aruco_api.idle_time.nanoseconds > 3e9 : # 3sec
            return "walk_to_hotspot"                    # 偵測不到目標 marker，退回去重走

        return None
