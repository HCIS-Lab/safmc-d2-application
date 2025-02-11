from typing import Optional

from api import ArucoApi, DroneApi
from common.logger import Logger

from .behavior import Behavior


class AlignToSupplyBehavior(Behavior):  # 精準定位

    def __init__(self, logger: Logger, drone_api: DroneApi, aruco_api: ArucoApi):
        super().__init__(logger)

        self.drone_api = drone_api
        self.aruco_api = aruco_api
        self.speed: float = 0.75  # 機器速度倍率
        self.dist_threshold = 0.1  # 距離誤差閾值

        self.drone_api = drone_api
        self.aruco_api = aruco_api
        self.speed: float = 0.75  # 機器速度倍率
        self.dist_threshold = 0.1  # 距離誤差閾值, TODO 也許放到 constants.py?
        # self.outofframe_streak = 0  # 例外狀況：可能要退回之前的 state

    def execute(self):
        # 目前想法是依照 Aruco node 的資訊做 move with velocity
        # Aruco node 的回傳是無人機要移動到 Aruco marker 的距離

        vel = self.aruco_api.marker_position
        vel.z = 0

        if vel.magnitude > self.speed:
            vel = self.speed * vel.normalized

        self.drone_api.move_with_velocity(vel)

    def get_next_state(self) -> Optional[str]:
        pos = self.aruco_api.marker_position
        pos.z = 0

        if pos.magnitude <= self.dist_threshold:
            return "load"  # 等待
        return None

        # TODO
        # if aruco_info[0] < 0:  # 偵測不到 aruco marker
        #     self.outofframe_streak += 1
        #     if self.outofframe_streak > 60:  # 應該就是被神秘力量拉走了
        #         return "walk_to_hotspot"  # 回去重走
        # else:
        #     self.outofframe_streak = 0
