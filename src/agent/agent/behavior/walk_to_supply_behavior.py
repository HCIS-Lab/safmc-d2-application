from typing import Optional

from agent.constants import DELTA_TIME, NAV_THRESHOLD
from api import DroneApi
from common.logger import Logger
from common.ned_coordinate import NEDCoordinate

from .behavior import Behavior


class WalkToSupplyBehavior(Behavior):

    def __init__(self, logger: Logger, drone_api: DroneApi):
        super().__init__(logger)
        self.drone_api = drone_api
        self.speed: float = 0.5

    def on_enter(self):

        # TODO supply zone 的兩端點位置如何決定?
        self.point_a: NEDCoordinate = NEDCoordinate(1, 1, self.drone_api.local_position.z)
        self.point_b: NEDCoordinate = NEDCoordinate(1, 7, self.drone_api.local_position.z)

        self.target_position: NEDCoordinate = self.point_a

    def execute(self):
        current_location = self.drone_api.local_position

        self.logger.info(f"target position: {self.target_position}")
        self.logger.info(f"current position: {current_location}")

        # 往 target_position 移動, 速度大小是 self.speed
        dist = NEDCoordinate.distance(current_location, self.target_position)
        vel = (self.target_position - current_location).normalized * min(self.speed, dist)
        self.drone_api.move_with_velocity(vel)

        if NEDCoordinate.distance(self.drone_api.local_position, self.target_position) <= NAV_THRESHOLD:
            # 回頭 (A to B or B to A)
            self.logger.info(f"reached target position, changing direction")
            self.target_position = self.point_b if self.target_position == self.point_a else self.point_a

    def get_next_state(self) -> Optional[str]:
        if self.drone_api.get_aruco_info_sub()[0] >= 0: # 如果畫面中有出現 aruco marker
            return "align_to_supply" # 開始精準定位
        return None
    

class AlignToSupplyBehavior(Behavior): # 精準定位

    def __init__(self, logger: Logger, drone_api: DroneApi): # TODO: 需要 aruco 的 node
        super().__init__(logger)
        self.drone_api = drone_api
        self.speed: float = 0.75 # 機器速度倍率
        self.dist_threshold = 0.1 # 距離誤差閾值


    def on_enter(self):
        pass

    def execute(self):
        # 目前想法是依照 Aruco node 的資訊做 move with velocity
        # Aruco node 的回傳是無人機要移動到 Aruco marker 的距離
        vel_tuple = [(self.vel_factor * coord) if coord >= self.dist_threshold else 0 for coord in self.drone_api.get_aruco_point_sub()]
        vel = NEDCoordinate(vel_tuple[0], vel_tuple[1], vel_tuple[2])
        self.drone_api.move_with_velocity(vel)


    def get_next_state(self) -> Optional[str]:
        if all ([dist <= self.dist_threshold for dist in self.drone_api.get_aruco_point_sub()]) : # 如果定位已經完成了，aruco node 傳回的各方向距離都會低於閾值
            return "load" # 拿東西
        return None

