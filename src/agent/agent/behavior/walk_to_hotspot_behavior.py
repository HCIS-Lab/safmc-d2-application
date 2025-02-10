from typing import Optional

from agent.constants import DELTA_TIME, NAV_THRESHOLD
from api import DroneApi
from common.logger import Logger
from common.ned_coordinate import NEDCoordinate

from .behavior import Behavior


class WalkToHotspotBehavior(Behavior):

    def __init__(self, logger: Logger, drone_api: DroneApi):
        super().__init__(logger)
        self.drone_api = drone_api
        self.speed: float = 0.5


    def on_enter(self):
        # TODO hotspot 位置如何決定? 應該是要 mediator 告訴他?
        self.target_position: NEDCoordinate = NEDCoordinate(5.65797, -6.2135, self.drone_api.local_position.z)


    def execute(self):
        current_location = self.drone_api.local_position

        self.logger.info(f"target position: {self.target_position}")
        self.logger.info(f"current position: {current_location}")

        # 往 target_position 移動, 速度大小是 self.speed
        dist = NEDCoordinate.distance(current_location, self.target_position)
        vel = (self.target_position - current_location).normalized * min(self.speed, dist)
        self.drone_api.move_with_velocity(vel, DELTA_TIME)


    def get_next_state(self) -> Optional[str]:
        if False: # TODO: 如果畫面中有出現 aruco marker
            return "align_to_hotspot" # TODO 開始精準定位
        return None
    

class AlignToHotspotBehavior(Behavior): # 精準定位

    def __init__(self, logger: Logger, drone_api: DroneApi): # TODO: 需要 aruco 的 node
        super().__init__(logger)
        self.drone_api = drone_api
        self.speed: float = 0.2 # 機器速度倍率
        self.dist_threshold = 0.1 # 距離誤差閾值


    def on_enter(self):
        pass

    def execute(self):
        # TODO: 依照 Aruco Marker 精準定位
        # 目前想法是依照 Aruco node 的資訊做 move with velocity
        # Aruco node 的回傳是無人機要移動到 Aruco marker 的距離
        pass 


    def get_next_state(self) -> Optional[str]:
        if all ([dist <= self.dist_threshold for dist in self.drone_api.get_aruco_point_sub()]) : # 如果定位已經完成了，aruco node 傳回的各方向距離都會低於閾值
            return "wait" # 等待
        return None
