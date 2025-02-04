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
        self.speed: float = 0.2 # 調降速度讓機器不要跑過頭
        self.tolerance: float = 0.05  # 距離誤差


    def on_enter(self):
        # TODO 精準定位要用相對位置還是絕對位置？
        self.target_position: NEDCoordinate = NEDCoordinate()

    def execute(self):
        current_location = self.drone_api.local_position

        self.logger.info(f"target position: {self.target_position}")
        self.logger.info(f"current position: {current_location}")

        # TODO: 依照 Aruco Marker 精準定位


    def get_next_state(self) -> Optional[str]:
        if False: # TODO: 如果定位已經完成了
            return "wait" # 等待
        return None
