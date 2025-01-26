from .behavior import Behavior
from api import DroneApi
from common.logger import Logger
from common.ned_coordinate import NEDCoordinate
from agent.constants import DELTA_TIME, NAV_THRESHOLD
from typing import Optional

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
            return None # TODO 開始精準定位
        return None
