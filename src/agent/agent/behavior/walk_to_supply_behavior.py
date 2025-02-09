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
        if False: # TODO: 如果畫面中有出現 aruco marker
            return None # TODO 開始精準定位
        return None
