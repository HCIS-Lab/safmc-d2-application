from typing import Optional

from agent.constants import DELTA_TIME, NAV_THRESHOLD
from api import DroneApi, MediatorApi
from common.logger import Logger
from common.ned_coordinate import NEDCoordinate

from .behavior import Behavior


class WalkToSupplyBehavior(Behavior):

    def __init__(self, logger: Logger, drone_api: DroneApi, mediator_api: MediatorApi):
        super().__init__(logger)
        self.drone_api = drone_api
        self.mediator_api = mediator_api
        self.speed: float = 0.5

    def on_enter(self):

        # TODO supply zone 的兩端點位置如何決定?
        self.point_a: NEDCoordinate = NEDCoordinate(self.mediator_api.supply_zone[0])
        self.point_b: NEDCoordinate = NEDCoordinate(self.mediator_api.supply_zone[1])

        self.target_position: NEDCoordinate = self.point_a

    def execute(self):
        current_location = self.drone_api.local_position

        self.logger.info(f"target position: {self.target_position}")
        self.logger.info(f"current position: {current_location}")

        # 往 target_position 移動, 速度大小是 self.speed
        dist = NEDCoordinate.distance(current_location, self.target_position)
        vel = (self.target_position - current_location).normalized * min(self.speed, dist)
        self.drone_api.move_with_velocity(vel, DELTA_TIME)

        if NEDCoordinate.distance(self.drone_api.local_position, self.target_position) <= NAV_THRESHOLD:
            # 回頭 (A to B or B to A)
            self.logger.info(f"reached target position, changing direction")
            self.target_position = self.point_b if self.target_position == self.point_a else self.point_a

    def get_next_state(self) -> Optional[str]:
        if False: # TODO: 如果畫面中有出現 aruco marker
            return None # TODO 開始精準定位
        return None
