from typing import Optional

from agent.constants import DELTA_TIME, NAV_THRESHOLD
from api import DroneApi
from common.context import Context
from common.ned_coordinate import NEDCoordinate

from .behavior import Behavior


class WalkToSupplyBehavior(Behavior):

    def __init__(self):
        self.speed: float = 0.5

    def on_enter(self, ctx: Context):
        drone_api: DroneApi = ctx.drone_api

        self.point_a: NEDCoordinate = NEDCoordinate(1, 1, drone_api.local_position.z)
        self.point_b: NEDCoordinate = NEDCoordinate(1, 7, drone_api.local_position.z)

        self.target_position: NEDCoordinate = self.point_a

    def execute(self, ctx: Context):
        drone_api: DroneApi = ctx.drone_api

        ctx.log_info(f"target position: {self.target_position}")
        ctx.log_info(f"current position: {drone_api.local_position}")
        # drone_api.move(self.target_position)
        
        diff = self.target_position - drone_api.local_position
        direction = diff.normalized
        if diff.magnitude < self.speed:
            drone_api.add_velocity2(diff, DELTA_TIME)
        else:
            drone_api.add_velocity2(direction * self.speed, DELTA_TIME)

        if NEDCoordinate.distance(drone_api.local_position, self.target_position) <= NAV_THRESHOLD:
            # 回頭 (A to B or B to A)
            ctx.log_info(f"Reached target {self.target_position}. Changing direction.")
            self.target_position = self.point_b if self.target_position == self.point_a else self.point_a

    def get_next_state(self, context: Context) -> Optional[str]:
        drone_api: DroneApi = context.drone_api

        if False:
            return "load"
        return None
