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
        current_location = drone_api.local_position

        ctx.log_info(f"target position: {self.target_position}")
        ctx.log_info(f"current position: {current_location}")

        dist = NEDCoordinate.distance(current_location, self.target_position)
        vel = (self.target_position - current_location).normalized * min(self.speed, dist)
        drone_api.move_with_velocity(vel, DELTA_TIME)

        if NEDCoordinate.distance(drone_api.local_position, self.target_position) <= NAV_THRESHOLD:
            # 回頭 (A to B or B to A)
            ctx.log_info(f"Reached target {self.target_position}. Changing direction.")
            self.target_position = self.point_b if self.target_position == self.point_a else self.point_a

    def get_next_state(self, context: Context) -> Optional[str]:
        drone_api: DroneApi = context.drone_api

        if False:
            return "load"
        return None
