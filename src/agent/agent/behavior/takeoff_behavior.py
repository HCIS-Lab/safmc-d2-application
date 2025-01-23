from typing import Optional

from agent.constants import NAV_THRESH, TAKEOFF_HEIGHT
from api import DroneApi
from common.context import Context
from common.ned_coordinate import NEDCoordinate

from .behavior import Behavior


class TakeoffBehavior(Behavior):

    def on_enter(self, ctx: Context):
        drone_api: DroneApi = ctx.drone_api

        self.takeoff_position = drone_api.start_position - NEDCoordinate.down * TAKEOFF_HEIGHT
        ctx.log_info(f"Target takeoff position: {self.takeoff_position}")

    def execute(self, ctx: Context):
        drone_api: DroneApi = ctx.drone_api

        ctx.log_info(f"Current position: {drone_api.local_position}")

        vel = (self.takeoff_position - drone_api.local_position).normalized
        drone_api.add_velocity(vel)

    def get_next_state(self, ctx: Context) -> Optional[str]:
        drone_api: DroneApi = ctx.drone_api

        if NEDCoordinate.distance(drone_api.local_position, self.takeoff_position) <= NAV_THRESH:
            ctx.log_info("Takeoff altitude reached.")
            return "walk_to_hotspot" if drone_api.is_loaded else "walk_to_supply"

        return None
