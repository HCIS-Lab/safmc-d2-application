from typing import Optional

from agent.constants import NAV_THRESHOLD, TAKEOFF_HEIGHT
from api import DroneApi, MagnetApi
from common.context import Context
from common.ned_coordinate import NEDCoordinate

from .behavior import Behavior


class TakeoffBehavior(Behavior):

    def __init__(self):
        self.velocity: float = 0.5

    def on_enter(self, ctx: Context):
        drone_api: DroneApi = ctx.drone_api

        self.target_position = drone_api.local_position - NEDCoordinate.down * TAKEOFF_HEIGHT

    def execute(self, ctx: Context):
        drone_api: DroneApi = ctx.drone_api

        ctx.log_info(f"target position: {self.target_position}")
        ctx.log_info(f"current position: {drone_api.local_position}")
        drone_api.move(self.target_position)

    def get_next_state(self, ctx: Context) -> Optional[str]:
        drone_api: DroneApi = ctx.drone_api
        magnet_api: MagnetApi = ctx.magnet_api
    
        if self.__has_reached_final_position(ctx):
            ctx.log_info("Takeoff altitude reached.")
            return "walk_to_hotspot" if magnet_api.is_loaded else "walk_to_supply"
        return None

    def __has_reached_final_position(self, ctx: Context) -> bool:
        drone_api: DroneApi = ctx.drone_api
        return NEDCoordinate.distance(drone_api.local_position, self.target_position) <= NAV_THRESHOLD
