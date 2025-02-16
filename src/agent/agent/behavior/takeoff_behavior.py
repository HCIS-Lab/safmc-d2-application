from typing import Optional

from agent.constants import NAV_THRESHOLD, TAKEOFF_HEIGHT
from api import DroneApi, MagnetApi, MediatorApi
from common.coordinate import Coordinate
from common.logger import Logger

from .behavior import Behavior


class TakeoffBehavior(Behavior):

    def __init__(self, logger: Logger, drone_api: DroneApi, magnet_api: MagnetApi, mediator_api: MediatorApi):
        super().__init__(logger)
        self.drone_api = drone_api
        self.magnet_api = magnet_api
        self.mediator_api = mediator_api

    def on_enter(self):
        self.target_position = self.drone_api.local_position - Coordinate.down * TAKEOFF_HEIGHT

    def execute(self):
        self.drone_api.move_to(self.target_position)

    def on_exit(self):
        self.drone_api.reset_start_position()

    def __has_reached_final_position(self) -> bool:
        return Coordinate.distance(self.drone_api.local_position, self.target_position) <= NAV_THRESHOLD

    def get_next_state(self) -> Optional[str]:
        if not self.mediator_api.is_ok_to_arm:  # disarm
            return "idle"
        if not self.drone_api.is_armed:
            return "arm"
        if self.__has_reached_final_position():
            if (self.drone_api.last_state != "walk_to_supply"):
                return self.drone_api.last_state
            return "walk_to_hotspot" if self.magnet_api.is_loaded else "walk_to_supply"
        return None
