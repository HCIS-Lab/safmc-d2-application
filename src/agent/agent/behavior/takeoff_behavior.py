from typing import Optional

from agent.constants import NAV_THRESHOLD, TAKEOFF_HEIGHT
from api import DroneApi, MagnetApi
from common.logger import Logger
from common.ned_coordinate import NEDCoordinate

from .behavior import Behavior


class TakeoffBehavior(Behavior):

    def __init__(self, logger: Logger, drone_api: DroneApi, magnet_api: MagnetApi):
        super().__init__(logger)
        self.drone_api = drone_api
        self.magnet_api = magnet_api

    def on_enter(self):
        self.target_position = self.drone_api.local_position - NEDCoordinate.down * TAKEOFF_HEIGHT

    def execute(self):
        self.log_position(self.target_position, self.drone_api.local_position)
        self.drone_api.move_to(self.target_position)

    def get_next_state(self) -> Optional[str]:
        if self.__has_reached_final_position():
            return "walk_to_hotspot"
            self.logger.info("takeoff altitude reached")
            return "walk_to_hotspot" if self.magnet_api.is_loaded else "walk_to_supply"
        return None

    def __has_reached_final_position(self) -> bool:
        return NEDCoordinate.distance(self.drone_api.local_position, self.target_position) <= NAV_THRESHOLD
