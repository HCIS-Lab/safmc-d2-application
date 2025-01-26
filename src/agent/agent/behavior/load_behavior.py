from typing import Optional

from agent.constants import LOAD_HEIGHT, NAV_THRESHOLD
from api import DroneApi, MagnetApi
from common.logger import Logger
from common.ned_coordinate import NEDCoordinate

from .behavior import Behavior


class LoadBehavior(Behavior):

    def __init__(self, logger: Logger, drone_api: DroneApi, magnet_api: MagnetApi):
        super().__init__(logger)
        self.drone_api = drone_api
        self.magnet_api = magnet_api

    def on_enter(self):

        # TODO supply zone 的兩端點位置如何決定?
        self.origin_position: NEDCoordinate = self.drone_api.local_position
        
        self.load_position: NEDCoordinate = self.drone_api.local_position
        self.load_position.z = self.drone_api.start_position.z - LOAD_HEIGHT

        self.target_position = self.load_position

    def execute(self):

        self.logger.info(f"target position: {self.target_position}")
        self.logger.info(f"current position: {self.drone_api.local_position}")

        if NEDCoordinate.distance(self.drone_api.local_position, self.load_position) <= NAV_THRESHOLD:
            self.magnet_api.activate_magnet()

        if self.magnet_api.is_loaded:
            self.logger.info("successfully loaded, reascending to takeoff height")
            self.target_position = self.origin_position

        self.drone_api.move_to(self.target_position)

    def get_next_state(self) -> Optional[str]:
        if self.magnet_api.is_loaded and NEDCoordinate.distance(self.drone_api.local_position, self.origin_position) <= NAV_THRESHOLD:
            return "walk_to_hotspot"
        return None
