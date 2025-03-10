# TODO split to 3 state: DESCEND -> LOAD -> ASCEND

from typing import Optional

from agent.constants import HEIGHT_THRESHOLD, LOAD_HEIGHT, NAV_THRESHOLD
from api import ApiRegistry, DroneApi, MagnetApi, MediatorApi
from common.coordinate import Coordinate
from common.logger import Logger

from .behavior import Behavior


class LoadBehavior(Behavior):

    drone_api: DroneApi
    magnet_api: MagnetApi
    mediator_api: MediatorApi

    def __init__(self, logger: Logger):
        super().__init__(logger)
        self.drone_api = ApiRegistry.get(DroneApi)
        self.magnet_api = ApiRegistry.get(MagnetApi)
        self.mediator_api = ApiRegistry.get(MediatorApi)

    def on_enter(self):
        self.origin_position: Coordinate = self.drone_api.local_position
        self.load_position: Coordinate = self.drone_api.local_position
        self.load_position.z = self.drone_api.local_position.z - LOAD_HEIGHT
        self.target_position = self.load_position

    def execute(self):

        self.logger.info(
            f"target position: {self.target_position}, current position: {self.drone_api.local_position}"
        )

        if (
            Coordinate.distance_2d(self.drone_api.local_position, self.load_position)
            <= NAV_THRESHOLD
            and (self.drone_api.local_position.z - self.load_position.z)
            < HEIGHT_THRESHOLD
        ):
            self.magnet_api.activate_magnet()

        if self.magnet_api.is_loaded:
            self.logger.info("successfully loaded, reascending to takeoff height")
            self.target_position = self.origin_position

        self.drone_api.move_to(self.target_position)

    def get_next_state(self) -> Optional[str]:
        if not self.drone_api.is_armed:
            self.drone_api.set_resume_state("load")  # TODO 留下/不留下?
            return "idle"

        if (
            self.magnet_api.is_loaded
            and Coordinate.distance(self.drone_api.local_position, self.origin_position)
            <= NAV_THRESHOLD
        ):
            return "walk_to_hotspot"
        return None
