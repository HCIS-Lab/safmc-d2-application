# TODO[lnfu] split to 3 state: DESCEND -> LOAD -> ASCEND

from typing import Optional

from agent.constants import (
    NAVIGATION_HEIGHT_TOLERANCE,
    LOAD_HEIGHT,
    NAVIGATION_GOAL_TOLERANCE,
)
from api import ApiRegistry, MagnetApi, MediatorApi, Px4Api
from common.coordinate import Coordinate
from common.logger import Logger

from .behavior import Behavior


class LoadBehavior(Behavior):

    px4_api: Px4Api
    magnet_api: MagnetApi
    mediator_api: MediatorApi

    def __init__(self, logger: Logger):
        super().__init__(logger)
        self.px4_api = ApiRegistry.get(Px4Api)
        self.magnet_api = ApiRegistry.get(MagnetApi)
        self.mediator_api = ApiRegistry.get(MediatorApi)

    def on_enter(self):
        self.origin_position: Coordinate = self.px4_api.local_position
        self.load_position: Coordinate = self.px4_api.local_position
        self.load_position.z = self.px4_api.local_position.z - LOAD_HEIGHT
        self.target_position = self.load_position

    def execute(self):

        self.logger.info(
            f"target position: {self.target_position}, current position: {self.px4_api.local_position}"
        )

        if (
            Coordinate.distance_2d(self.px4_api.local_position, self.load_position)
            <= NAVIGATION_GOAL_TOLERANCE
            and (self.px4_api.local_position.z - self.load_position.z)
            < NAVIGATION_HEIGHT_TOLERANCE
        ):
            self.magnet_api.activate_magnet()

        if self.magnet_api.is_loaded:
            self.logger.info("successfully loaded, reascending to takeoff height")
            self.target_position = self.origin_position

        self.px4_api.move_to(self.target_position)

    def get_next_state(self) -> Optional[str]:
        if not self.px4_api.is_armed:
            self.px4_api.set_resume_state("load")  # TODO[lnfu] 留下/不留下?
            return "idle"

        if (
            self.magnet_api.is_loaded
            and Coordinate.distance(self.px4_api.local_position, self.origin_position)
            <= NAVIGATION_GOAL_TOLERANCE
        ):
            return "walk_to_hotspot"
        return None
