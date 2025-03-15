from typing import Optional

from agent.constants import NAVIGATION_GOAL_TOLERANCE, TAKEOFF_HEIGHT
from api import ApiRegistry, MagnetApi, MediatorApi, Px4Api
from common.coordinate import Coordinate
from common.logger import Logger

from .behavior import Behavior


class TakeoffBehavior(Behavior):

    px4_api: Px4Api
    magnet_api: MagnetApi
    mediator_api: MediatorApi

    def __init__(self, logger: Logger):
        super().__init__(logger)
        self.px4_api = ApiRegistry.get(Px4Api)
        self.magnet_api = ApiRegistry.get(MagnetApi)
        self.mediator_api = ApiRegistry.get(MediatorApi)

    def on_enter(self):
        self.target_position = (
            self.px4_api.local_position - Coordinate.down * TAKEOFF_HEIGHT
        )

    def execute(self):
        self.px4_api.change_control_field("position")
        self.px4_api.move_to(self.target_position)

    def __has_reached_final_position(self) -> bool:
        return (
            Coordinate.distance(self.px4_api.local_position, self.target_position)
            <= NAVIGATION_GOAL_TOLERANCE
        )

    def get_next_state(self) -> Optional[str]:
        if not self.px4_api.is_armed:
            return "idle"

        if self.__has_reached_final_position():
            if self.px4_api.last_state != "walk_to_supply":  # TODO[lnfu] 留下/不留下?
                return self.px4_api.last_state
            return "walk_to_hotspot" if self.magnet_api.is_loaded else "walk_to_supply"
        return None
