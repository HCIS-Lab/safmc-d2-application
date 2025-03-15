from typing import Optional

from api import ApiRegistry, MagnetApi, MediatorApi, Px4Api
from common.coordinate import Coordinate
from common.logger import Logger

from .behavior import Behavior


class TakeoffBehavior(Behavior):

    _px4_api: Px4Api
    _magnet_api: MagnetApi
    _mediator_api: MediatorApi

    def __init__(
        self, logger: Logger, navigation_goal_tolerance: float, takeoff_height: float
    ):
        super().__init__(logger)
        self._px4_api = ApiRegistry.get(Px4Api)
        self._magnet_api = ApiRegistry.get(MagnetApi)
        self._mediator_api = ApiRegistry.get(MediatorApi)
        self._navigation_goal_tolerance = navigation_goal_tolerance
        self._takeoff_height = takeoff_height

    def on_enter(self):
        self.target_position = (
            self._px4_api.local_position - Coordinate.down * self._takeoff_height
        )

    def execute(self):
        self._px4_api.change_control_field("position")
        self._px4_api.move_to(self.target_position)

    def __has_reached_final_position(self) -> bool:
        return (
            Coordinate.distance(self._px4_api.local_position, self.target_position)
            <= self._navigation_goal_tolerance
        )

    def get_next_state(self) -> Optional[str]:
        if not self._px4_api.is_armed:
            return "idle"

        if self.__has_reached_final_position():
            if self._px4_api.last_state != "walk_to_supply":  # TODO[lnfu] 留下/不留下?
                return self._px4_api.last_state
            return "walk_to_hotspot" if self._magnet_api.is_loaded else "walk_to_supply"
        return None
