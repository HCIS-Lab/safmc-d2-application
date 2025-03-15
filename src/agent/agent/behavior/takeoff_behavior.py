from typing import Optional

from api import ApiRegistry, MagnetApi, MediatorApi, Px4Api
from common.coordinate import Coordinate

from .behavior import Behavior


class TakeoffBehavior(Behavior):

    _px4_api: Px4Api
    _magnet_api: MagnetApi
    _mediator_api: MediatorApi

    def __init__(self, walk_goal_radius: float, takeoff_height: float):
        self._px4_api = ApiRegistry.get(Px4Api)
        self._magnet_api = ApiRegistry.get(MagnetApi)
        self._mediator_api = ApiRegistry.get(MediatorApi)
        self._walk_goal_radius = walk_goal_radius
        self._takeoff_height = takeoff_height

    def on_enter(self):
        self.target_p = (
            self._px4_api.local_position - Coordinate.down * self._takeoff_height
        )

    def execute(self):
        self._px4_api.change_control_field("position")
        self._px4_api.move_to(self.target_p)

    def get_next_state(self) -> Optional[str]:
        if not self._px4_api.is_armed:
            return "idle"

        # 如果到達目標高度, 就切換到 walk_to_supply | walk_to_hotspot
        if (
            Coordinate.distance(self._px4_api.local_position, self.target_p)
            <= self._walk_goal_radius
        ):
            return "walk_to_hotspot" if self._magnet_api.is_loaded else "walk_to_supply"
        return None
