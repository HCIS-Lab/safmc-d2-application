from typing import Optional
from api import ApiRegistry, MagnetApi, MediatorApi, Px4Api
from common.coordinate import Coordinate
from common.logger import Logger
from .behavior import Behavior


class TakeoffBehavior(Behavior):

    def __init__(self, walk_goal_radius: float, takeoff_height: float):
        self._px4_api = ApiRegistry.get(Px4Api)
        self._magnet_api = ApiRegistry.get(MagnetApi)
        self._mediator_api = ApiRegistry.get(MediatorApi)

        self._walk_goal_radius = walk_goal_radius
        self._takeoff_height = takeoff_height

        self._target_p: Optional[Coordinate] = None

    def _compute_target_position(self):
        """計算目標起飛位置"""
        if self._px4_api.local_position is None:
            Logger.error("CANNOT GET local_position")
            return None
        return self._px4_api.local_position - Coordinate.down * self._takeoff_height

    def on_enter(self):
        Logger.info("ENTER TAKEOFF STATE")
        self._target_p = self._compute_target_position()

    def execute(self):
        if self._target_p is None:
            self._target_p = self._compute_target_position()
            if self._target_p is None:
                return  # 無法獲取 local_position，無法執行

        self._px4_api.change_control_field("position")
        self._px4_api.move_position_to(self._target_p)

    def get_next_state(self) -> Optional[str]:
        return "walk_to_supply"

        if not self._px4_api.is_armed:
            return "idle"

        # 檢查是否到達目標高度，決定下一步行為
        if (
            Coordinate.distance(self._px4_api.local_position, self._target_p)
            <= self._walk_goal_radius
        ):
            return "walk_to_hotspot" if self._magnet_api.is_loaded else "walk_to_supply"

        return None
