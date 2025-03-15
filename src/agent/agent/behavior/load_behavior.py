# TODO[lnfu] split to 3 state: DESCEND -> LOAD -> ASCEND

from typing import Optional

from api import ApiRegistry, MagnetApi, MediatorApi, Px4Api
from common.coordinate import Coordinate
from common.logger import Logger

from .behavior import Behavior


class LoadBehavior(Behavior):

    _px4_api: Px4Api
    _magnet_api: MagnetApi
    _mediator_api: MediatorApi

    def __init__(
        self,
        logger: Logger,
        navigation_goal_tolerance: float,
        navigation_height_tolerance: float,
        navigation_aruco_tolerance: float,
        load_height: float,
    ):
        super().__init__(logger)
        self._px4_api = ApiRegistry.get(Px4Api)
        self._magnet_api = ApiRegistry.get(MagnetApi)
        self._mediator_api = ApiRegistry.get(MediatorApi)
        self._navigation_goal_tolerance = navigation_goal_tolerance
        self._navigation_height_tolerance = navigation_height_tolerance
        self._navigation_aruco_tolerance = navigation_aruco_tolerance
        self._load_height = load_height

    def on_enter(self):
        self.origin_position: Coordinate = self._px4_api.local_position
        self.load_position: Coordinate = self._px4_api.local_position
        self.load_position.z = self._px4_api.local_position.z - self._load_height
        self.target_position = self.load_position

    def execute(self):

        self.logger.info(
            f"target position: {self.target_position}, current position: {self._px4_api.local_position}"
        )

        if (
            Coordinate.distance_2d(self._px4_api.local_position, self.load_position)
            <= self._navigation_goal_tolerance
            and (self._px4_api.local_position.z - self.load_position.z)
            < self._navigation_height_tolerance
        ):
            self._magnet_api.activate_magnet()

        if self._magnet_api.is_loaded:
            self.logger.info("successfully loaded, reascending to takeoff height")
            self.target_position = self.origin_position

        self._px4_api.move_to(self.target_position)

    def get_next_state(self) -> Optional[str]:
        if not self._px4_api.is_armed:
            self._px4_api.set_resume_state("load")  # TODO[lnfu] 留下/不留下?
            return "idle"

        if (
            self._magnet_api.is_loaded
            and Coordinate.distance(self._px4_api.local_position, self.origin_position)
            <= self._navigation_goal_tolerance
        ):
            return "walk_to_hotspot"
        return None
