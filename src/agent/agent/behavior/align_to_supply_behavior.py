from typing import Optional

from api import ApiRegistry, ArucoApi, MediatorApi, Px4Api
from common.coordinate import Coordinate
from common.logger import Logger

from .behavior import Behavior


class AlignToSupplyBehavior(Behavior):

    _aruco_api: ArucoApi
    _px4_api: Px4Api
    _mediator_api: MediatorApi

    _speed = 0.3  # 最大速度 TODO move to yaml/constant file

    _navigation_aruco_tolerance: float  # TODO[lnfu] rename

    def __init__(self, logger: Logger, navigation_aruco_tolerance: float):
        super().__init__(logger)
        self._aruco_api = ApiRegistry.get(ArucoApi)
        self._px4_api = ApiRegistry.get(Px4Api)
        self._mediator_api = ApiRegistry.get(MediatorApi)
        self._navigation_aruco_tolerance = navigation_aruco_tolerance

    def execute(self):
        self._px4_api.change_control_field("velocity")
        vel = -self._aruco_api.marker_position
        vel = Coordinate.clamp_magnitude_2d(vel, self._speed)
        self._px4_api.move_with_velocity_2d(vel)

    def get_next_state(self) -> Optional[str]:
        if not self._px4_api.is_armed:
            self._px4_api.set_resume_state("align_to_supply")  # TODO[lnfu] 留下/不留下?
            return "idle"

        if (
            self._aruco_api.marker_position.magnitude_2d
            <= self._navigation_aruco_tolerance
        ):
            return "load"  # 等待
        if self._aruco_api.idle_time.nanoseconds > 3e9:  # 3sec
            return "walk_to_supply"  # 偵測不到目標 marker，退回去重走

        return None
