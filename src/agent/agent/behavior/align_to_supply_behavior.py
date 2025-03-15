from typing import Optional

from api import ApiRegistry, ArucoApi, MediatorApi, Px4Api
from common.coordinate import Coordinate

from .behavior import Behavior


class AlignToSupplyBehavior(Behavior):

    _aruco_api: ArucoApi
    _px4_api: Px4Api
    _mediator_api: MediatorApi

    _align_speed: float
    _align_goal_radius: float

    def __init__(self, align_goal_radius: float, align_speed: float):
        self._aruco_api = ApiRegistry.get(ArucoApi)
        self._px4_api = ApiRegistry.get(Px4Api)
        self._mediator_api = ApiRegistry.get(MediatorApi)

        self._align_goal_radius = align_goal_radius
        self._align_speed = align_speed

    def execute(self):
        self._px4_api.change_control_field("velocity")

        # TODO[lnfu] 到底要是水平移動, 然後 landing, load, takeoff, 還是直接 align 的時候垂直, load, takeoff???
        vel = Coordinate.clamp_magnitude(
            self._aruco_api.marker_position_diff, self._align_speed
        )
        self._px4_api.move_with_velocity(vel)

    def get_next_state(self) -> Optional[str]:
        if not self._px4_api.is_armed:
            return "idle"

        if self._aruco_api.marker_position_diff.magnitude <= self._align_goal_radius:
            return "load"

        # TODO[lnfu]! 等等回來重寫
        # if self._aruco_api.idle_time.nanoseconds > 3e9:  # 3sec
        #     return "walk_to_supply"  # 偵測不到目標 marker，退回去重走

        return None
