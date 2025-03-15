from typing import Optional

from api import ApiRegistry, ArucoApi, MediatorApi, Px4Api
from common.coordinate import Coordinate

from .behavior import Behavior


class AlignToHotspotBehavior(Behavior):

    _aruco_api: ArucoApi
    _px4_api: Px4Api
    _mediator_api: MediatorApi

    _align_speed: float
    _align_goal_radius: float
    _align_timeout: float

    def __init__(
        self, align_goal_radius: float, align_speed: float, align_timeout: float
    ):
        self._aruco_api = ApiRegistry.get(ArucoApi)
        self._px4_api = ApiRegistry.get(Px4Api)
        self._mediator_api = ApiRegistry.get(MediatorApi)

        self._align_goal_radius = align_goal_radius
        self._align_speed = align_speed
        self._align_timeout = align_timeout

    def execute(self):
        self._px4_api.change_control_field("velocity")

        vel = self._aruco_api.marker_position_diff
        vel.z = 0  # 只要水平移動
        vel = Coordinate.clamp_magnitude_2d(vel, self._align_speed)
        self._px4_api.move_with_velocity_2d(vel)

    def get_next_state(self) -> Optional[str]:
        if not self._px4_api.is_armed:
            return "idle"

        if self._aruco_api.elapsed_time.nanoseconds > self._align_timeout:
            return "walk_to_hotspot"

        if self._aruco_api.marker_position_diff.magnitude_2d <= self._align_goal_radius:
            return "wait"  # 等待

        return None
