from typing import Optional

from api import ApiRegistry, ArucoApi, MediatorApi, Px4Api, UwbApi
from common.coordinate import Coordinate
from common.logger import Logger

from .behavior import Behavior


class WalkToSupplyBehavior(Behavior):

    _aruco_api: ArucoApi
    _px4_api: Px4Api
    _mediator_api: MediatorApi
    _uwb_api: UwbApi

    _walk_speed: float
    _walk_goal_radius: float

    def __init__(self, walk_goal_radius: float, walk_speed: float):
        self._aruco_api = ApiRegistry.get(ArucoApi)
        self._px4_api = ApiRegistry.get(Px4Api)
        self._mediator_api = ApiRegistry.get(MediatorApi)
        self._uwb_api = ApiRegistry.get(UwbApi)

        self._walk_goal_radius = walk_goal_radius
        self._walk_speed = walk_speed

    def on_enter(self):
        self.target_index = 0

        self._aruco_api.set_target_marker_id(self._mediator_api.supply_zone_marker_id)
        self._aruco_api.reset()

    def execute(self):
        self._px4_api.change_control_field("velocity")

        target_p = self._uwb_api.get_supply_zone_local_ps(
            self._mediator_api.supply_zone_code, self._px4_api.local_position
        )[self.target_index]
        current_p = self._px4_api.local_position

        # 往 target_position 移動, 速度大小是 self.speed
        vel = Coordinate.clamp_magnitude_2d(target_p - current_p, self._walk_speed)
        self._px4_api.move_with_velocity_2d(vel)

        # TODO[lnfu] 加上避障 (APF)

        # 如果到達 target_p, 換方向
        if Coordinate.distance_2d(current_p, target_p) <= self._walk_goal_radius:
            Logger.info(f"reached target position, changing to next target")
            self.target_index = (self.target_index + 1) % len(
                self._mediator_api.supply_zone_points
            )  # 0, 1, 2, 3, 0, ...

    def get_next_state(self) -> Optional[str]:
        if not self._px4_api.is_armed:
            return "idle"

        if self._aruco_api.is_marker_detected:  # 偵測到目標的 ArUco Marker
            return "align_to_supply"  # 開始精準定位

        return None
