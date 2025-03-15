from typing import Optional

from api import ApiRegistry, ArucoApi, LidarApi, MediatorApi, Px4Api
from common.apf_navigator import ApfNavigator
from common.coordinate import Coordinate
from common.logger import Logger

from .behavior import Behavior


class WalkToHotspotBehavior(Behavior):

    _aruco_api: ArucoApi
    _px4_api: Px4Api
    _lidar_api: LidarApi
    _mediator_api: MediatorApi

    _walk_speed = 0.5

    def __init__(self):
        self._aruco_api = ApiRegistry.get(ArucoApi)
        self._px4_api = ApiRegistry.get(Px4Api)
        self._lidar_api = ApiRegistry.get(LidarApi)
        self._mediator_api = ApiRegistry.get(MediatorApi)

        self.apf_navigator = ApfNavigator(
            k_att=0.5, k_rep=3.0, safe_dist=1.5, max_speed=self._walk_speed
        )

    def on_enter(self):
        self.target_p: Coordinate = self._mediator_api.drop_zone_point

        self._aruco_api.set_target_marker_id(self._mediator_api.drop_zone_marker_id)
        self._aruco_api.reset()

    def execute(self):
        self._px4_api.change_control_field("velocity")

        obstacle_points = self._lidar_api.get_obstacle_points_2d(max_distance=5.0)
        mediator_points = [
            (point.x, point.y) for point in self._mediator_api.obstacle_array
        ]

        vel = self.apf_navigator.compute_velocity(
            current_position=self._px4_api.local_position,
            target_position=self.target_p,
            obstacle_points=obstacle_points,
            mediator_obstacles=mediator_points,
            heading=self._px4_api.heading,
        )

        self._px4_api.move_with_velocity(vel)  # TODO 2D?

    def get_next_state(self) -> Optional[str]:
        if not self._px4_api.is_armed:
            return "idle"

        if self._aruco_api.is_marker_detected:  # 偵測到目標的 ArUco Marker
            return "align_to_hotspot"  # 開始精準定位
        return None
