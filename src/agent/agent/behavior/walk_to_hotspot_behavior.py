from typing import Optional

from api import ApiRegistry, ArucoApi, LidarApi, MediatorApi, Px4Api, UwbApi
from common.apf_navigator import ApfNavigator

from .behavior import Behavior

import numpy as np


class WalkToHotspotBehavior(Behavior):

    _aruco_api: ArucoApi
    _px4_api: Px4Api
    _lidar_api: LidarApi
    _mediator_api: MediatorApi
    _uwb_api: UwbApi

    _walk_speed = 0.5

    def __init__(self):
        self._aruco_api = ApiRegistry.get(ArucoApi)
        self._px4_api = ApiRegistry.get(Px4Api)
        self._lidar_api = ApiRegistry.get(LidarApi)
        self._mediator_api = ApiRegistry.get(MediatorApi)
        self._uwb_api = ApiRegistry.get(UwbApi)

        self.apf_navigator = ApfNavigator(
            k_att=0.5, k_rep=3.0, safe_dist=1.5, max_speed=self._walk_speed
        )

    def on_enter(self):
        self._aruco_api.set_target_marker_id(self._mediator_api.drop_zone_marker_id)
        self._aruco_api.reset()

    def execute(self):
        self._px4_api.change_control_field("velocity")

        target_p = self._uwb_api.get_drop_zone_local_p(
            self._mediator_api.drop_zone_code, self._px4_api.local_position
        )

        heading = self._aruco_api.heading
        cos_theta = np.cos(heading)
        sin_theta = np.sin(heading)

        obstacle_points = [
            (cos_theta * x - sin_theta * y, sin_theta * x + cos_theta * y)
            for x, y in self._lidar_api.get_obstacle_points_2d(max_distance=5.0)
        ]

        other_drone_points = [
            (point.x, point.y)
            for point in self._uwb_api.get_other_agent_local_ps(
                self._px4_api.local_position
            )
        ]

        vel = self.apf_navigator.compute_velocity(
            current_position=self._px4_api.local_position,
            target_position=target_p,
            obstacle_points=obstacle_points,
            mediator_obstacles=other_drone_points,
            heading=self._px4_api.heading,
        )

        self._px4_api.move_with_velocity(vel)  # TODO 2D?

    def get_next_state(self) -> Optional[str]:
        if not self._px4_api.is_armed:
            return "idle"

        if self._aruco_api.is_marker_detected:  # 偵測到目標的 ArUco Marker
            return "align_to_hotspot"  # 開始精準定位

        return None
