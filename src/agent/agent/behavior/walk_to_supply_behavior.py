from typing import Optional

from api import ApiRegistry, ArucoApi, MediatorApi, Px4Api, UwbApi
from common.coordinate import Coordinate
from common.logger import Logger

from .behavior import Behavior

import numpy as np

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
        Logger.info("ENTER WALK_TO_SUPPLY STATE")

        self.target_index = 0

        self._aruco_api.set_target_marker_id(self._mediator_api.supply_zone_marker_id)
        self._aruco_api.reset()

    def execute(self):
        self._px4_api.change_control_field("velocity")

        target_p = Coordinate(5,-5)

        heading = self._aruco_api.heading

        obstacle_points = self._lidar_api.get_obstacle_points_2d(max_distance=5.0)

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
            heading=heading,
        )

        self._px4_api.move_with_velocity(vel) 


    def get_next_state(self) -> Optional[str]:

        # if not self._px4_api.is_armed:
        #     return "idle"

        if self._aruco_api.is_marker_detected:  # 偵測到目標的 ArUco Marker
            return "align_to_supply"  # 開始精準定位

        return None
