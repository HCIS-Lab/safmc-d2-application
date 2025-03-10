from typing import Optional

from api import ApiRegistry, ArucoApi, LidarApi, MediatorApi, Px4Api
from common.apf_navigator import ApfNavigator
from common.coordinate import Coordinate
from common.logger import Logger

from .behavior import Behavior


class WalkToHotspotBehavior(Behavior):

    aruco_api: ArucoApi
    drone_api: Px4Api
    lidar_api: LidarApi
    mediator_api: MediatorApi

    speed = 0.5

    def __init__(self, logger: Logger):
        super().__init__(logger)
        self.aruco_api = ApiRegistry.get(ArucoApi)
        self.drone_api = ApiRegistry.get(Px4Api)
        self.lidar_api = ApiRegistry.get(LidarApi)
        self.mediator_api = ApiRegistry.get(MediatorApi)

        self.apf_navigator = ApfNavigator(
            k_att=0.5, k_rep=3.0, safe_dist=1.5, max_speed=self.speed
        )

    def on_enter(self):
        self.target_position: Coordinate = self.mediator_api.drop_zone
        # 重設 ArUco Marker
        # TODO 透過 mediator 設定 target marker id
        self.aruco_api.reset()  # TODO 或許可以直接把設定 target 寫在 reset() 裡面

    def execute(self):
        self.drone_api.change_control_field("velocity")
        obstacle_points = self.lidar_api.get_obstacle_points_2d(max_distance=5.0)
        mediator_points = [
            (point.x, point.y) for point in self.mediator_api.obstacle_array
        ]

        vel = self.apf_navigator.compute_velocity(
            current_position=self.drone_api.local_position,
            target_position=self.target_position,
            obstacle_points=obstacle_points,
            mediator_obstacles=mediator_points,
            heading=self.drone_api.heading,
        )

        self.drone_api.move_with_velocity(vel)  # TODO 2D?

    def get_next_state(self) -> Optional[str]:
        if not self.drone_api.is_armed:
            self.drone_api.set_resume_state("walk_to_hotspot")  # TODO 留下/不留下?
            return "idle"

        if self.aruco_api.is_marker_detected:  # 偵測到目標的 ArUco Marker
            return "align_to_hotspot"  # 開始精準定位
        return None
