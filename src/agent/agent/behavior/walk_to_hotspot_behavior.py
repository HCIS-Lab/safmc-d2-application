from typing import Optional

from api import ArucoApi, DroneApi, LidarApi, MediatorApi
from common.apf_navigator import ApfNavigator
from common.coordinate import Coordinate
from common.logger import Logger

from .behavior import Behavior


class WalkToHotspotBehavior(Behavior):

    def __init__(self, logger: Logger, drone_api: DroneApi, lidar_api: LidarApi, aruco_api: ArucoApi, mediator_api: MediatorApi):
        super().__init__(logger)
        self.drone_api = drone_api
        self.lidar_api = lidar_api
        self.aruco_api = aruco_api
        self.mediator_api = mediator_api
        self.speed: float = 0.5

        self.apf_navigator = ApfNavigator(
            k_att=0.5,
            k_rep=3.0,
            safe_dist=1.5,
            max_speed=self.speed
        )

    def on_enter(self):
        self.target_position: Coordinate = self.mediator_api.drop_zone

        # 重設 ArUco Marker
        # TODO 透過 mediator 設定 target marker id
        self.aruco_api.reset()  # TODO 或許可以直接把設定 target 寫在 reset() 裡面

    def execute(self):
        self.drone_api.change_control_field("velocity")

        obstacle_points = self.lidar_api.get_obstacle_points_2d(max_distance=5.0)
        mediator_points = [(point.x, point.y) for point in self.mediator_api.obstacle_array]

        vel = self.apf_navigator.compute_velocity(
            current_position=self.drone_api.local_position,
            target_position=self.target_position,
            obstacle_points=obstacle_points,
            mediator_obstacles=mediator_points,
            heading=self.drone_api.heading
        )

        self.drone_api.move_with_velocity(vel)  # TODO 2D?

    def get_next_state(self) -> Optional[str]:
        if not self.mediator_api.is_ok_to_arm:  # disarm
            return "idle"
        if not self.drone_api.is_armed:
            self.drone_api.set_resume_state("walk_to_hotspot")  # TODO 留下/不留下?
            return "arm"

        if self.aruco_api.is_marker_detected:  # 偵測到目標的 ArUco Marker
            return "align_to_hotspot"  # 開始精準定位
        return None
