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
        obstacle_points = self.lidar_api.get_obstacle_points_2d(max_distance=5.0)
        current_location = self.drone_api.local_position
        self.mediator_api.send_status(8, current_location)
        self.mediator_api.send_status(8, current_location)

        heading = self.drone_api.heading
        vel = self.apf_navigator.compute_velocity(
            current_position=current_location,
            target_position=self.target_position,
            obstacle_points=obstacle_points,
            heading=heading
        )

        self.logger.info(
            f"target position: {self.target_position}, {self.drone_api.global_position}, current position: {current_location}, vel: {vel}")

        self.drone_api.move_with_velocity(vel)

    def get_next_state(self) -> Optional[str]:
        if self.mediator_api.received_disarm_signal:
            return "idle"
        if not self.drone_api.is_armed:
            self.drone_api.set_resume_state("walk_to_hotspot")
            return "arm"
        if self.aruco_api.is_marker_detected:  # 偵測到目標的 ArUco Marker
            return "align_to_hotspot"  # 開始精準定位
        return None

    def get_target(self):
        target: Coordinate = Coordinate(
            self.mediator_api.drop_zone[0], self.mediator_api.drop_zone[1], self.mediator_api.drop_zone[2])
        target = self.to_local(target)
        return target

    def to_local(self, target_global):
        global_position = self.drone_api.global_position
        local_position = self.drone_api.local_position
        x = target_global.x - global_position.x + local_position.x
        y = target_global.y - global_position.y + local_position.y
        z = local_position.z
        return Coordinate(x, y, z)
