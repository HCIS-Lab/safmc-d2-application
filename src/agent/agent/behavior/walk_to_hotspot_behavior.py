from typing import Optional

from agent.constants import DELTA_TIME, NAV_THRESHOLD
from api import DroneApi, LidarApi
from common.logger import Logger
from common.ned_coordinate import NEDCoordinate
from common.apf_navigator import ApfNavigator

from .behavior import Behavior


class WalkToHotspotBehavior(Behavior):

    def __init__(self, logger: Logger, drone_api: DroneApi, lidar_api: LidarApi):
        super().__init__(logger)
        self.drone_api = drone_api
        self.lidar_api = lidar_api
        self.speed: float = 0.5

        self.apf_navigator = ApfNavigator(
            k_att=0.5,
            k_rep=3.0,
            safe_dist=1.5,
            max_speed=self.speed
        )


    def on_enter(self):
        # TODO hotspot 位置如何決定? 應該是要 mediator 告訴他?
        self.target_position: NEDCoordinate = NEDCoordinate(13, 6.2135, self.drone_api.local_position.z)


    def execute(self):
        obstacle_points = self.lidar_api.get_obstacle_points_2d(max_distance=5.0)
        current_location = self.drone_api.local_position
        heading = self.drone_api.heading
        vel = self.apf_navigator.compute_velocity(
            current_position=current_location,
            target_position=self.target_position,
            obstacle_points=obstacle_points,
            heading=heading
        )

        self.logger.info(f"target position: {self.target_position}, current position: {current_location}, vel: {vel}")

        # 往 target_position 移動, 速度大小是 self.speed
        # dist = NEDCoordinate.distance(current_location, self.target_position)
        # vel = (self.target_position - current_location).normalized * min(self.speed, dist)
        # self.drone_api.move_with_velocity(vel, DELTA_TIME)

        self.drone_api.move_with_velocity(vel, DELTA_TIME)


    def get_next_state(self) -> Optional[str]:
        if False: # TODO: 如果畫面中有出現 aruco marker
            return None # TODO 開始精準定位
        return None
