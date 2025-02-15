from typing import Optional

from agent.constants import DELTA_TIME, NAV_THRESHOLD
from api import DroneApi, LidarApi
from common.bug_navigator import bugNavigator
from common.logger import Logger
from common.ned_coordinate import NEDCoordinate

from .behavior import Behavior


class BonusBehavior(Behavior):

    def __init__(self, logger: Logger, drone_api: DroneApi, lidar_api: LidarApi):
        super().__init__(logger)
        self.drone_api = drone_api
        self.lidar_api = lidar_api
        self.bug_navigator = bugNavigator(0.7)

    def on_enter(self):
        self.target_position: NEDCoordinate = NEDCoordinate(17, 8, self.drone_api.home_position.z)

    def execute(self):
        obstacle_points = self.lidar_api.get_obstacle_points_2d(max_distance=5.0)
        current_location = self.drone_api.local_position
        vel = self.bug_navigator.compute_pos(
            current_position=current_location,
            target_position=self.target_position,
            obstacle_points=obstacle_points,
        )

        self.logger.info(f"target position: {self.target_position}, current position: {current_location}, vel: {vel}")

        # 往 target_position 移動, 速度大小是 self.speed
        # dist = NEDCoordinate.distance(current_location, self.target_position)
        # vel = (self.target_position - current_location).normalized * min(self.speed, dist)
        # self.drone_api.move_with_velocity(vel, DELTA_TIME)

        self.drone_api.move_with_velocity(vel)

    def get_next_state(self) -> Optional[str]:
        if False:  # TODO: 如果畫面中有出現 aruco marker
            return None  # TODO 開始精準定位
        return None
