from typing import Optional

from api import ApiRegistry, DroneApi, LidarApi, MediatorApi
from common.bug_navigator import BugNavigator
from common.logger import Logger

from .behavior import Behavior


class BonusBehavior(Behavior):

    drone_api: DroneApi
    mediator_api: MediatorApi
    lidar_api: LidarApi

    def __init__(self, logger: Logger):
        super().__init__(logger)
        self.drone_api = ApiRegistry.get(DroneApi)
        self.mediator_api = ApiRegistry.get(MediatorApi)
        self.lidar_api = ApiRegistry.get(LidarApi)

        self.bug_navigator = BugNavigator(0.7)

    def on_enter(self):
        # TODO
        # self.target_position: Coordinate = Coordinate(
        #     17, 8, self.drone_api.home_position.z
        # )
        pass

    def execute(self):
        obstacle_points = self.lidar_api.get_obstacle_points_2d(
            max_distance=5.0
        )  # TODO 5.0
        current_location = self.drone_api.local_position
        vel = self.bug_navigator.compute_pos(
            current_position=current_location,
            target_position=self.target_position,
            obstacle_points=obstacle_points,
        )
        self.drone_api.move_with_velocity(vel)

    def get_next_state(self) -> Optional[str]:
        if not self.drone_api.is_armed:
            self.drone_api.set_resume_state("bonus")  # TODO 留下/不留下?
            return "idle"

        if False:  # TODO: 如果畫面中有出現 aruco marker
            return None  # TODO 開始精準定位
        return None
