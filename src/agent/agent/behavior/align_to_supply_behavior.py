from typing import Optional

from agent.constants import ARUCO_DIST_THRESHOLD
from api import ApiRegistry, ArucoApi, MediatorApi, Px4Api
from common.coordinate import Coordinate
from common.logger import Logger

from .behavior import Behavior


class AlignToSupplyBehavior(Behavior):

    aruco_api: ArucoApi
    drone_api: Px4Api
    mediator_api: MediatorApi

    speed = 0.3  # 最大速度 TODO move to yaml/constant file

    def __init__(self, logger: Logger):
        super().__init__(logger)
        self.aruco_api = ApiRegistry.get(ArucoApi)
        self.drone_api = ApiRegistry.get(Px4Api)
        self.mediator_api = ApiRegistry.get(MediatorApi)

    def execute(self):
        self.drone_api.change_control_field("velocity")
        vel = -self.aruco_api.marker_position
        vel = Coordinate.clamp_magnitude_2d(vel, self.speed)
        self.drone_api.move_with_velocity_2d(vel)

    def get_next_state(self) -> Optional[str]:
        if not self.drone_api.is_armed:
            self.drone_api.set_resume_state("align_to_supply")  # TODO 留下/不留下?
            return "idle"

        if self.aruco_api.marker_position.magnitude_2d <= ARUCO_DIST_THRESHOLD:
            return "load"  # 等待
        if self.aruco_api.idle_time.nanoseconds > 3e9:  # 3sec
            return "walk_to_supply"  # 偵測不到目標 marker，退回去重走

        return None
