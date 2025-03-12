from typing import Optional

from agent.constants import ARUCO_DIST_THRESHOLD
from api import ApiRegistry, ArucoApi, MediatorApi, Px4Api
from common.coordinate import Coordinate
from common.logger import Logger

from .behavior import Behavior


class AlignToHotspotBehavior(Behavior):

    aruco_api: ArucoApi
    px4_api: Px4Api
    mediator_api: MediatorApi

    speed = 0.3  # 最大速度 TODO move to yaml/constant file

    def __init__(self, logger: Logger):
        super().__init__(logger)
        self.aruco_api = ApiRegistry.get(ArucoApi)
        self.px4_api = ApiRegistry.get(Px4Api)
        self.mediator_api = ApiRegistry.get(MediatorApi)

    def execute(self):
        self.px4_api.change_control_field("velocity")
        vel = Coordinate.clamp_magnitude_2d(-self.aruco_api.marker_position, self.speed)
        self.px4_api.move_with_velocity_2d(vel)

    def get_next_state(self) -> Optional[str]:
        if not self.px4_api.is_armed:
            self.px4_api.set_resume_state("align_to_hotspot")
            return "idle"

        if self.aruco_api.marker_position.magnitude_2d <= ARUCO_DIST_THRESHOLD:
            return "wait"  # 等待
        if self.aruco_api.idle_time.nanoseconds > 3e9:  # 3sec TODO magic number
            return "walk_to_hotspot"  # 偵測不到目標 marker，退回去重走

        return None
