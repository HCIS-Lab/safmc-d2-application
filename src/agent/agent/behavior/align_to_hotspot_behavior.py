
from typing import Optional

from agent.constants import ARUCO_DIST_THRESHOLD
from api import ArucoApi, DroneApi, MediatorApi
from common.coordinate import Coordinate
from common.logger import Logger

from .behavior import Behavior


class AlignToHotspotBehavior(Behavior):  # 精準定位

    def __init__(self, logger: Logger, drone_api: DroneApi, mediator_api: MediatorApi,  aruco_api: ArucoApi):
        super().__init__(logger)
        self.drone_api = drone_api
        self.mediator_api = mediator_api
        self.aruco_api = aruco_api
        self.speed: float = 0.3  # 最大速度

    def execute(self):
        self.drone_api.change_control_field("velocity")

        vel = -self.aruco_api.marker_position
        vel = Coordinate.clamp_magnitude_2d(vel, self.speed)
        self.drone_api.move_with_velocity_2d(vel)

    def get_next_state(self) -> Optional[str]:
        if not self.drone_api.is_armed:
            self.drone_api.set_resume_state("align_to_hotspot")
            return "idle"

        if self.aruco_api.marker_position.magnitude_2d <= ARUCO_DIST_THRESHOLD:
            return "wait"  # 等待
        if self.aruco_api.idle_time.nanoseconds > 3e9:  # 3sec
            return "walk_to_hotspot"                    # 偵測不到目標 marker，退回去重走

        return None
