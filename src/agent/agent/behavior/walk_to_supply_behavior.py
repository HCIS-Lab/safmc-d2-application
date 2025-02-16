from typing import Optional

from agent.constants import NAV_THRESHOLD
from api import ArucoApi, DroneApi, MediatorApi
from common.coordinate import Coordinate
from common.logger import Logger

from .behavior import Behavior


class WalkToSupplyBehavior(Behavior):

    def __init__(self, logger: Logger, drone_api: DroneApi, aruco_api: ArucoApi, mediator_api: MediatorApi):
        super().__init__(logger)
        self.drone_api = drone_api
        self.aruco_api = aruco_api
        self.mediator_api = mediator_api
        self.speed: float = 0.5

    def on_enter(self):
        self.target_index: int = 0
        self.target_position: Coordinate = self.mediator_api.supply_zone[self.target_index]
        self.target_position.z = self.drone_api.local_position.z  # TODO 用 drone_api?????

        # 重設 ArUco Marker
        # TODO 透過 mediator 設定 target marker id
        self.aruco_api.reset()  # TODO 或許可以直接把設定 target 寫在 reset() 裡面

    def execute(self):
        current_location = self.drone_api.local_position

        # 往 target_position 移動, 速度大小是 self.speed
        dist = Coordinate.distance(current_location, self.target_position)
        vel = (self.target_position - current_location).normalized * min(self.speed, dist)
        self.drone_api.move_with_velocity(vel)

        # TODO 加上避障

        self.logger.info(f"target position: {self.target_position}, current position: {current_location}, vel: {vel}")

        if Coordinate.distance(self.drone_api.local_position, self.target_position) <= NAV_THRESHOLD:
            # 回頭 (A to B or B to A)
            self.logger.info(f"reached target position, changing to next target")
            self.target_index = (self.target_index + 1) % len(self.mediator_api.supply_zone)  # 0, 1, 2, 3, 0, ...
            self.target_position = self.mediator_api.supply_zone[self.target_index]  # Update target position
            self.target_position.z = self.drone_api.local_position.z  # TODO 用 drone_api?????

    def get_next_state(self) -> Optional[str]:
        if self.mediator_api.received_disarm_signal:
            return "idle"
        if not self.drone_api.is_armed:
            self.drone_api.set_resume_state("walk_to_supply")
            return "arm"
        if self.aruco_api.is_marker_detected:  # 偵測到目標的 ArUco Marker
            return "align_to_supply"  # 開始精準定位
        return None
