from typing import Optional

from agent.constants import NAV_THRESHOLD
from api import ArucoApi, DroneApi, MediatorApi
from common.logger import Logger
from common.ned_coordinate import NEDCoordinate

from .behavior import Behavior


class WalkToSupplyBehavior(Behavior):

    def __init__(self, logger: Logger, drone_api: DroneApi, aruco_api: ArucoApi, mediator_api: MediatorApi):
        super().__init__(logger)
        self.drone_api = drone_api
        self.aruco_api = aruco_api
        self.mediator_api = mediator_api
        self.speed: float = 0.5

    def on_enter(self):

        # TODO supply zone 的兩端點位置如何決定?
        self.mediator_api.send_status(3, self.drone_api.local_position)
        self.point_a, self.point_b = self.get_target()
        self.target_position: NEDCoordinate = self.point_a

        # 重設 ArUco Marker
        # TODO 透過 mediator 設定 target marker id
        self.aruco_api.reset()  # TODO 或許可以直接把設定 target 寫在 reset() 裡面

    def execute(self):
        current_location = self.drone_api.local_position
        self.mediator_api.send_status(3, current_location)

        self.logger.info(f"target position: {self.target_position}, {self.drone_api.global_position}, {self.drone_api.local_position}, {self.mediator_api.supply_zone[0]}")
        self.logger.info(f"current position: {current_location}")

        # 往 target_position 移動, 速度大小是 self.speed
        dist = NEDCoordinate.distance(current_location, self.target_position)
        vel = (self.target_position - current_location).normalized * min(self.speed, dist)
        self.drone_api.move_with_velocity(vel)

        if NEDCoordinate.distance(self.drone_api.local_position, self.target_position) <= NAV_THRESHOLD:
            # 回頭 (A to B or B to A)
            self.logger.info(f"reached target position, changing direction")
            self.target_position = self.point_b if self.target_position == self.point_a else self.point_a

    def get_next_state(self) -> Optional[str]:
        if self.aruco_api.is_marker_detected:  # 偵測到目標的 ArUco Marker
            return "align_to_supply"  # 開始精準定位
        return None
    
    def get_target(self):
        point_a: NEDCoordinate = NEDCoordinate(self.mediator_api.supply_zone[0][0], self.mediator_api.supply_zone[0][1], self.mediator_api.supply_zone[0][2])
        point_b: NEDCoordinate = NEDCoordinate(self.mediator_api.supply_zone[1][0], self.mediator_api.supply_zone[1][1], self.mediator_api.supply_zone[1][2])
        point_a = self.to_local(point_a)
        point_b = self.to_local(point_b)
        return point_a, point_b
    
    def to_local(self, target_global):
        global_position = self.drone_api.global_position
        local_position = self.drone_api.local_position
        x = target_global.x - global_position.x + local_position.x
        y = target_global.y - global_position.y + local_position.y
        z = local_position.z
        return NEDCoordinate(x, y, z)
