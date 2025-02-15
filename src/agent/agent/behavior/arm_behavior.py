from typing import Optional

from api import DroneApi, MediatorApi
from common.logger import Logger
from common.ned_coordinate import NEDCoordinate

from .behavior import Behavior


class ArmBehavior(Behavior):
    def __init__(self, logger: Logger, drone_api: DroneApi, mediator_api: MediatorApi):
        super().__init__(logger)
        self.drone_api = drone_api
        self.mediator_api = mediator_api

    def execute(self):
        self.logger.info(f"Armed status: {self.drone_api.is_armed}")
        self.logger.info(f"Vehicle timestamp: {self.drone_api.vehicle_timestamp}")
        self.logger.info(f"Preflight checks passed: {self.drone_api.is_each_pre_flight_check_passed}")

        # TODO 改成 mediator 來控制
        if (not self.drone_api.is_armed) and self.drone_api.is_each_pre_flight_check_passed:
            self.logger.info("Drone is ready to arm and start offboard control.")
            self.drone_api.activate_offboard_control_mode()
            self.drone_api.arm()

        if self.drone_api.is_armed:
            self.mediator_api.armed()

    def get_next_state(self) -> Optional[str]:
        if self.mediator_api.takeoff_ready:
            return "takeoff"
        return None

    def on_exit(self):
        self.drone_api.reset_origin(NEDCoordinate(0, 0, 0)) # TODO
        self.drone_api.reset_start_position()
