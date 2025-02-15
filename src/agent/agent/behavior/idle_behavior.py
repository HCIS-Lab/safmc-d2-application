from typing import Optional

from api import DroneApi, MediatorApi
from common.logger import Logger
from common.ned_coordinate import NEDCoordinate
from agent.constants import TAKEOFF_HEIGHT

from .behavior import Behavior


class IdleBehavior(Behavior):
    def __init__(self, logger: Logger, drone_api: DroneApi, mediator_api: MediatorApi):
        super().__init__(logger)
        self.drone_api = drone_api
        self.mediator_api = mediator_api

    def execute(self):
        self.logger.info(f"Armed status: {self.drone_api.is_armed}")
        self.logger.info(f"Vehicle timestamp: {self.drone_api.vehicle_timestamp}")
        self.logger.info(f"Preflight checks passed: {self.drone_api.is_each_pre_flight_check_passed}")

        self.mediator_api.online()

    def get_next_state(self) -> Optional[str]:
        if self.mediator_api.arm_ready:
            return "arm"
        return None

    # def on_exit(self):
    #     self.drone_api.reset_origin(NEDCoordinate(0, 0, 0)) # TODO
    #     self.drone_api.reset_start_position()
