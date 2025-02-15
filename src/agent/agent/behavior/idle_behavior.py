from typing import Optional

from api import DroneApi, MediatorApi
from common.logger import Logger

from .behavior import Behavior


class IdleBehavior(Behavior):
    def __init__(self, logger: Logger, drone_api: DroneApi, mediator_api: MediatorApi):
        super().__init__(logger)
        self.drone_api = drone_api
        self.mediator_api = mediator_api

    def execute(self):
        pf_pass = self.drone_api.is_each_pre_flight_check_passed
        self.logger.info(f"Preflight checks passed: {pf_pass}")

        if pf_pass:
            self.mediator_api.online()

    def get_next_state(self) -> Optional[str]:
        if self.mediator_api.is_ready_to_arm:
            return "arm"
        return None
