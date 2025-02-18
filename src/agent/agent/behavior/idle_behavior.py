from typing import Optional

from api import DroneApi, MediatorApi
from common.logger import Logger

from .behavior import Behavior


class IdleBehavior(Behavior):
    def __init__(self, logger: Logger, drone_api: DroneApi, mediator_api: MediatorApi):
        super().__init__(logger)
        self.drone_api = drone_api
        self.mediator_api = mediator_api

    def on_enter(self):
        self.mediator_api.reset_states()

    def execute(self):
        if self.drone_api.is_armed:
            self.drone_api.disarm()

        self.drone_api.activate_offboard_control_mode()  # TODO 一直發送會不會有問題?

        pf_pass = self.drone_api.is_each_pre_flight_check_passed
        self.logger.info(f"Preflight checks passed: {pf_pass}")

        if pf_pass:
            self.mediator_api.online()

    def get_next_state(self) -> Optional[str]:
        if self.mediator_api.is_ok_to_arm:
            return "arm"
        return None
