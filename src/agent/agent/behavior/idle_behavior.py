from typing import Optional

from api import ApiRegistry, DroneApi, MediatorApi
from common.logger import Logger

from .behavior import Behavior


class IdleBehavior(Behavior):
    drone_api: DroneApi
    mediator_api: MediatorApi

    def __init__(self, logger: Logger):
        super().__init__(logger)
        self.drone_api = ApiRegistry.get(DroneApi)
        self.mediator_api = ApiRegistry.get(MediatorApi)

    def on_enter(self):
        self.mediator_api.reset_states()  # TODO

    def execute(self):
        self.drone_api.reset_start_position()  # TODO
        self.drone_api.activate_offboard_control_mode()  # TODO 一直發送會不會有問題?
        pf_pass = self.drone_api.is_each_pre_flight_check_passed
        self.logger.info(f"Preflight checks passed: {pf_pass}")

        if pf_pass:
            self.mediator_api.online()

    def get_next_state(self) -> Optional[str]:
        if self.drone_api.is_armed:
            return "takeoff"
        return None
