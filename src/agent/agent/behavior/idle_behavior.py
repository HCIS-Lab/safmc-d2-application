from typing import Optional

from api import ApiRegistry, MediatorApi, Px4Api
from common.logger import Logger

from .behavior import Behavior


class IdleBehavior(Behavior):
    px4_api: Px4Api
    mediator_api: MediatorApi

    def __init__(self, logger: Logger):
        super().__init__(logger)
        self.px4_api = ApiRegistry.get(Px4Api)
        self.mediator_api = ApiRegistry.get(MediatorApi)

    def on_enter(self):
        self.mediator_api.reset_states()  # TODO

    def execute(self):
        self.px4_api.activate_offboard_control_mode()  # TODO 一直發送會不會有問題?
        pf_pass = self.px4_api.is_each_pre_flight_check_passed
        self.logger.info(f"Preflight checks passed: {pf_pass}")

        if pf_pass:
            self.mediator_api.online()

    def get_next_state(self) -> Optional[str]:
        if self.px4_api.is_armed and self.mediator_api.is_ok_to_takeoff:
            return "takeoff"
        return None
