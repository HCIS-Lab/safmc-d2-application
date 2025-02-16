from typing import Optional

from api import DroneApi, MediatorApi
from common.logger import Logger

from .behavior import Behavior


class ArmBehavior(Behavior):
    def __init__(self, logger: Logger, drone_api: DroneApi, mediator_api: MediatorApi):
        super().__init__(logger)
        self.drone_api = drone_api
        self.mediator_api = mediator_api

    def execute(self):
        self.logger.info(f"Armed status: {self.drone_api.is_armed}")

        if not self.drone_api.is_armed:
            self.drone_api.activate_offboard_control_mode()  # TODO 要不要移動到 IDLE state?
            self.drone_api.arm()
        else:
            self.mediator_api.arm_ack()

    def get_next_state(self) -> Optional[str]:
        if self.mediator_api.received_disarm_signal:
            return "idle"
        if self.mediator_api.is_ready_to_takeoff:
            return "takeoff"
        return None
