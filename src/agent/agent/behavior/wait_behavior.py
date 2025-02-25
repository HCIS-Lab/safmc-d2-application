from typing import Optional

from api import DroneApi, MediatorApi
from common.logger import Logger

from .behavior import Behavior


class WaitBehavior(Behavior):
    def __init__(self, logger: Logger, drone_api: DroneApi, mediator_api: MediatorApi):
        super().__init__(logger)
        self.drone_api = drone_api
        self.mediator_api = mediator_api

    def execute(self):
        self.mediator_api.wait_to_drop()

    def get_next_state(self) -> Optional[str]:
        if not self.drone_api.is_armed:
            self.drone_api.set_resume_state("wait")  # TODO 留下/不留下?
            return "idle"

        if self.mediator_api.is_ok_to_drop:
            return "drop"
        return None
