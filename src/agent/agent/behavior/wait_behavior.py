from typing import Optional

from api import ApiRegistry, MediatorApi, Px4Api
from common.logger import Logger

from .behavior import Behavior


class WaitBehavior(Behavior):

    drone_api: Px4Api
    mediator_api: MediatorApi

    def __init__(self, logger: Logger):
        super().__init__(logger)
        self.drone_api = ApiRegistry.get(Px4Api)
        self.mediator_api = ApiRegistry.get(MediatorApi)

    def execute(self):
        self.mediator_api.wait_to_drop()

    def get_next_state(self) -> Optional[str]:
        if not self.drone_api.is_armed:
            self.drone_api.set_resume_state("wait")  # TODO 留下/不留下?
            return "idle"

        if self.mediator_api.is_ok_to_drop:
            return "drop"
        return None
