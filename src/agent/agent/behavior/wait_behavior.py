from typing import Optional

from api import ApiRegistry, MediatorApi, Px4Api
from common.logger import Logger

from .behavior import Behavior


class WaitBehavior(Behavior):

    px4_api: Px4Api
    mediator_api: MediatorApi

    def __init__(self, logger: Logger):
        super().__init__(logger)
        self.px4_api = ApiRegistry.get(Px4Api)
        self.mediator_api = ApiRegistry.get(MediatorApi)

    def execute(self):
        self.mediator_api.wait_to_drop()

    def get_next_state(self) -> Optional[str]:
        if not self.px4_api.is_armed:
            self.px4_api.set_resume_state("wait")  # TODO[lnfu] 留下/不留下?
            return "idle"

        if self.mediator_api.is_ok_to_drop:
            return "drop"
        return None
