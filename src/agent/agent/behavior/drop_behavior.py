from typing import Optional

from api import ApiRegistry, MagnetApi, MediatorApi, Px4Api
from common.logger import Logger

from .behavior import Behavior


class DropBehavior(Behavior):

    px4_api: Px4Api
    magnet_api: MagnetApi
    mediator_api: MediatorApi

    def __init__(self, logger: Logger):
        super().__init__(logger)
        self.px4_api = ApiRegistry.get(Px4Api)
        self.magnet_api = ApiRegistry.get(MagnetApi)
        self.mediator_api = ApiRegistry.get(MediatorApi)

    def execute(self):
        self.logger.info("Deactivating magnet to drop payload.")
        self.logger.info(f"Load status: {self.magnet_api.is_loaded}")
        self.magnet_api.deactivate_magnet()

    def get_next_state(self) -> Optional[str]:
        if not self.px4_api.is_armed:
            self.px4_api.set_resume_state("drop")  # TODO[lnfu] 留下/不留下?
            return "idle"

        if not self.magnet_api.is_loaded:
            return "walk_to_supply"
        return None
