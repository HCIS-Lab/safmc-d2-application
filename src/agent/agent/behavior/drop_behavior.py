from typing import Optional

from api import ApiRegistry, DroneApi, MagnetApi, MediatorApi
from common.logger import Logger

from .behavior import Behavior


class DropBehavior(Behavior):

    drone_api: DroneApi
    magnet_api: MagnetApi
    mediator_api: MediatorApi

    def __init__(self, logger: Logger):
        super().__init__(logger)
        self.drone_api = ApiRegistry.get(DroneApi)
        self.magnet_api = ApiRegistry.get(MagnetApi)
        self.mediator_api = ApiRegistry.get(MediatorApi)

    def execute(self):
        self.logger.info("Deactivating magnet to drop payload.")
        self.logger.info(f"Load status: {self.magnet_api.is_loaded}")
        self.magnet_api.deactivate_magnet()

    def get_next_state(self) -> Optional[str]:
        if not self.drone_api.is_armed:
            self.drone_api.set_resume_state("drop")  # TODO 留下/不留下?
            return "idle"

        if not self.magnet_api.is_loaded:
            return "walk_to_supply"
        return None
