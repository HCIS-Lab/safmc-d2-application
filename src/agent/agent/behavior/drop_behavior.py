from typing import Optional

from api import DroneApi, MagnetApi, MediatorApi
from common.logger import Logger

from .behavior import Behavior


class DropBehavior(Behavior):

    def __init__(self, logger: Logger, drone_api: DroneApi, mediator_api: MediatorApi, magnet_api: MagnetApi):
        super().__init__(logger)
        self.drone_api = drone_api
        self.mediator_api = mediator_api
        self.magnet_api = magnet_api

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
