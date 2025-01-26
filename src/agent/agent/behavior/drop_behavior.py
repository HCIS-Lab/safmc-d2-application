from typing import Optional

from api import MagnetApi
from common.logger import Logger

from .behavior import Behavior


class DropBehavior(Behavior):

    def __init__(self, logger: Logger, magnet_api: MagnetApi):
        super().__init__(logger)
        self.magnet_api = magnet_api

    def execute(self):
        self.logger.info("deactivating magnet to drop payload.")
        self.logger.info(f"load status: {self.magnet_api.is_loaded}")
        self.magnet_api.deactivate_magnet()

    def get_next_state(self) -> Optional[str]:
        if not self.magnet_api.is_loaded:
            return "walk_to_supply"
        return None
