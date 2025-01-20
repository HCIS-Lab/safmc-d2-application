from typing import Optional

from api import MagnetApi
from common.context import Context

from .behavior import Behavior


class DropBehavior(Behavior):
    @staticmethod
    def execute(context: Context):
        magnet_api: MagnetApi = context.magnet_api
        logger = context.logger

        logger.info("Deactivating magnet to drop payload.")
        magnet_api.deactivate_magnet()

    @staticmethod
    def proceed(context: Context) -> Optional[str]:
        magnet_api: MagnetApi = context.magnet_api
        logger = context.logger

        logger.info(f"Payload loaded status: {magnet_api.is_loaded}")
        if not magnet_api.is_loaded:
            logger.info(
                "Payload successfully dropped. Transitioning to WALK_TO_SUPPLY.")
            return "walk_to_supply"
        return None
