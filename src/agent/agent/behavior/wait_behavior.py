from typing import Optional

from common.context import Context
from api import MediatorApi


from .behavior import Behavior


class WaitBehavior(Behavior):
    @staticmethod
    def execute(context: Context):
        mediator_api: MediatorApi = context.mediator_api
        logger = context.logger

        logger.info("Waiting for drop signal.")
        mediator_api.wait_to_drop()

    @staticmethod
    def proceed(context: Context) -> Optional[str]:
        mediator_api: MediatorApi = context.mediator_api
        logger = context.logger

        if mediator_api.signal():
            logger.info("Drop signal received. Transitioning to DROP.")
            return "drop"
