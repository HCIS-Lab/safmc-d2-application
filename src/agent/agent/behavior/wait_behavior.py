from typing import Optional

from api import MediatorApi
from common.context import Context

from .behavior import Behavior


class WaitBehavior(Behavior):
    def execute(self, context: Context):
        mediator_api: MediatorApi = context.mediator_api
        logger = context.logger

        logger.info("Waiting for drop signal.")
        mediator_api.wait_to_drop()

    def get_next_state(self, context: Context) -> Optional[str]:
        mediator_api: MediatorApi = context.mediator_api
        logger = context.logger

        if mediator_api.signal():
            logger.info("Drop signal received. Transitioning to DROP.")
            return "drop"
