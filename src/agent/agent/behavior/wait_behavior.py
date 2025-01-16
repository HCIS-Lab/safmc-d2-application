from typing import Optional

from agent.common.context import Context

from .behavior import Behavior


class WaitBehavior(Behavior):
    @staticmethod
    def execute(context: Context):
        mediator_api = context.mediator_api
        logger = context.logger
        logger.info("Waiting for drop signal.")
        mediator_api.wait_to_drop()

    @staticmethod
    def proceed(context: Context) -> Optional[str]:
        logger = context.logger
        if context.mediator_api.signal():
            logger.info("Drop signal received. Transitioning to DROP.")
            return "drop"
