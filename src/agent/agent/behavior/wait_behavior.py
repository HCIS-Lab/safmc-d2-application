from typing import Optional

from agent.common.context import Context

from .behavior import Behavior


class WaitBehavior(Behavior):
    @staticmethod
    def execute(context: Context):
        print("WaitBehavior")
        mediator_api = context.mediator_api
        mediator_api.wait_to_drop()

    @staticmethod
    def proceed(context: Context) -> Optional[str]:
        if context.mediator_api.signal():
            return "drop"
