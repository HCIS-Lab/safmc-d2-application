from typing import Optional

from agent.api import MagnetApi
from agent.common.context import Context

from .behavior import Behavior


class DropBehavior(Behavior):
    @staticmethod
    def execute(context: Context):
        print("DropBehavior")
        magnet_api: MagnetApi = context.magnet_api
        magnet_api.deactivate_magnet()

    @staticmethod
    def proceed(context: Context) -> Optional[str]:
        magnet_api: MagnetApi = context.magnet_api
        if not magnet_api.is_loaded:
            return "walk_to_supply"
        return None
