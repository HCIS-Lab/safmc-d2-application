from agent.api import MagnetApi
from agent.common.context import Context

from .behavior import Behavior


class DropBehavior(Behavior):
    @staticmethod
    def execute(context: Context):
        magnet_api: MagnetApi = context.magnet_api
        magnet_api.deactivate_magnet()

    @staticmethod
    def proceed(context: Context, agent_machine):
        magnet_api: MagnetApi = context.magnet_api
        if not magnet_api.is_loaded:
            agent_machine.walk_to_supply()
