
from agent_machine import AgentMachine
from rclpy.clock import Clock
from rclpy.impl.rcutils_logger import RcutilsLogger

from agent.api import DroneApi, MediatorApi, MagnetApi
from agent.common.context import Context

from .behavior import Behavior


class DropBehavior(Behavior):
    @staticmethod
    def execute(context: Context):
        magnet_api: MagnetApi = context.magnet_api
        magnet_api.deactivate_magnet()

    @staticmethod
    def proceed(context: Context, agent_machine: AgentMachine):
        magnet_api: MagnetApi = context.magnet_api
        if not magnet_api.is_loaded:
            agent_machine.walk_to_supply()
