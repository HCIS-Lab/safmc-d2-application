
from agent_machine import AgentMachine
from rclpy.clock import Clock
from rclpy.impl.rcutils_logger import RcutilsLogger

from agent.api import DroneApi, MediatorApi
from agent.common.context import Context

from .behavior import Behavior


class DropBehavior(Behavior):
    @staticmethod
    def execute(context: Context):
        drone_api: DroneApi = context.drone_api
        drone_api.drop_payload()

    @staticmethod
    def proceed(context: Context, agent_machine: AgentMachine):
        if not context.drone_api.is_loaded:
            agent_machine.walk_to_supply()
