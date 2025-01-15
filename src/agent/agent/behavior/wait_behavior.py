from agent.api import DroneApi, MediatorApi
from rclpy.impl.rcutils_logger import RcutilsLogger
from rclpy.clock import Clock
from agent.common.context import Context

from .behavior import Behavior
from agent_machine import AgentMachine


class WaitBehavior(Behavior):
    @staticmethod
    def execute(context: Context):
        mediator_api = context.mediator_api
        mediator_api.wait_to_drop()

    @staticmethod
    def proceed(drone_api: DroneApi, mediator_api: MediatorApi, logger: RcutilsLogger, clock: Clock, agent_machine: AgentMachine):
        if mediator_api.signal():
            agent_machine.drop()
