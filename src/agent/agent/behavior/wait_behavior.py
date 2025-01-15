
from rclpy.clock import Clock
from rclpy.impl.rcutils_logger import RcutilsLogger

from agent.api import DroneApi, MediatorApi
from agent.common.context import Context

from .behavior import Behavior


class WaitBehavior(Behavior):
    @staticmethod
    def execute(context: Context):
        mediator_api = context.mediator_api
        mediator_api.wait_to_drop()

    @staticmethod
    def proceed(context: Context, agent_machine):
        if context.mediator_api.signal():
            agent_machine.drop()
