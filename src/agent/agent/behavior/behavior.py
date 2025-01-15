from abc import ABC, abstractmethod

from rclpy.clock import Clock
from agent.common.context import Context
from rclpy.impl.rcutils_logger import RcutilsLogger
from agent.api import DroneApi, MediatorApi

# TODO: come up with a solution - avoid passing agent_machine directly
from agent_machine import AgentMachine


class Behavior(ABC):
    @staticmethod
    @abstractmethod
    def execute(context: Context):
        pass

    @staticmethod
    @abstractmethod
    def proceed(drone_api: DroneApi, mediator_api: MediatorApi, logger: RcutilsLogger, clock: Clock, agent_machine: AgentMachine):
        pass
