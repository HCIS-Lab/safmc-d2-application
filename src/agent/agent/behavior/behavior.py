from abc import ABC, abstractmethod

# TODO: come up with a solution - avoid passing agent_machine directly
from agent.agent_machine import AgentMachine
from rclpy.clock import Clock
from rclpy.impl.rcutils_logger import RcutilsLogger

from agent.api import DroneApi, MediatorApi
from agent.common.context import Context


class Behavior(ABC):
    @staticmethod
    @abstractmethod
    def execute(context: Context):
        pass

    @staticmethod
    @abstractmethod
    def proceed(context: Context, agent_machine: AgentMachine):
        pass
