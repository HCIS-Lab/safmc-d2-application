from abc import ABC, abstractmethod

from rclpy.clock import Clock
from agent.common.context import Context
from rclpy.impl.rcutils_logger import RcutilsLogger
from agent.api import DroneApi, MediatorApi


class Behavior(ABC):
    @staticmethod
    @abstractmethod
    def execute(context: Context):
        pass

    @staticmethod
    @abstractmethod
    def proceed(drone_api: DroneApi, mediator_api: MediatorApi, logger: RcutilsLogger, clock: Clock):
        pass
