from abc import ABC, abstractmethod
from agent.api import DroneApi, MediatorApi
from rclpy.impl.rcutils_logger import RcutilsLogger
from rclpy.clock import Clock


class Behavior(ABC):
    @staticmethod
    @abstractmethod
    def execute(drone_api: DroneApi, mediator_api: MediatorApi, logger: RcutilsLogger, clock: Clock):
        pass
