from rclpy.clock import Clock
from rclpy.impl.rcutils_logger import RcutilsLogger

from agent.api import DroneApi, MediatorApi

from .behavior import Behavior


class WaitBehavior(Behavior):
    @staticmethod
    def execute(drone_api: DroneApi, mediator_api: MediatorApi, logger: RcutilsLogger, clock: Clock):
        mediator_api.wait_to_drop()

    @staticmethod
    @abstractmethod
    def proceed(drone_api: DroneApi, mediator_api: MediatorApi, logger: RcutilsLogger, clock: Clock):
        if mediator_api.signal():
            drop()