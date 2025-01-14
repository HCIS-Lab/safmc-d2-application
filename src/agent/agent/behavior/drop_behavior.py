from rclpy.clock import Clock
from rclpy.impl.rcutils_logger import RcutilsLogger

from agent.api import DroneApi, MediatorApi

from .behavior import Behavior


class DropBehavior(Behavior):
    @staticmethod
    def execute(drone_api: DroneApi, mediator_api: MediatorApi, logger: RcutilsLogger, clock: Clock):
        drone_api.deactivate_magnet()
