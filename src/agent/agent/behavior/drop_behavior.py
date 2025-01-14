from .behavior import Behavior

from api import DroneApi, MediatorApi
from rclpy.impl.rcutils_logger import RcutilsLogger
from rclpy.clock import Clock


class DropBehavior(Behavior):
    @staticmethod
    def execute(drone_api: DroneApi, mediator_api: MediatorApi, logger: RcutilsLogger, clock: Clock):
        drone_api.drop_payload()
