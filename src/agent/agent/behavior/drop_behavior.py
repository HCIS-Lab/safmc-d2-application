
from agent.api import DroneApi, MediatorApi
from rclpy.impl.rcutils_logger import RcutilsLogger
from rclpy.clock import Clock
from agent.common.context import Context

from .behavior import Behavior


class DropBehavior(Behavior):
    @staticmethod
    def execute(context: Context):
        drone_api: DroneApi = context.drone_api
        drone_api.drop_payload()

    @staticmethod
    def proceed(drone_api: DroneApi, mediator_api: MediatorApi, logger: RcutilsLogger, clock: Clock):
        if not drone_api.is_loaded:
            walk_to_supply()
