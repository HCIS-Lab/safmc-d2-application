from .behavior import Behavior
from agent.api.drone_api import DroneApi, NEDCoordinate
from agent.api.mediator_api import MediatorApi
from rclpy.impl.rcutils_logger import RcutilsLogger
from rclpy.clock import Clock


class WaitBehavior(Behavior):
    @staticmethod
    def execute(drone_api: DroneApi, mediator_api: MediatorApi, logger: RcutilsLogger, clock: Clock):
        target_coord = NEDCoordinate(5.0, 5.0, 1.0)
        drone_api.publish_goto_setpoint(target_coord)