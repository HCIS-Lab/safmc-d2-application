from rclpy.clock import Clock
from rclpy.impl.rcutils_logger import RcutilsLogger

from agent.api import DroneApi, MediatorApi
from agent.api.drone_api import NEDCoordinate
from agent.constants import NAV_THRESH, TAKEOFF_HEIGHT

from .behavior import Behavior


class TakeoffBehavior(Behavior):
    @staticmethod
    def execute(drone_api: DroneApi, mediator_api: MediatorApi, logger: RcutilsLogger, clock: Clock):

        takeoff_coord = drone_api.start_position
        takeoff_coord = NEDCoordinate(
            takeoff_coord.x,
            takeoff_coord.y,
            takeoff_coord.z - TAKEOFF_HEIGHT
        )

        if (drone_api.goal_arrived(takeoff_coord, NAV_THRESH)):
            drone_api.set_altitude_reached(True)
        else:
            drone_api.publish_goto_setpoint(
                clock.now().nanoseconds, takeoff_coord)
