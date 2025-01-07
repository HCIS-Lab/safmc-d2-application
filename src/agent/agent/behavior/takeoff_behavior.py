from .behavior import Behavior
from agent.api import DroneApi, MediatorApi
from agent.api.drone_api import NEDCoordinate
from agent.constants import TAKEOFF_HEIGHT,NAV_THRESH


class TakeoffBehavior(Behavior):
    @staticmethod
    def execute(drone_api: DroneApi, mediator_api: MediatorApi):

        takeoff_coord = drone_api.get_home_coord()
        takeoff_coord = NEDCoordinate(
            takeoff_coord.x,
            takeoff_coord.y,
            takeoff_coord.z - TAKEOFF_HEIGHT
        )

        if (drone_api.goal_arrived(takeoff_coord, NAV_THRESH)):
            drone_api.set_altitude_reached(True)
        else:
            drone_api.publish_goto_setpoint(takeoff_coord)
