from .behavior import Behavior
from agent.api import DroneApi, MediatorApi
from agent.api.drone_api import NEDCoordinate
from agent.constants import LOAD_HEIGHT, TAKEOFF_HEIGHT, NAV_THRESH


class LoadBehavior(Behavior):
    @staticmethod
    def execute(drone_api: DroneApi, mediator_api: MediatorApi):

        load_coord = drone_api.get_local_coord()
        load_coord = NEDCoordinate(
            load_coord.x,
            load_coord.y,
            -LOAD_HEIGHT
        )

        if (drone_api.goal_arrived(load_coord, NAV_THRESH)):
            
            drone_api.grab_payload()# grab
            if drone_api.is_grabbed():
                load_coord = drone_api.get_local_coord()
                load_coord = NEDCoordinate(
                    load_coord.x,
                    load_coord.y,
                    LOAD_HEIGHT - TAKEOFF_HEIGHT
                )

                if (drone_api.goal_arrived(load_coord, NAV_THRESH)):
                    drone_api.has_loaded()
                else:
                    drone_api.publish_goto_setpoint(load_coord)

        else:
            drone_api.publish_goto_setpoint(load_coord)