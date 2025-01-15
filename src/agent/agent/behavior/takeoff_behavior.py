from typing import Optional

from agent.api import DroneApi, MagnetApi
from agent.api.drone_api import NEDCoordinate
from agent.common.context import Context
from agent.constants import NAV_THRESH, TAKEOFF_HEIGHT

from .behavior import Behavior


class TakeoffBehavior(Behavior):
    @staticmethod
    def execute(context: Context):
        drone_api: DroneApi = context.drone_api

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
                context.get_current_timestamp(), takeoff_coord)

    @staticmethod
    def proceed(context: Context) -> Optional[str]:
        drone_api: DroneApi = context.drone_api
        magnet_api: MagnetApi = context.magnet_api
        if drone_api.is_altitude_reached:
            if magnet_api.is_loaded:
                return "walk_to_hotspot"
            else:
                return "walk_to_supply"
