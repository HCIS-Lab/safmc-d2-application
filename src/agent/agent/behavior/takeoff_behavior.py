from typing import Optional

from agent.api import DroneApi
from agent.api.drone_api import NEDCoordinate
from agent.common.context import Context
from agent.constants import NAV_THRESH, TAKEOFF_HEIGHT

from .behavior import Behavior


class TakeoffBehavior(Behavior):
    altitude_reached: bool = False
    
    @staticmethod
    def execute(context: Context):
        drone_api: DroneApi = context.drone_api
        logger = context.logger

        takeoff_coord = drone_api.start_position
        takeoff_coord = NEDCoordinate(
            takeoff_coord.x,
            takeoff_coord.y,
            takeoff_coord.z - TAKEOFF_HEIGHT
        )

        logger.info(f"Target takeoff position: z={takeoff_coord.z}")
        logger.info(f"Current position: z={drone_api.local_position.z}")
        TakeoffBehavior.altitude_reached = (
            NEDCoordinate.goal_arrived(drone_api.local_position, takeoff_coord, NAV_THRESH)
        )
        if TakeoffBehavior.altitude_reached:
            logger.info("Takeoff altitude reached.")
        else:
            drone_api.publish_goto_setpoint(
                context.get_current_timestamp(), takeoff_coord)

    @staticmethod
    def proceed(context: Context) -> Optional[str]:
        drone_api: DroneApi = context.drone_api
        if TakeoffBehavior.altitude_reached:
            if drone_api.is_loaded:
                return "walk_to_hotspot"
            else:
                return "walk_to_supply"
