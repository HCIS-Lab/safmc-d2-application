from typing import Optional

from agent.api import DroneApi
from agent.api.drone_api import NEDCoordinate
from agent.common.context import Context
from agent.constants import NAV_THRESH, TAKEOFF_HEIGHT

from .behavior import Behavior


class TakeoffBehavior(Behavior):
    
    @staticmethod
    def execute(context: Context):
        drone_api: DroneApi = context.drone_api
        logger = context.logger

        takeoff_coord = drone_api.start_position - NEDCoordinate.down * TAKEOFF_HEIGHT

        logger.info(f"Target takeoff position: z={takeoff_coord.z}")
        logger.info(f"Current position: z={drone_api.local_position.z}")

        drone_api.publish_goto_setpoint(context.get_current_timestamp(), takeoff_coord)

    @staticmethod
    def proceed(context: Context) -> Optional[str]:

        drone_api: DroneApi = context.drone_api
        logger = context.logger
        takeoff_coord = drone_api.start_position - NEDCoordinate.down * TAKEOFF_HEIGHT

        if NEDCoordinate.distance(drone_api.local_position, takeoff_coord) <= NAV_THRESH:
            logger.info("Takeoff altitude reached.")
            if drone_api.is_loaded:
                return "walk_to_hotspot"
            else:
                return "walk_to_supply"
