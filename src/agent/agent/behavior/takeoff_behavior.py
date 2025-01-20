from typing import Optional

from agent.constants import NAV_THRESH, TAKEOFF_HEIGHT
from api import DroneApi
from common.context import Context
from common.ned_coordinate import NEDCoordinate

from .behavior import Behavior


class TakeoffBehavior(Behavior):

    @staticmethod
    def execute(context: Context):
        drone_api: DroneApi = context.drone_api
        logger = context.logger

        takeoff_position = drone_api.start_position - NEDCoordinate.down * TAKEOFF_HEIGHT

        logger.info(f"Target takeoff position: z={takeoff_position.z}")
        logger.info(f"Current position: z={drone_api.local_position.z}")
        drone_api.publish_goto_setpoint(
            context.get_current_timestamp(), takeoff_position)

    @staticmethod
    def proceed(context: Context) -> Optional[str]:
        drone_api: DroneApi = context.drone_api
        logger = context.logger

        takeoff_position = drone_api.start_position - NEDCoordinate.down * TAKEOFF_HEIGHT

        if NEDCoordinate.distance(drone_api.local_position, takeoff_position) <= NAV_THRESH:
            logger.info("Takeoff altitude reached.")
            return "walk_to_hotspot" if drone_api.is_loaded else "walk_to_supply"

        return None
