from typing import Optional

from agent.constants import LOAD_HEIGHT, NAV_THRESH, TAKEOFF_HEIGHT
from api import DroneApi, MagnetApi
from common.context import Context
from common.ned_coordinate import NEDCoordinate

from .behavior import Behavior


class LoadBehavior(Behavior):

    @staticmethod
    def execute(context: Context):
        drone_api: DroneApi = context.drone_api
        magnet_api: MagnetApi = context.magnet_api
        logger = context.logger

        load_position = drone_api.local_position

        if drone_api.is_loaded:
            load_position.z = drone_api.start_position.z - TAKEOFF_HEIGHT
        else:
            load_position.z = drone_api.start_position.z - LOAD_HEIGHT

        logger.info(
            f"Target load position: ({load_position.x}, {load_position.y}, {load_position.z})")
        logger.info(
            f"Current position: ({drone_api.local_position.x}, {drone_api.local_position.y}, {drone_api.local_position.z})")

        if NEDCoordinate.distance(drone_api.local_position, load_position) <= NAV_THRESH:

            magnet_api.activate_magnet()

            if drone_api.is_loaded:
                logger.info(
                    "Payload successfully loaded. Ascending to takeoff height.")
                load_position.z = drone_api.start_position.z - TAKEOFF_HEIGHT

                logger.info(
                    f"Target load position: ({load_position.x}, {load_position.y}, {load_position.z})")
                logger.info(
                    f"Current position: ({drone_api.local_position.x}, {drone_api.local_position.y}, {drone_api.local_position.z})")

                drone_api.publish_goto_setpoint(
                    context.get_current_timestamp(), load_position)
            else:
                logger.info("Reached loading position. Activating magnet.")
        else:
            logger.info("Descending to loading position.")
            drone_api.publish_goto_setpoint(
                context.get_current_timestamp(), load_position)

    @staticmethod
    def proceed(context: Context) -> Optional[str]:

        drone_api: DroneApi = context.drone_api
        logger = context.logger

        load_position = drone_api.local_position
        load_position.z = drone_api.start_position.z - TAKEOFF_HEIGHT

        if NEDCoordinate.distance(drone_api.local_position, load_position) <= NAV_THRESH and drone_api.is_loaded:
            logger.info("Load process complete.")
            return "walk_to_hotspot"
        return None
