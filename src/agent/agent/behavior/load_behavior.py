from typing import Optional

from agent.api import DroneApi
from agent.api.drone_api import NEDCoordinate
from agent.common.context import Context
from agent.constants import LOAD_HEIGHT, NAV_THRESH, TAKEOFF_HEIGHT

from .behavior import Behavior


class LoadBehavior(Behavior):
    load_complete: bool = False 

    @staticmethod
    def execute(context: Context):
        drone_api: DroneApi = context.drone_api
        logger = context.logger

        load_position = drone_api.local_position

        if drone_api.is_loaded:
            load_position = NEDCoordinate(
                load_position.x,
                load_position.y,
                drone_api.start_position.z - TAKEOFF_HEIGHT
            )
        else:
            load_position = NEDCoordinate(
                load_position.x,
                load_position.y,
                drone_api.start_position.z - LOAD_HEIGHT
            )
        logger.info(f"Target load position: ({load_position.x}, {load_position.y}, {load_position.z})")
        logger.info(f"Current position: ({drone_api.local_position.x}, {drone_api.local_position.y}, {drone_api.local_position.z})")

        if NEDCoordinate.distance(drone_api.local_position, load_position) <= NAV_THRESH:
            
            drone_api.activate_magnet()
            
            if drone_api.is_loaded:
                logger.info("Payload successfully loaded. Ascending to takeoff height.")
                load_position = NEDCoordinate(
                    load_position.x,
                    load_position.y,
                    drone_api.start_position.z - TAKEOFF_HEIGHT
                )
                logger.info(f"Target load position: ({load_position.x}, {load_position.y}, {load_position.z})")
                logger.info(f"Current position: ({drone_api.local_position.x}, {drone_api.local_position.y}, {drone_api.local_position.z})")
                
                if NEDCoordinate.distance(drone_api.local_position, load_position) <= NAV_THRESH:
                    logger.info("Load process complete.")
                    LoadBehavior.load_complete = True
                else:
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
        if LoadBehavior.load_complete:
            return "walk_to_hotspot"
        return None
