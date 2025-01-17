from typing import Optional

from agent.api import DroneApi
from agent.api.drone_api import NEDCoordinate
from agent.common.context import Context
from agent.constants import NAV_THRESH

from .behavior import Behavior


class WalkToSupplyBehavior(Behavior):

    @staticmethod
    def execute(context: Context):
        drone_api: DroneApi = context.drone_api
        logger = context.logger

        supply_position = drone_api.get_supply_position()
        supply_position.z = drone_api.local_position.z

        logger.info(f"Supply position: ({supply_position.x}, {supply_position.y}, {supply_position.z})")
        logger.info(f"Current position: ({drone_api.local_position.x}, {drone_api.local_position.y}, {drone_api.local_position.z})")

        drone_api.publish_goto_setpoint(context.get_current_timestamp(), supply_position)

    @staticmethod
    def proceed(context: Context) -> Optional[str]:

        drone_api: DroneApi = context.drone_api
        logger = context.logger
        supply_position = drone_api.get_supply_position()
        supply_position.z = drone_api.local_position.z

        if NEDCoordinate.distance(drone_api.local_position, supply_position) <= NAV_THRESH:
            logger.info("Supply reached.")
            return "load"
