from typing import Optional
from venv import logger

from sympy import true

from agent.api.drone_api import DroneApi
from agent.common.context import Context
from agent.constants import NAV_THRESH

from .behavior import Behavior


class WalkToHotspotBehavior(Behavior):
    @staticmethod
    def execute(context: Context):
        print ("WalkToHotspot")
        print (context.get_current_timestamp())

        drone_api: DroneApi = context.drone_api
        logger = context.logger

        hotspot_position = drone_api.get_hotspot_coord()  

        logger.info(
            f"Hotspot position: ({hotspot_position.x}, {hotspot_position.y}, {hotspot_position.z})")
        logger.info(
            f"Current position: ({drone_api.local_position.x}, {drone_api.local_position.y}, {drone_api.local_position.z})")

      
        drone_api.walk_with_avoidence(float(hotspot_position.x), float(hotspot_position.y), -1.5, context.get_current_timestamp())
        

        if (drone_api.goal_arrived(hotspot_position, NAV_THRESH)):
            drone_api.set_hotspot_reached(True)
        

    @staticmethod
    def proceed(context: Context) -> Optional[str]:
        drone_api: DroneApi = context.drone_api

        if drone_api.get_hotspot_reached():
            logger.info("hotspot reached.")
            return "drop"
            
