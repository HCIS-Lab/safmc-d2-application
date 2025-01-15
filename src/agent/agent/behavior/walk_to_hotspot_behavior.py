from .behavior import Behavior
from agent.api.drone_api import DroneApi
from agent.constants import NAV_THRESH
from agent.common.context import Context
from typing import Optional


class WalkToHotspotBehavior(Behavior):
    @staticmethod
    def execute(context: Context):

        drone_api: DroneApi = context.drone_api

        hotspot_coord = drone_api.get_hotspot_coord()
        if drone_api.goal_arrived(hotspot_coord[0], NAV_THRESH):
            drone_api.set_hotspot_reached(True)
        else:
            drone_api.publish_goto_setpoint(
                context.get_current_timestamp(), hotspot_coord[0])
            drone_api.set_hotspot_reached(False)

    @staticmethod
    def proceed(context: Context) -> Optional[str]:
        drone_api: DroneApi = context.drone_api

        if drone_api.get_hotspot_reached():
            return "drop"
