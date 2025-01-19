from typing import Optional

from agent.api import DroneApi
from agent.common.context import Context
from agent.constants import NAV_THRESH

from .behavior import Behavior


class WalkToSupplyBehavior(Behavior):
    @staticmethod
    def execute(context: Context):
        
        drone_api: DroneApi = context.drone_api

        supply_coord = drone_api.get_supply_coord()

        if drone_api.goal_arrived(supply_coord, NAV_THRESH):
            drone_api.set_supply_reached(True)
        else:
            print ("WalkToSupplyBehavior execute")
            drone_api.maintain_offboard_control(context.get_current_timestamp())
            drone_api.publish_position_setpoint(float(supply_coord.x), float(supply_coord.y), float(supply_coord.z), context.get_current_timestamp())
            
            

    @staticmethod
    def proceed(context: Context) -> Optional[str]:
        if context.drone_api.get_supply_reached():
            return "load"
